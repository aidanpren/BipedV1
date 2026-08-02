from enum import Enum

import rclpy
from rclpy.qos import QoSProfile, DurabilityPolicy
from std_msgs.msg import String
from sensor_msgs.msg import Joy
from robot_interfaces.srv import SetMode


class Mode(Enum):
    DISABLED = 0
    TELEOP = 1
    AUTONOMOUS = 2


# modes that command motion — entering one requires a live controller.
# DISABLED never does: you must always be able to stop the robot.
MOTION_MODES = {Mode.TELEOP, Mode.AUTONOMOUS}


class ModeManager:
    def __init__(self):
        self.node = rclpy.create_node('mode_manager')

        # ── controller buttons ────────────────────────────────────────────────
        # THESE INDICES MUST BE MEASURED, NOT LOOKED UP. DS4 button numbering
        # differs between the SDL and joydev backends, and a wrong index does
        # not error — the button simply never fires, which is indistinguishable
        # from "the feature is broken". Run `python3 tools/joy_probe.py` (or
        # `ros2 topic echo /joy`) and press the button you want.
        #
        # Defaults below are the common joydev DS4 layout and are a STARTING
        # GUESS ONLY:  4=L1  5=R1  6=L2  7=R2  9=Options
        #
        # They live in real.yaml/sim.yaml (loaded from SOURCE via --params-file),
        # not in a share-installed yaml, specifically so that finding the right
        # index is edit-and-restart rather than edit-and-colcon-build.
        self.node.declare_parameter('teleop_button', 9)          # Options

        # DISABLED needs a COMBO, deliberately. Entering teleop is a single tap;
        # leaving it cuts torque and drops the robot, so it must not be
        # reachable by one fat finger mid-drive. Every listed button must be
        # held at once. L1+L2 keeps the mode gestures on the left hand, clear of
        # teleop_twist_joy's enable(R2)/turbo(R1) on the right.
        self.node.declare_parameter('disable_buttons', [4, 6])   # L1 + L2

        # NOTE if L2 does not appear as a BUTTON on your driver: it is an
        # analog trigger and some backends expose it only as an AXIS. joy_probe
        # will show you which. If it is axis-only, pick a different button here
        # rather than adding axis thresholding — a digital combo is easier to
        # reason about than a threshold that can drift.

        # controller-presence watchdog (same shape as balance_controller's
        # cmd_vel watchdog): stamp the time each /joy message arrives, then
        # judge freshness. None = we have never heard a controller, i.e. the
        # boot-with-no-controller case.
        self.last_joy_time = None
        self.joy_timeout = 0.5   # s; joy_node autorepeats at 20 Hz (~50 ms)

        # edge-detection state. joy_node AUTOREPEATS at 20 Hz, so a held button
        # arrives as a continuous stream of identical messages. Acting on the
        # level would re-fire the transition twenty times a second; we act only
        # on the rising edge.
        self.prev_teleop_button = False
        self.prev_disable_combo = False
        self.warned_short_joy = False

        # latched (transient-local, depth 1) so a late-joining dashboard learns
        # the current mode the instant it connects.
        latched_qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.mode_pub = self.node.create_publisher(String, 'mode', latched_qos)

        # boot into DISABLED (motors off); publish it immediately so the
        # latched topic has a value from the start.
        self.current_mode = Mode.DISABLED
        self.publish_mode()

        self.srv = self.node.create_service(
            SetMode, 'set_mode', self.set_mode_callback)
        self.joy_sub = self.node.create_subscription(
            Joy, 'joy', self.joy_callback, 10)

    # ── joystick ─────────────────────────────────────────────────────────────
    def joy_callback(self, msg):
        # Stamp presence FIRST, before any button handling. The interlock in
        # request_mode() asks whether a controller is live, and the message we
        # are holding IS that evidence — stamping afterwards would make a
        # button press race its own liveness check.
        self.last_joy_time = self.node.get_clock().now()

        teleop_btn = int(self.node.get_parameter('teleop_button').value)
        disable_btns = list(self.node.get_parameter('disable_buttons').value)

        # A short buttons array means a different pad, or a driver reporting a
        # layout we did not expect. Reading past the end would raise inside a
        # subscription callback on every message. Warn ONCE and do nothing —
        # silently ignoring it would hide a wrong index, and spamming would
        # bury everything else in the journal.
        needed = max([teleop_btn] + list(disable_btns)) if disable_btns else teleop_btn
        if len(msg.buttons) <= needed:
            if not self.warned_short_joy:
                self.warned_short_joy = True
                self.node.get_logger().warn(
                    f'/joy has {len(msg.buttons)} buttons but the configured '
                    f'indices need at least {needed + 1}. Mode buttons are '
                    f'INACTIVE. Check teleop_button / disable_buttons against '
                    f'`python3 tools/joy_probe.py`.')
            return

        # DISABLE is checked FIRST. If a configuration ever made both gestures
        # true in the same message, the safe-direction one must win.
        combo = all(bool(msg.buttons[i]) for i in disable_btns)
        if combo and not self.prev_disable_combo:
            self.request_mode(Mode.DISABLED, source='joy combo')
        self.prev_disable_combo = combo

        pressed = bool(msg.buttons[teleop_btn])
        # Do not let the teleop tap fire while the disable combo is held, in
        # case someone configures overlapping buttons.
        if pressed and not self.prev_teleop_button and not combo:
            self.request_mode(Mode.TELEOP, source='joy button')
        self.prev_teleop_button = pressed

    def controller_present(self):
        if self.last_joy_time is None:
            return False
        age = (self.node.get_clock().now() - self.last_joy_time).nanoseconds * 1e-9
        return age < self.joy_timeout

    # ── mode policy ──────────────────────────────────────────────────────────
    def request_mode(self, target, source):
        """The ONE place a mode transition is policed.

        Both entry points — the SetMode service and the controller buttons —
        come through here. That is the whole point of the function: if the
        button handler called set_mode() directly it would bypass the
        motion-mode interlock, and the robot would have two different sets of
        rules depending on who asked.

        Returns (success, message) so the service can report it verbatim.
        """
        if target in MOTION_MODES and not self.controller_present():
            msg = 'no controller present'
            self.node.get_logger().warn(f'{source}: refused {target.name.lower()} — {msg}')
            return False, msg

        if target is self.current_mode:
            # Not an error. A second tap of the teleop button is a no-op, not a
            # failure, and it must not spam the latched topic.
            return True, f'already {target.name.lower()}'

        self.set_mode(target, source)
        return True, f'mode set to {target.name.lower()}'

    def publish_mode(self):
        self.mode_pub.publish(String(data=self.current_mode.name.lower()))

    def set_mode(self, mode, source='service'):
        self.current_mode = mode
        self.publish_mode()
        self.node.get_logger().info(f'mode -> {mode.name.lower()}  ({source})')

    def set_mode_callback(self, request, response):
        # 1. validate the requested string against the known modes. an unknown
        #    name is exactly the reject case a service (vs a topic) exists for.
        try:
            target = Mode[request.mode.upper()]
        except KeyError:
            response.success = False
            response.message = f"unknown mode '{request.mode}'"
            return response

        # 2. interlock + apply, through the same policy the buttons use.
        response.success, response.message = self.request_mode(
            target, source='set_mode service')
        return response


def main(args=None):
    rclpy.init(args=args)
    mode_manager = ModeManager()
    rclpy.spin(mode_manager.node)
    rclpy.shutdown()
