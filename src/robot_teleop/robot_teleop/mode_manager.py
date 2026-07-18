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

        # controller-presence watchdog (same shape as balance_controller's
        # cmd_vel watchdog): stamp the time each /joy message arrives, then
        # judge freshness. None = we have never heard a controller, i.e. the
        # boot-with-no-controller case.
        self.last_joy_time = None
        self.joy_timeout = 0.5   # s; joy_node autorepeats at 20 Hz (~50 ms)

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

    def joy_callback(self, msg):
        # the message content doesn't matter here — its arrival is the signal
        # that a controller is alive.
        self.last_joy_time = self.node.get_clock().now()

    def controller_present(self):
        if self.last_joy_time is None:
            return False
        age = (self.node.get_clock().now() - self.last_joy_time).nanoseconds * 1e-9
        return age < self.joy_timeout

    def publish_mode(self):
        self.mode_pub.publish(String(data=self.current_mode.name.lower()))

    def set_mode(self, mode):
        self.current_mode = mode
        self.publish_mode()
        self.node.get_logger().info(f'mode -> {mode.name.lower()}')

    def set_mode_callback(self, request, response):
        # 1. validate the requested string against the known modes. an unknown
        #    name is exactly the reject case a service (vs a topic) exists for.
        try:
            target = Mode[request.mode.upper()]
        except KeyError:
            response.success = False
            response.message = f"unknown mode '{request.mode}'"
            return response

        # 2. interlock: entering a motion mode requires a live controller.
        if target in MOTION_MODES and not self.controller_present():
            response.success = False
            response.message = 'no controller present'
            return response

        # 3. accept the change.
        self.set_mode(target)
        response.success = True
        response.message = f'mode set to {target.name.lower()}'
        return response


def main(args=None):
    rclpy.init(args=args)
    mode_manager = ModeManager()
    rclpy.spin(mode_manager.node)
    rclpy.shutdown()
