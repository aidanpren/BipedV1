"""Drop-in replacement for Gazebo + gz_ros2_control on the real robot.

Speaks the SAME topics the sim stack does, so balance_controller cannot tell
the difference:

    subscribes  wheel_effort_controller/commands  Float64MultiArray [L, R] Nm at the WHEEL
    publishes   joint_states                      JointState (radians, rad/s)

Test it with no hardware:
    sudo modprobe vcan
    sudo ip link add dev vcan0 type vcan && sudo ip link set up vcan0
    python3 tools/fake_odrive.py --channel vcan0 --node-id 1 &
    python3 tools/fake_odrive.py --channel vcan0 --node-id 2 &
    ros2 run robot_base odrive_bridge
"""
import math
import threading
import time

import can
import rclpy
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray

from robot_base.odrive_can import (
    CMD_GET_ENCODER_ESTIMATES, AxisState, ControlMode, InputMode, ODriveClient,
)

TURNS_TO_RAD = 2 * math.pi


# ── friction compensation (2026-08-02) ──────────────────────────────────────
# Symptom that motivated it: the robot balances in place but cannot hold still.
# It leans, the wheels do not respond, it leans further, the wheels break free
# and over-correct. Bounded hunting, random direction, always recovers — the
# signature of a symmetric stiction deadband, NOT a bias (a bias drifts the same
# way every time).
#
# WHY THIS LIVES IN THE BRIDGE AND NOT IN balance_controller. This is actuator
# LINEARISATION, not control: it makes the motor behave like the ideal torque
# source the balance law already assumes it is. It also needs PER-WHEEL
# velocity, and balance_controller only tracks the average of the two.
#
# Both effects default to 0.0 — installing this changes NOTHING until the
# values are measured on the robot and set deliberately.

def friction_feedforward(vel, tau_c, v_eps):
    """Coulomb friction feedforward: push in the direction of travel.

    Tapered through zero rather than a hard sign(v), because sign() chatters
    at the zero crossing and the wheels sit at v=0 constantly on a balancing
    robot.

    IMPORTANT AND EASY TO MISREAD: this returns ~0 at v~0, so it does NOT fix
    standstill hunting. Friction opposes MOTION; there is no motion to oppose
    yet. This term is what makes DRIVING smooth. Dither is what deals with
    standstill. They are not alternatives — they cover different regimes.
    """
    if tau_c <= 0.0 or v_eps <= 0.0:
        return 0.0
    return tau_c * max(-1.0, min(1.0, vel / v_eps))


def dither(t, amp, hz):
    """Square-wave dither used to linearise stiction at standstill.

    Keeping the wheel micro-moving means there is no STATIC friction left to
    break: the motor only ever fights the (lower, smoother) Coulomb friction.
    Square rather than sine because time spent at full amplitude is what breaks
    stiction, and a square spends all of it there.

    A PHASE ACCUMULATOR, not `sin(2*pi*f*t) >= 0`. That difference is not
    stylistic — the sine form is broken at exactly the frequencies we want.
    Sampling it at 100 Hz with hz=50 evaluates sin(pi*i), which is 0 for every
    integer i, so the sign falls out of floating-point rounding noise instead
    of the waveform. Measured result: 16 high / 24 low and a -0.04 Nm MEAN,
    i.e. a DC torque bias on the wheels — the exact failure mode we ruled out
    as the cause of the hunting. The accumulator lands cleanly on both 25 Hz
    (+ + - -) and 50 Hz (+ - + -).

    Frequency must sit far above the balance loop (~1.6 rad/s ceiling) but with
    enough samples per cycle to survive command-rate jitter. 100 Hz commands
    make 50 Hz Nyquist-exact and therefore fragile: the BNO085 drops samples
    over I2C, so the real rate is NOT a clean 100 Hz. 25 Hz gives four samples
    per cycle and degrades gracefully. Driven off wall time, not a call
    counter, so the FREQUENCY stays correct however the command rate jitters.
    """
    if amp <= 0.0 or hz <= 0.0:
        return 0.0
    return amp if (t * hz) % 1.0 < 0.5 else -amp


# Dither is applied in ANTIPHASE: +1 on the left wheel, -1 on the right.
#
# This matters more than it looks. In phase, the two wheels shake the robot
# fore-and-aft — injecting a disturbance straight into the pitch loop we are
# trying to help. In antiphase the translational components cancel exactly and
# what is left is a tiny yaw wiggle, and the balance loop is blind to yaw.
# Each wheel still individually sees the full +/-amp alternation, so stiction
# breaking is unaffected: friction is a per-wheel phenomenon.
DITHER_PHASE = {'left': +1.0, 'right': -1.0}


class ODriveBridge:
    def __init__(self):
        self.node = rclpy.create_node('odrive_bridge')

        # every tunable is a parameter — flip can_channel to the real bus on the robot
        self.node.declare_parameter('can_interface', 'socketcan')
        self.node.declare_parameter('can_channel', 'vcan0')
        self.node.declare_parameter('bitrate', 500000)
        self.node.declare_parameter('left_node_id', 1)
        self.node.declare_parameter('right_node_id', 2)
        self.node.declare_parameter('gear_ratio', 8.0)
        # the two motors are physically mirrored, so one side must be inverted
        self.node.declare_parameter('invert_left', False)
        self.node.declare_parameter('invert_right', True)
        self.node.declare_parameter('current_limit', 12.0)   # A
        self.node.declare_parameter('vel_limit', 20.0)       # output rev/s
        self.node.declare_parameter('publish_rate', 50.0)    # Hz
        self.node.declare_parameter('cmd_timeout', 0.5)      # s

        # Friction compensation. BOTH DEFAULT OFF — see the module header.
        # Read live every command so they can be tuned with `ros2 param set`
        # while the robot is balancing, without a restart.
        self.node.declare_parameter('friction_ff', 0.0)      # Nm at the WHEEL
        self.node.declare_parameter('friction_v_eps', 0.2)   # rad/s taper width
        self.node.declare_parameter('dither_torque', 0.0)    # Nm at the WHEEL
        self.node.declare_parameter('dither_hz', 25.0)       # Hz — see dither()

        def p(name):
            return self.node.get_parameter(name).value

        gear = p('gear_ratio')
        self.invert = {'left': -1.0 if p('invert_left') else 1.0,
                       'right': -1.0 if p('invert_right') else 1.0}
        self.cmd_timeout = p('cmd_timeout')

        self.bus = can.interface.Bus(interface=p('can_interface'),
                                     channel=p('can_channel'),
                                     bitrate=p('bitrate'))
        self.clients = {
            'left':  ODriveClient(self.bus, node_id=p('left_node_id'),  gear_ratio=gear),
            'right': ODriveClient(self.bus, node_id=p('right_node_id'), gear_ratio=gear),
        }

        # latest feedback, written by the reader thread, read by the timer.
        # OUTPUT-shaft turns / turns-per-second.
        self.fb = {'left': [0.0, 0.0], 'right': [0.0, 0.0]}
        self._lock = threading.Lock()
        self.last_cmd_time = self.node.get_clock().now()

        self.joint_pub = self.node.create_publisher(JointState, 'joint_states', 10)
        self.cmd_sub = self.node.create_subscription(
            Float64MultiArray, 'wheel_effort_controller/commands',
            self.command_callback, 10)

        self.arm(p('vel_limit'), p('current_limit'))

        self.running = True
        self.reader = threading.Thread(target=self.can_reader, daemon=True)
        self.reader.start()
        self.timer = self.node.create_timer(1.0 / p('publish_rate'), self.update)

    # ── ODrive lifecycle ─────────────────────────────────────────────────────
    def arm(self, vel_limit, current_limit):
        for name, c in self.clients.items():
            c.set_limits(vel_limit, current_limit)
            c.set_controller_modes(ControlMode.TORQUE, InputMode.PASSTHROUGH)
            c.set_axis_state(AxisState.CLOSED_LOOP_CONTROL)
            self.node.get_logger().info(f'armed {name} (node {c.node_id}) in torque mode')

    def disarm(self):
        for c in self.clients.values():
            try:
                c.set_torque(0.0)
                c.set_axis_state(AxisState.IDLE)
            except Exception:
                pass

    # ── CAN receive. A reply is a SEPARATE event from the request, so this
    #    runs continuously and just stashes whatever turns up. ───────────────
    def can_reader(self):
        while self.running:
            try:
                msg = self.bus.recv(timeout=0.1)
            except Exception:
                break
            if msg is None:
                continue
            for name, c in self.clients.items():
                decoded = c.decode(msg)
                if decoded and decoded[0] == 'encoder':
                    pos, vel = decoded[1]            # OUTPUT-shaft turns, turns/s
                    with self._lock:
                        self.fb[name] = [pos, vel]
                    break

    # ── torque out ───────────────────────────────────────────────────────────
    def command_callback(self, msg):
        if len(msg.data) < 2:
            return
        self.last_cmd_time = self.node.get_clock().now()
        left_cmd, right_cmd = float(msg.data[0]), float(msg.data[1])

        # ── SAFETY CONTRACT: exactly [0.0, 0.0] means CUT TORQUE. ────────────
        # It passes through uncompensated — no feedforward, no dither, nothing.
        #
        # This is load-bearing, not defensive coding. balance_controller keeps
        # publishing at 100 Hz while DISABLED, and it publishes literal
        # [0.0, 0.0] on all three of its cut paths (disabled mode, cutoff_pitch
        # exceeded, and pre-settle). Without this branch, dither would energise
        # the motors on a robot the operator had deliberately disabled — the
        # exact thing CLAUDE.md forbids.
        #
        # Exact float equality is deliberate: those paths assign the literal
        # 0.0, whereas the live law (torque -/+ t_yaw) landing on exactly 0.0
        # on BOTH wheels in the same tick is not a thing that happens. If it
        # ever did, the cost is one tick without dither.
        if left_cmd == 0.0 and right_cmd == 0.0:
            for c in self.clients.values():
                c.set_torque(0.0)
            return

        def p(name):
            return self.node.get_parameter(name).value

        tau_c, v_eps = p('friction_ff'), p('friction_v_eps')
        dither_amp, dither_hz = p('dither_torque'), p('dither_hz')

        # Commands arrive at the ~100 Hz IMU rate, so anything approaching
        # 50 Hz samples the square wave at its own transition and degenerates
        # into a DC bias. Cheap to say out loud; expensive to diagnose on a
        # robot that has started drifting for no visible reason.
        if dither_amp > 0.0 and dither_hz > 30.0:
            self.node.get_logger().warn(
                f'dither_hz {dither_hz:.1f} is too close to half the ~100 Hz '
                'command rate; the square wave degenerates into a DC torque '
                'bias. Use 25 Hz or below.', throttle_duration_sec=10.0)

        d = dither(time.monotonic(), dither_amp, dither_hz)

        # Feedback is OUTPUT-shaft turns/s in the MOTOR's sign convention;
        # invert brings it into the robot frame, which is the frame the
        # commanded torque is already in. Compensate there, then apply invert
        # ONCE at the end — the same convention joint_states is published in.
        with self._lock:
            vel = {n: self.fb[n][1] for n in self.clients}

        for name, cmd in (('left', left_cmd), ('right', right_cmd)):
            v_robot = self.invert[name] * vel[name] * TURNS_TO_RAD
            torque = (cmd
                      + friction_feedforward(v_robot, tau_c, v_eps)
                      + DITHER_PHASE[name] * d)
            # ODriveClient.set_torque takes WHEEL Nm and divides by the gear
            # ratio internally — do NOT pre-divide here.
            self.clients[name].set_torque(self.invert[name] * torque)

    # ── timer: poll feedback, publish joint_states ──────────────────────────
    def update(self):
        for c in self.clients.values():
            c.request(CMD_GET_ENCODER_ESTIMATES)

        # SAFETY: stale commands -> coast (zero torque), not last-value-forever.
        # NOTE this is NOT balance_controller's watchdog, which keeps balancing
        # on stale cmd_vel. Here a stale command means the CONTROLLER itself has
        # stopped, so there is nothing left to balance with. Revisit before the
        # robot ever runs untethered.
        age = (self.node.get_clock().now() - self.last_cmd_time).nanoseconds * 1e-9
        if age > self.cmd_timeout:
            for c in self.clients.values():
                c.set_torque(0.0)

        with self._lock:
            left, right = list(self.fb['left']), list(self.fb['right'])

        msg = JointState()
        msg.header.stamp = self.node.get_clock().now().to_msg()
        msg.name = ['left_wheel_joint', 'right_wheel_joint']
        # ODrive reports TURNS; balance_controller expects RADIANS.
        msg.position = [self.invert['left'] * left[0] * TURNS_TO_RAD,
                        self.invert['right'] * right[0] * TURNS_TO_RAD]
        msg.velocity = [self.invert['left'] * left[1] * TURNS_TO_RAD,
                        self.invert['right'] * right[1] * TURNS_TO_RAD]
        self.joint_pub.publish(msg)

    def shutdown(self):
        self.running = False
        self.disarm()


def main(args=None):
    rclpy.init(args=args)
    bridge = ODriveBridge()
    try:
        rclpy.spin(bridge.node)
    except KeyboardInterrupt:
        pass
    finally:
        bridge.shutdown()
        bridge.bus.shutdown()
        rclpy.shutdown()
