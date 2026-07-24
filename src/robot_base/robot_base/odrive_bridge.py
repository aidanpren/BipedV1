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

import can
import rclpy
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray

from robot_base.odrive_can import (
    CMD_GET_ENCODER_ESTIMATES, AxisState, ControlMode, InputMode, ODriveClient,
)

TURNS_TO_RAD = 2 * math.pi


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
        for name, torque in zip(('left', 'right'), msg.data[:2]):
            # ODriveClient.set_torque takes WHEEL Nm and divides by the gear
            # ratio internally — do NOT pre-divide here.
            self.clients[name].set_torque(self.invert[name] * float(torque))

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
