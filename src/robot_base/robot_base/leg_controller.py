"""Position control for the two LEG ODrives (the non-wheel motors).

Separate node from odrive_bridge on purpose: different control mode, different
failure behaviour, and SocketCAN happily gives both nodes their own socket on
the same interface.

    subscribes  leg_position_cmd   Float64  target leg position, OUTPUT turns
    publishes   leg_states         JointState (left_leg_joint, right_leg_joint)

SAFETY, because these carry the robot's weight:
  * commands are clamped to [pos_min, pos_max] — soft travel limits
  * the setpoint is RAMPED here at max_speed rather than stepped, so a loaded
    leg is never slammed. This does not depend on the ODrive's input_mode
    config, which we cannot verify from here.
  * torque cut == COLLAPSE for a leg, unlike a wheel where it just coasts.
    Read the note on shutdown() before wiring this into DISABLED.

Dry-run with no hardware (note --load-torque: that is the robot's weight):
    python3 tools/fake_odrive.py --channel vcan0 --node-id 3 --load-torque 0.15 &
    python3 tools/fake_odrive.py --channel vcan0 --node-id 4 --load-torque 0.15 &
    ros2 run robot_base leg_controller --ros-args -p can_channel:=vcan0
"""
import threading

import can
import rclpy
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64

from robot_base.odrive_can import (
    CMD_GET_ENCODER_ESTIMATES, AxisState, ControlMode, InputMode, ODriveClient,
)


class LegController:
    def __init__(self):
        self.node = rclpy.create_node('leg_controller')

        self.node.declare_parameter('can_interface', 'socketcan')
        self.node.declare_parameter('can_channel', 'vcan0')
        self.node.declare_parameter('bitrate', 500000)
        self.node.declare_parameter('left_node_id', 3)
        self.node.declare_parameter('right_node_id', 4)
        self.node.declare_parameter('gear_ratio', 8.0)
        self.node.declare_parameter('invert_left', False)
        self.node.declare_parameter('invert_right', True)
        self.node.declare_parameter('current_limit', 12.0)
        self.node.declare_parameter('vel_limit', 5.0)        # output turns/s
        self.node.declare_parameter('publish_rate', 50.0)
        # soft travel limits, OUTPUT turns. MEASURE THESE ON THE REAL LINKAGE.
        self.node.declare_parameter('pos_min', -0.25)
        self.node.declare_parameter('pos_max', 0.25)
        self.node.declare_parameter('home_position', 0.0)
        self.node.declare_parameter('max_speed', 0.15)        # output turns/s
        # 1 PASSTHROUGH, 3 POS_FILTER, 5 TRAP_TRAJ. 3/5 need extra config in
        # odrivetool; the node-side ramp below protects us either way.
        self.node.declare_parameter('input_mode', InputMode.POS_FILTER)
        self.node.declare_parameter('idle_on_shutdown', True)

        def p(name):
            return self.node.get_parameter(name).value

        self.pos_min, self.pos_max = p('pos_min'), p('pos_max')
        self.max_speed = p('max_speed')
        self.idle_on_shutdown = p('idle_on_shutdown')
        self.invert = {'left': -1.0 if p('invert_left') else 1.0,
                       'right': -1.0 if p('invert_right') else 1.0}

        self.bus = can.interface.Bus(interface=p('can_interface'),
                                     channel=p('can_channel'),
                                     bitrate=p('bitrate'))
        gear = p('gear_ratio')
        self.clients = {
            'left':  ODriveClient(self.bus, node_id=p('left_node_id'),  gear_ratio=gear),
            'right': ODriveClient(self.bus, node_id=p('right_node_id'), gear_ratio=gear),
        }

        self.fb = {'left': [0.0, 0.0], 'right': [0.0, 0.0]}   # output turns, turns/s
        self._lock = threading.Lock()

        home = self.clamp(p('home_position'))
        self.target = home        # where we've been told to go
        self.setpoint = home      # where we're actually commanding, ramped

        self.state_pub = self.node.create_publisher(JointState, 'leg_states', 10)
        self.cmd_sub = self.node.create_subscription(
            Float64, 'leg_position_cmd', self.command_callback, 10)

        self.arm(p('vel_limit'), p('current_limit'), p('input_mode'))

        self.running = True
        threading.Thread(target=self.can_reader, daemon=True).start()
        self.rate = p('publish_rate')
        self.timer = self.node.create_timer(1.0 / self.rate, self.update)

    def clamp(self, v):
        return max(self.pos_min, min(self.pos_max, float(v)))

    # ── lifecycle ────────────────────────────────────────────────────────────
    def arm(self, vel_limit, current_limit, input_mode):
        for name, c in self.clients.items():
            c.set_limits(vel_limit, current_limit)
            c.set_controller_modes(ControlMode.POSITION, input_mode)
            c.set_axis_state(AxisState.CLOSED_LOOP_CONTROL)
            self.node.get_logger().info(
                f'armed {name} leg (node {c.node_id}) in POSITION mode')

    def shutdown(self):
        self.running = False
        if self.idle_on_shutdown:
            # WARNING: for a LEG, IDLE means the joint goes limp and the robot
            # sits down. Safe on a stand; think hard before doing this with the
            # robot's weight on the legs.
            self.node.get_logger().warn('idling leg motors — legs will go limp')
            for c in self.clients.values():
                try:
                    c.set_axis_state(AxisState.IDLE)
                except Exception:
                    pass

    # ── CAN ──────────────────────────────────────────────────────────────────
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
                    with self._lock:
                        self.fb[name] = list(decoded[1])
                    break

    def command_callback(self, msg):
        requested = float(msg.data)
        self.target = self.clamp(requested)
        if abs(self.target - requested) > 1e-9:
            self.node.get_logger().warn(
                f'leg command {requested:.3f} clamped to {self.target:.3f} '
                f'(limits {self.pos_min}..{self.pos_max})')

    # ── control ──────────────────────────────────────────────────────────────
    def update(self):
        # ramp the setpoint instead of stepping it: a loaded leg given a step
        # sees a huge instantaneous position error and slams.
        step = self.max_speed / self.rate
        delta = self.target - self.setpoint
        if abs(delta) <= step:
            self.setpoint = self.target
        else:
            self.setpoint += step * (1.0 if delta > 0 else -1.0)

        for name, c in self.clients.items():
            c.set_input_pos(self.invert[name] * self.setpoint)
            c.request(CMD_GET_ENCODER_ESTIMATES)

        with self._lock:
            left, right = list(self.fb['left']), list(self.fb['right'])

        msg = JointState()
        msg.header.stamp = self.node.get_clock().now().to_msg()
        msg.name = ['left_leg_joint', 'right_leg_joint']
        msg.position = [self.invert['left'] * left[0],
                        self.invert['right'] * right[0]]
        msg.velocity = [self.invert['left'] * left[1],
                        self.invert['right'] * right[1]]
        self.state_pub.publish(msg)

    @property
    def sag(self):
        """Commanded minus actual — how far the load is pulling the leg down."""
        with self._lock:
            return (self.setpoint - self.invert['left'] * self.fb['left'][0],
                    self.setpoint - self.invert['right'] * self.fb['right'][0])


def main(args=None):
    rclpy.init(args=args)
    ctrl = LegController()
    try:
        rclpy.spin(ctrl.node)
    except KeyboardInterrupt:
        pass
    finally:
        ctrl.shutdown()
        ctrl.bus.shutdown()
        rclpy.shutdown()
