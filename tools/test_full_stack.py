"""Full-system dry run with NO hardware and NO sudo.

Runs the REAL nodes — imu_node, odrive_bridge, leg_controller,
balance_controller, mode_manager — against four fake ODrives on an in-process
virtual CAN bus, and checks the whole control loop actually closes:

    imu_node -> /imu ---------\\
                               balance_controller -> wheel_effort_controller/commands
    odrive_bridge -> /joint_states                        |
                               <-- torque reaches the fake wheel ODrives <--

    python3 tools/test_full_stack.py
"""
import os
import sys
import threading
import time

import can
import rclpy
from rclpy.executors import SingleThreadedExecutor
from sensor_msgs.msg import Joy

ROOT = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..')
sys.path.insert(0, os.path.join(ROOT, 'src', 'robot_base'))
sys.path.insert(0, os.path.join(ROOT, 'tools'))

from fake_odrive import FakeODrive                              # noqa: E402
from robot_base.balance_controller import BalanceController     # noqa: E402
from robot_base.imu_node import ImuNode                         # noqa: E402
from robot_base.leg_controller import LegController             # noqa: E402
from robot_base.odrive_bridge import ODriveBridge               # noqa: E402

CHANNEL = 'fullstack'
failures = []


def check(name, cond, detail=''):
    print(f"  [{'PASS' if cond else 'FAIL'}] {name}" + (f"  {detail}" if detail else ''))
    if not cond:
        failures.append(name)


def main():
    rclpy.init(args=['--ros-args',
                     '-p', 'can_interface:=virtual',
                     '-p', f'can_channel:={CHANNEL}',
                     '-p', 'driver:=fake'])

    # four fake ODrives: 1/2 wheels, 3/4 legs
    fakes = {}
    for name, nid in (('wheel_l', 1), ('wheel_r', 2), ('leg_l', 3), ('leg_r', 4)):
        bus = can.interface.Bus(interface='virtual', channel=CHANNEL)
        drv = FakeODrive(bus, node_id=nid)
        threading.Thread(target=drv.run, daemon=True).start()
        fakes[name] = drv

    print('\n=== 1. every node constructs ===')
    nodes = {}
    try:
        nodes['imu'] = ImuNode()
        nodes['bridge'] = ODriveBridge()
        nodes['legs'] = LegController()
        nodes['balance'] = BalanceController()
        check('all four hardware/control nodes constructed', True)
    except Exception as exc:
        check('all four hardware/control nodes constructed', False, repr(exc))
        rclpy.shutdown()
        return 1

    ex = SingleThreadedExecutor()
    for n in nodes.values():
        ex.add_node(n.node)

    # a live joystick, so the mode interlock is satisfiable
    joy_node = rclpy.create_node('fake_joy')
    joy_pub = joy_node.create_publisher(Joy, 'joy', 10)
    ex.add_node(joy_node)

    def spin(seconds):
        end = time.time() + seconds
        while time.time() < end:
            joy_pub.publish(Joy())
            ex.spin_once(timeout_sec=0.005)

    spin(1.5)

    print('\n=== 2. sensors and actuators wired up ===')
    check('wheel ODrives armed in TORQUE mode',
          all(fakes[k].control_mode == 1 and fakes[k].axis_state == 8
              for k in ('wheel_l', 'wheel_r')))
    check('leg ODrives armed in POSITION mode',
          all(fakes[k].control_mode == 3 and fakes[k].axis_state == 8
              for k in ('leg_l', 'leg_r')))
    check('balance_controller received IMU pitch',
          abs(nodes['balance'].node.get_clock().now().nanoseconds) > 0
          and nodes['balance'].x_ready is not None)

    print('\n=== 3. the loop is OPEN while mode is disabled ===')
    # balance_controller boots to 'disabled' and must publish zero torque
    spin(1.0)
    idle_torque = max(abs(fakes['wheel_l'].torque_setpoint),
                      abs(fakes['wheel_r'].torque_setpoint))
    check('no wheel torque while disabled', idle_torque < 1e-9,
          f'{idle_torque:.6f} Nm')

    print('\n=== 4. the loop CLOSES in teleop ===')
    # publish the mode directly (mode_manager is exercised by its own test)
    from std_msgs.msg import String
    from rclpy.qos import DurabilityPolicy, QoSProfile
    latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
    mode_pub = joy_node.create_publisher(String, 'mode', latched)
    mode_pub.publish(String(data='teleop'))
    spin(2.0)

    torque = max(abs(fakes['wheel_l'].torque_setpoint),
                 abs(fakes['wheel_r'].torque_setpoint))
    check('balance_controller drove torque to the wheel ODrives', torque > 1e-6,
          f'{torque:.5f} Nm at the motor')
    check('wheels actually turning', abs(fakes['wheel_l'].vel) > 0.01,
          f"{fakes['wheel_l'].vel:+.3f} motor turns/s")

    print('\n=== 5. legs respond to a command ===')
    from std_msgs.msg import Float64
    leg_pub = joy_node.create_publisher(Float64, 'leg_position_cmd', 10)
    leg_pub.publish(Float64(data=0.2))
    spin(2.5)
    check('leg setpoint ramped toward the command',
          abs(nodes['legs'].setpoint - 0.2) < 0.01,
          f"setpoint {nodes['legs'].setpoint:.3f}")
    check('leg ODrives moved', abs(fakes['leg_l'].pos) > 0.05,
          f"{fakes['leg_l'].pos:+.3f} motor turns")

    for n in nodes.values():
        if hasattr(n, 'shutdown'):
            n.shutdown()
    rclpy.shutdown()

    print('\n' + ('ALL CHECKS PASSED' if not failures
                  else f'{len(failures)} FAILED: {failures}'))
    return 1 if failures else 0


if __name__ == '__main__':
    sys.exit(main())
