"""No-hardware, no-sudo integration test: odrive_bridge <-> two fake ODrives.

Runs everything in ONE process over python-can's in-process 'virtual' bus, so
it needs neither vcan (sudo) nor a CANable. Checks the conversions that are
easy to get wrong.

    python3 tools/test_bridge_vs_fake.py
"""
import math
import os
import sys
import threading
import time

import can
import rclpy
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray

ROOT = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..')
sys.path.insert(0, os.path.join(ROOT, 'src', 'robot_base'))
sys.path.insert(0, os.path.join(ROOT, 'tools'))

from fake_odrive import FakeODrive                      # noqa: E402
from robot_base.odrive_bridge import ODriveBridge       # noqa: E402

CHANNEL = 'testbus'
GEAR = 8.0
failures = []


def check(name, cond, detail=''):
    print(f"  [{'PASS' if cond else 'FAIL'}] {name}" + (f"  {detail}" if detail else ''))
    if not cond:
        failures.append(name)


def main():
    rclpy.init(args=['--ros-args',
                     '-p', 'can_interface:=virtual',
                     '-p', f'can_channel:={CHANNEL}',
                     '-p', 'publish_rate:=50.0'])

    fakes = {}
    for name, nid in (('left', 1), ('right', 2)):
        bus = can.interface.Bus(interface='virtual', channel=CHANNEL)
        drv = FakeODrive(bus, node_id=nid)
        threading.Thread(target=drv.run, daemon=True).start()
        fakes[name] = drv

    bridge = ODriveBridge()
    received = []
    bridge.node.create_subscription(JointState, 'joint_states',
                                    lambda m: received.append(m), 10)
    cmd_pub = bridge.node.create_publisher(
        Float64MultiArray, 'wheel_effort_controller/commands', 10)

    print('\n=== 1. arming ===')
    for _ in range(40):
        rclpy.spin_once(bridge.node, timeout_sec=0.01)
    check('both ODrives armed into CLOSED_LOOP torque mode',
          all(f.axis_state == 8 and f.control_mode == 1 for f in fakes.values()),
          f"left={fakes['left'].axis_state}/{fakes['left'].control_mode} "
          f"right={fakes['right'].axis_state}/{fakes['right'].control_mode}")

    print('\n=== 2. torque command conversion (the divide-by-gear trap) ===')
    WHEEL_TORQUE = 0.8
    deadline = time.time() + 1.5
    while time.time() < deadline:
        cmd_pub.publish(Float64MultiArray(data=[WHEEL_TORQUE, WHEEL_TORQUE]))
        rclpy.spin_once(bridge.node, timeout_sec=0.01)

    expect = WHEEL_TORQUE / GEAR
    check('left motor torque = wheel torque / 8',
          abs(fakes['left'].torque_setpoint - expect) < 1e-6,
          f"got {fakes['left'].torque_setpoint:.5f}, expected {expect:.5f}")
    check('right motor torque inverted (mirrored motor)',
          abs(fakes['right'].torque_setpoint + expect) < 1e-6,
          f"got {fakes['right'].torque_setpoint:.5f}, expected {-expect:.5f}")

    print('\n=== 3. joint_states published ===')
    check('joint_states arriving', len(received) > 10, f'{len(received)} msgs')
    if received:
        check('joint names match balance_controller',
              received[-1].name == ['left_wheel_joint', 'right_wheel_joint'],
              str(received[-1].name))
        check('velocity array populated', len(received[-1].velocity) == 2)

    print('\n=== 4. motor actually spun, units in RADIANS ===')
    last = received[-1]
    # fake stores MOTOR turns; bridge should report OUTPUT radians
    expect_rad = fakes['left'].pos / GEAR * 2 * math.pi
    check('left wheel is turning', abs(last.velocity[0]) > 0.05,
          f'{last.velocity[0]:+.3f} rad/s')
    check('position converted motor-turns -> output radians',
          abs(last.position[0] - expect_rad) < 0.5,
          f'got {last.position[0]:+.3f} rad, fake at {expect_rad:+.3f} rad')
    # The mirrored-motor inversion is applied TWICE — once when sending torque,
    # once when reading feedback back — so it cancels in joint space. That is
    # the point: the physical motors turn opposite ways, but both wheels must
    # report the SAME sign for "forward", because balance_controller averages
    # them (v = r*(vel_l + vel_r)/2). Opposite signs there would cancel to zero.
    check('motors turn opposite ways in MOTOR frame',
          fakes['left'].vel * fakes['right'].vel < 0,
          f"{fakes['left'].vel:+.3f} vs {fakes['right'].vel:+.3f} motor turns/s")
    check('both wheels report SAME sign in JOINT frame',
          last.velocity[0] * last.velocity[1] > 0,
          f'{last.velocity[0]:+.3f} vs {last.velocity[1]:+.3f} rad/s')

    print('\n=== 5. stale-command watchdog coasts to zero torque ===')
    deadline = time.time() + 1.2          # stop publishing commands
    while time.time() < deadline:
        rclpy.spin_once(bridge.node, timeout_sec=0.01)
    check('torque zeroed after cmd_timeout',
          abs(fakes['left'].torque_setpoint) < 1e-9,
          f"{fakes['left'].torque_setpoint:.6f}")

    bridge.shutdown()
    rclpy.shutdown()

    print('\n' + ('ALL CHECKS PASSED' if not failures
                  else f'{len(failures)} FAILED: {failures}'))
    return 1 if failures else 0


if __name__ == '__main__':
    sys.exit(main())
