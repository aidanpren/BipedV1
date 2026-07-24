"""No-hardware test for imu_node: does /imu carry a pitch balance_controller
can actually use, and does the mounting correction really take effect?

    python3 tools/test_imu_node.py
"""
import math
import os
import sys
import time

import rclpy
from sensor_msgs.msg import Imu

ROOT = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..')
sys.path.insert(0, os.path.join(ROOT, 'src', 'robot_base'))

from robot_base.imu_node import ImuNode          # noqa: E402

failures = []


def check(name, cond, detail=''):
    print(f"  [{'PASS' if cond else 'FAIL'}] {name}" + (f"  {detail}" if detail else ''))
    if not cond:
        failures.append(name)


def collect(mount_rpy, seconds=2.5):
    """Run imu_node with a given mount rotation; return the messages seen."""
    rclpy.init(args=['--ros-args',
                     '-p', 'driver:=fake',
                     '-p', 'publish_rate:=100.0',
                     '-p', f'mount_rpy:=[{mount_rpy[0]},{mount_rpy[1]},{mount_rpy[2]}]'])
    node = ImuNode()
    got = []
    node.node.create_subscription(Imu, 'imu', lambda m: got.append(m), 10)
    end = time.time() + seconds
    while time.time() < end:
        rclpy.spin_once(node.node, timeout_sec=0.005)
    rclpy.shutdown()
    return got


def pitch_of(msg):
    """The EXACT formula balance_controller uses."""
    q = msg.orientation
    sinp = max(-1.0, min(1.0, 2.0 * (q.w * q.y - q.z * q.x)))
    return math.asin(sinp)


def main():
    print('\n=== 1. identity mount: pitch round-trips through the quaternion ===')
    msgs = collect([0.0, 0.0, 0.0])
    check('messages published', len(msgs) > 100, f'{len(msgs)} msgs')
    check('frame_id set', msgs[-1].header.frame_id == 'imu_link',
          msgs[-1].header.frame_id)

    pitches = [pitch_of(m) for m in msgs]
    peak = max(abs(p) for p in pitches)
    # FakeDriver oscillates +/-0.15 rad; over 2.5 s at 0.5 Hz we see a full cycle
    check('pitch amplitude recovered (~0.15 rad)', abs(peak - 0.15) < 0.02,
          f'peak {peak:.4f} rad')
    check('pitch both signs seen (full cycle)',
          min(pitches) < -0.05 and max(pitches) > 0.05,
          f'{min(pitches):+.3f} .. {max(pitches):+.3f}')

    gy = [abs(m.angular_velocity.y) for m in msgs]
    check('pitch rate present on angular_velocity.y', max(gy) > 0.2,
          f'peak {max(gy):.3f} rad/s')

    print('\n=== 2. mount correction actually rotates the measurement ===')
    # Roll the sensor 90 deg: its pitch becomes ROLL in robot axes, so the
    # pitch balance_controller extracts must collapse to ~0.
    rolled = collect([math.pi / 2, 0.0, 0.0])
    rolled_peak = max(abs(pitch_of(m)) for m in rolled)
    check('90deg roll mount collapses pitch to ~0', rolled_peak < 0.02,
          f'peak {rolled_peak:.4f} rad (was {peak:.4f} at identity)')

    rolled_gz = max(abs(m.angular_velocity.z) for m in rolled)
    rolled_gy = max(abs(m.angular_velocity.y) for m in rolled)
    check('gyro rotated too: rate moved y -> z',
          rolled_gz > 0.2 and rolled_gy < 0.05,
          f'y peak {rolled_gy:.3f}, z peak {rolled_gz:.3f}')

    print('\n' + ('ALL CHECKS PASSED' if not failures
                  else f'{len(failures)} FAILED: {failures}'))
    return 1 if failures else 0


if __name__ == '__main__':
    sys.exit(main())
