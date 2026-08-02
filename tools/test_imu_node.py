"""No-hardware test for imu_node: does /imu carry a pitch balance_controller
can actually use, and does the mounting correction really take effect?

    python3 tools/test_imu_node.py

WHAT THIS TEST IS GUARDING (read before changing an assertion here).
imu_node corrects for how the board is bolted on with a RIGHT multiply by the
conjugate:  q_world<-robot = q_world<-sensor (x) conj(q_robot<-sensor).
Left-multiplying instead rotates in the WORLD frame. Both can be calibrated to
make a level robot read zero, so a left multiply SURVIVES bench calibration and
then gets the SIGN of pitch wrong as the robot tilts — which is the one thing
the correction exists to get right. That cost real hardware time on 2026-07-30.

Section 3 is the assertion that actually separates them, and it is worth more
than the axis-specific spot checks: for ANY mount, gravity and attitude must
tell the same story. FakeDriver reports accel = (-9.81 sin p, 0, 9.81 cos p)
alongside quat = Ry(p), so every published message must satisfy

    linear_acceleration.x  ==  -9.81 * sin(pitch_from_orientation)

That holds to machine epsilon for a right multiply at every mount tried, and is
violated by 1.5-19.6 m/s^2 by a left multiply at every non-identity mount. It
needs no phase alignment because both quantities come from the SAME message.
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

# The real robot's measured mount (real.yaml) — a board facing backwards.
REAL_MOUNT = [-0.051677, 0.016108, 3.107462]

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


def roll_of(msg):
    q = msg.orientation
    return math.atan2(2.0 * (q.w * q.x + q.y * q.z),
                      1.0 - 2.0 * (q.x * q.x + q.y * q.y))


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
    # YAW the sensor 90 deg: the board now faces left, so what the board calls
    # pitch is the robot's ROLL, and the pitch balance_controller extracts must
    # collapse to ~0.
    #
    # NOTE this used to test a 90 deg ROLL mount and assert the same collapse.
    # That is not what a roll mount does: it leaves pitch at 0.15 and puts
    # -90 deg on roll (verify with the ZYX decomposition — R = Ry(p)Rx(-90) is
    # literally yaw 0, pitch p, roll -90). The old assertion held only for the
    # left-multiply BUG, so it went red when the 2026-07-30 fix landed and sat
    # red because nothing offline was watching. Yaw is the axis that collapses
    # pitch, and it separates correct from buggy the same way.
    yawed = collect([0.0, 0.0, math.pi / 2])
    yaw_pitch_peak = max(abs(pitch_of(m)) for m in yawed)
    yaw_roll_peak = max(abs(roll_of(m)) for m in yawed)
    check('90deg yaw mount collapses pitch to ~0', yaw_pitch_peak < 0.02,
          f'peak {yaw_pitch_peak:.4f} rad (was {peak:.4f} at identity)')
    check('...and the tilt reappears on roll', abs(yaw_roll_peak - 0.15) < 0.02,
          f'roll peak {yaw_roll_peak:.4f} rad')

    yawed_gx = max(abs(m.angular_velocity.x) for m in yawed)
    yawed_gy = max(abs(m.angular_velocity.y) for m in yawed)
    check('gyro rotated too: rate moved y -> x',
          yawed_gx > 0.2 and yawed_gy < 0.05,
          f'y peak {yawed_gy:.3f}, x peak {yawed_gx:.3f}')

    print('\n=== 3. gravity and attitude agree (catches a left multiply) ===')
    for name, rpy in [('90deg roll', [math.pi / 2, 0.0, 0.0]),
                      ('90deg yaw', [0.0, 0.0, math.pi / 2]),
                      ('REAL robot mount', REAL_MOUNT)]:
        got = collect(rpy, seconds=1.2)
        worst = max(abs(m.linear_acceleration.x + 9.81 * math.sin(pitch_of(m)))
                    for m in got)
        check(f'{name}: accel.x == -9.81*sin(pitch)', worst < 1e-6,
              f'worst mismatch {worst:.2e} m/s2 over {len(got)} msgs')

    print('\n=== 4. the real mount FLIPS pitch sign (a backwards board) ===')
    # The whole point of the 2026-07-30 fix. The board faces backwards, so
    # nose-down for the robot is nose-up for the board and the sign MUST invert.
    # Checked against gravity rather than against a second run, because
    # FakeDriver free-runs and two runs are not phase aligned.
    real = collect(REAL_MOUNT, seconds=2.5)
    # accel.x < 0 means the robot is nose-up in FakeDriver's sensor convention;
    # pitch must carry the opposite sign to the raw sensor's at all times.
    agree = sum(1 for m in real
                if m.linear_acceleration.x * pitch_of(m) <= 1e-9)
    tilted = [m for m in real if abs(pitch_of(m)) > 0.05]
    check('pitch sign is consistent with measured gravity',
          agree == len(real), f'{agree}/{len(real)} messages')
    check('and the tilt is actually large enough to have a sign',
          len(tilted) > 50, f'{len(tilted)} msgs past 0.05 rad')

    print('\n' + ('ALL CHECKS PASSED' if not failures
                  else f'{len(failures)} FAILED: {failures}'))
    return 1 if failures else 0


if __name__ == '__main__':
    sys.exit(main())
