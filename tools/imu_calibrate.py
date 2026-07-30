"""Compute mount_rpy for real.yaml, and verify the sign of pitch.

mount_rpy is the rotation taking SENSOR axes into ROBOT axes. imu_node applies
it as quat_mul(q_mount, q_sensor), so if the robot is held LEVEL and FORWARD
the corrected orientation should be identity — meaning q_mount = conj(q_sensor).
This reads the raw sensor, averages, and prints the value to paste.

A wrong mount_rpy fails exactly like a wrong invert_*: the balance loop gets a
mis-signed pitch and drives into the fall. Do this before torque is enabled.

    # 1. imu_node with NO correction applied yet
    ros2 run robot_base imu_node --ros-args -p driver:=uart

    # 2. hold the robot LEVEL and facing FORWARD, then:
    python3 tools/imu_calibrate.py

    # 3. paste mount_rpy into real.yaml, restart imu_node WITH it, then:
    python3 tools/imu_calibrate.py --verify
"""
import argparse
import math
import sys

import rclpy
from sensor_msgs.msg import Imu


def quat_to_rpy(w, x, y, z):
    """Inverse of imu_node.quat_from_rpy (ZYX / aerospace convention)."""
    roll = math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
    s = max(-1.0, min(1.0, 2 * (w * y - z * x)))
    pitch = math.asin(s)
    yaw = math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
    return roll, pitch, yaw


def collect(node, samples, timeout):
    got = []

    def cb(msg):
        got.append((msg.orientation.w, msg.orientation.x,
                    msg.orientation.y, msg.orientation.z))

    sub = node.create_subscription(Imu, '/imu', cb, 20)
    end = node.get_clock().now().nanoseconds + int(timeout * 1e9)
    while len(got) < samples and node.get_clock().now().nanoseconds < end:
        rclpy.spin_once(node, timeout_sec=0.1)
    node.destroy_subscription(sub)
    return got


def average(quats):
    """Componentwise mean + renormalise. Fine for the small spread of a held
    robot; it is not a proper quaternion mean and would be wrong over big
    angular ranges."""
    n = len(quats)
    acc = [sum(q[i] for q in quats) / n for i in range(4)]
    # keep the hemisphere consistent, or opposite-sign quats cancel
    if acc[0] < 0:
        acc = [-v for v in acc]
    mag = math.sqrt(sum(v * v for v in acc))
    return [v / mag for v in acc]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--samples', type=int, default=100)
    ap.add_argument('--timeout', type=float, default=10.0)
    ap.add_argument('--verify', action='store_true',
                    help='check an already-applied mount_rpy instead')
    a = ap.parse_args()

    rclpy.init()
    node = rclpy.create_node('imu_calibrate')
    print(f'listening on /imu for up to {a.timeout}s — hold the robot LEVEL '
          'and FORWARD, still...')
    quats = collect(node, a.samples, a.timeout)
    if not quats:
        print('\nNo messages on /imu. Is imu_node running? '
              '(ros2 run robot_base imu_node --ros-args -p driver:=uart)')
        rclpy.shutdown()
        return 1

    w, x, y, z = average(quats)
    roll, pitch, yaw = quat_to_rpy(w, x, y, z)
    spread = max(abs(q[i] - [w, x, y, z][i]) for q in quats for i in range(4))
    print(f'\n{len(quats)} samples, max deviation {spread:.4f} '
          f'({"steady" if spread < 0.02 else "MOVING — hold it stiller"})')
    print(f'measured  roll {math.degrees(roll):+7.2f}  '
          f'pitch {math.degrees(pitch):+7.2f}  yaw {math.degrees(yaw):+7.2f}  (deg)')

    if a.verify:
        off = max(abs(roll), abs(pitch))
        print(f'\nlevel error: {math.degrees(off):.2f} deg')
        if off < math.radians(2):
            print('PASS — mount_rpy is good.')
        else:
            print('FAIL — still tilted. Re-run without --verify and repaste.')
        print('\nNow check the SIGN, which is what actually kills the robot:\n'
              '  ros2 topic echo /imu --field orientation\n'
              'Tip the robot NOSE-DOWN (forward). Note which way pitch moves,\n'
              'and confirm balance_controller drives the wheels the same way\n'
              'the robot is falling. Get this backwards and it accelerates the fall.')
        rclpy.shutdown()
        return 0

    # q_mount = conj(q_sensor), so that quat_mul(q_mount, q_sensor) = identity
    mr, mp, my = quat_to_rpy(w, -x, -y, -z)
    print('\npaste into src/robot_bringup/config/real.yaml under imu_node:\n')
    print(f'    mount_rpy: [{mr:.6f}, {mp:.6f}, {my:.6f}]')
    print(f'    # = [{math.degrees(mr):+.1f}, {math.degrees(mp):+.1f}, '
          f'{math.degrees(my):+.1f}] deg, measured {__import__("time").strftime("%Y-%m-%d")}')
    print('\nThen restart imu_node with it and run --verify.')
    print('NOTE yaw: this zeroes heading to whatever direction you are pointing\n'
          'now. That is right if the board is rotated in-plane on the chassis,\n'
          'wrong if you just want roll/pitch — in that case set the third\n'
          'element to 0.0 and leave yaw free.')
    rclpy.shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
