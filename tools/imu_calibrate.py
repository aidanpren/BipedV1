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


def quat_mul(a, b):
    aw, ax, ay, az = a
    bw, bx, by, bz = b
    return (aw * bw - ax * bx - ay * by - az * bz,
            aw * bx + ax * bw + ay * bz - az * by,
            aw * by - ax * bz + ay * bw + az * bx,
            aw * bz + ax * by - ay * bx + az * bw)


def quat_from_rpy(roll, pitch, yaw):
    cr, sr = math.cos(roll / 2), math.sin(roll / 2)
    cp, sp = math.cos(pitch / 2), math.sin(pitch / 2)
    cy, sy = math.cos(yaw / 2), math.sin(yaw / 2)
    return (cr * cp * cy + sr * sp * sy, sr * cp * cy - cr * sp * sy,
            cr * sp * cy + sr * cp * sy, cr * cp * sy - sr * sp * cy)


def solve(node, samples, timeout):
    """Find the mount rotation from TWO poses: level, then nose-down.

    One pose is not enough. Level tells you which rotations put the robot flat,
    but a whole family of those remain — including ones that read pitch
    BACKWARDS. Only a second, deliberately pitched pose picks the sign, and the
    sign is the thing that decides whether the balance loop catches a fall or
    drives into it.

    Roll/pitch are invariant to the sensor's arbitrary world heading (a
    left-multiplied Rz only shifts the yaw term of a ZYX decomposition), so we
    score on roll/pitch alone and leave heading free.
    """
    input('\n1/2  Hold the robot LEVEL and FORWARD, still. Press Enter...')
    level = collect(node, samples, timeout)
    if not level:
        return None
    input('2/2  Now tilt it NOSE-DOWN maybe 20-30 deg, hold still. Press Enter...')
    down = collect(node, samples, timeout)
    if not down:
        return None

    q_level, q_down = average(level), average(down)
    # how far it actually got tilted, in the raw sensor frame — used to sanity
    # check that the two poses really are different
    print(f'\ncaptured {len(level)} level + {len(down)} nose-down samples')

    best = []
    step = math.pi / 2
    seen = set()
    for i in range(4):
        for j in range(4):
            for k in range(4):
                rpy = (i * step, j * step, k * step)
                q_m = quat_from_rpy(*rpy)
                key = tuple(round(v, 6) for v in
                            (q_m if q_m[0] >= 0 else tuple(-v for v in q_m)))
                if key in seen:
                    continue
                seen.add(key)
                cq = quat_conj(q_m)
                lr, lp, _ = quat_to_rpy(*quat_mul(q_level, cq))
                _, dp, _ = quat_to_rpy(*quat_mul(q_down, cq))
                level_err = max(abs(lr), abs(lp))
                best.append((level_err, dp, rpy))

    # keep candidates that sit level AND read nose-down as POSITIVE pitch,
    # which is what balance_controller's positive k3 requires
    ok = [c for c in best if c[0] < math.radians(8) and c[1] > math.radians(5)]
    ok.sort(key=lambda c: c[0])
    return q_level, q_down, ok, sorted(best, key=lambda c: c[0])[:4]


def quat_conj(q):
    return (q[0], -q[1], -q[2], -q[3])


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--samples', type=int, default=100)
    ap.add_argument('--timeout', type=float, default=10.0)
    ap.add_argument('--verify', action='store_true',
                    help='check an already-applied mount_rpy instead')
    ap.add_argument('--watch', action='store_true',
                    help='live roll/pitch/yaw in DEGREES. Use this for the sign '
                         'check -- the raw quaternion y component is not pitch.')
    ap.add_argument('--solve', action='store_true',
                    help='two-pose solve: finds the mount that also gets the '
                         'PITCH SIGN right. Use this when level-only '
                         'calibration leaves nose-down reading negative.')
    a = ap.parse_args()

    rclpy.init()
    node = rclpy.create_node('imu_calibrate')

    if a.watch:
        print('live attitude — tilt NOSE-DOWN, pitch must go POSITIVE. Ctrl-C to stop.\n')
        last = [None]

        def cb(msg):
            o = msg.orientation
            r, p, y = quat_to_rpy(o.w, o.x, o.y, o.z)
            arrow = ''
            if last[0] is not None:
                d = p - last[0]
                arrow = ' pitch RISING' if d > 0.002 else (' pitch FALLING' if d < -0.002 else '')
            last[0] = p
            print(f'\rroll {math.degrees(r):+7.2f}  pitch {math.degrees(p):+7.2f}  '
                  f'yaw {math.degrees(y):+7.2f}{arrow:<15}', end='', flush=True)

        node.create_subscription(Imu, '/imu', cb, 20)
        try:
            rclpy.spin(node)
        except (KeyboardInterrupt, Exception):
            # SIGTERM tears the context down underneath spin(); that is a normal
            # way for this to end, not an error worth a traceback
            print()
        try:
            rclpy.shutdown()
        except Exception:
            pass
        return 0

    if a.solve:
        print('Run imu_node with mount_rpy [0.0, 0.0, 0.0] for this.')
        res = solve(node, a.samples, a.timeout)
        if res is None:
            print('\nNo messages on /imu — is imu_node running?')
            rclpy.shutdown()
            return 1
        _, _, ok, closest = res
        if ok:
            err, dp, rpy = ok[0]
            print(f'\n{len(ok)} candidate(s) satisfy level AND nose-down-positive.'
                  f'\nBest: level error {math.degrees(err):.2f} deg, '
                  f'nose-down pitch {math.degrees(dp):+.1f} deg\n')
            print('paste into real.yaml under imu_node:\n')
            print(f'    mount_rpy: [{rpy[0]:.6f}, {rpy[1]:.6f}, {rpy[2]:.6f}]')
            print(f'    # = [{math.degrees(rpy[0]):.0f}, {math.degrees(rpy[1]):.0f}, '
                  f'{math.degrees(rpy[2]):.0f}] deg, solved 2-pose')
            if len(ok) > 1:
                print('\nothers that also fit (differ only in heading):')
                for e, d, r in ok[1:4]:
                    print(f'    [{r[0]:.4f}, {r[1]:.4f}, {r[2]:.4f}]  '
                          f'level {math.degrees(e):.1f} deg')
        else:
            print('\nNO candidate satisfies both. Closest by level error:')
            for e, d, r in closest:
                print(f'    [{math.degrees(r[0]):4.0f},{math.degrees(r[1]):4.0f},'
                      f'{math.degrees(r[2]):4.0f}] deg  level {math.degrees(e):5.1f}  '
                      f'nose-down pitch {math.degrees(d):+6.1f}')
            print('\nIf every nose-down pitch is NEGATIVE, you tilted the wrong\n'
                  'way — nose-down means the FRONT goes toward the floor.\n'
                  'If level errors are all large, the board is not mounted at a\n'
                  '90-degree multiple and needs the one-pose calibration instead.')
        rclpy.shutdown()
        return 0
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
