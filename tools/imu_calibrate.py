"""Compute mount_rpy for real.yaml, and verify the sign of pitch.

mount_rpy is the FIXED physical rotation taking SENSOR axes into ROBOT axes.
imu_node applies it as quat_mul(q_sensor, quat_conj(q_mount)) — a BODY-side
correction. It must contain NO heading: a heading baked into the mount acts as
a body-frame yaw, and a body yaw MIXES roll into pitch, which no amount of
re-calibrating can undo.

USE --solve. It is the only mode that gets this right, because it builds the
mount from two physical directions rather than from one attitude reading:
gravity at level gives the robot's UP axis in sensor coords, and the rotation
axis between level and nose-down gives the robot's PITCH axis. Two axes fully
determine the frame, and neither depends on which way the robot is facing.

A wrong mount_rpy fails exactly like a wrong invert_*: the balance loop gets a
mis-signed pitch and drives into the fall. Do this before torque is enabled.

    # 1. imu_node with NO correction applied
    ros2 run robot_base imu_node --ros-args \
        --params-file src/robot_bringup/config/real.yaml \
        -p mount_rpy:="[0.0, 0.0, 0.0]"

    # 2. level pose, then nose-down pose:
    python3 tools/imu_calibrate.py --solve

    # 3. paste into real.yaml, restart imu_node WITHOUT the override, then:
    python3 tools/imu_calibrate.py --verify
    python3 tools/imu_calibrate.py --watch     # nose-down -> pitch POSITIVE
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


def align(quats, ref=None):
    """Put every sample in the same hemisphere as `ref` (default: the first).

    q and -q are the SAME rotation, and the sensor is free to report either.
    Near w = 0 -- which is exactly where a ~180 deg mount yaw puts you -- it
    flips between them sample to sample. Averaging without this cancels
    opposite-signed samples and produces a confident, wrong answer with a
    deviation of ~2.0 (the full component range). Fixing the sign of the SUM
    afterwards does not help: the damage is already done.
    """
    ref = quats[0] if ref is None else ref
    out = []
    for q in quats:
        dot = sum(a * b for a, b in zip(ref, q))
        out.append(tuple(-v for v in q) if dot < 0 else tuple(q))
    return out


def average(quats):
    """Componentwise mean + renormalise, after hemisphere alignment. Fine for
    the small spread of a held robot; not a proper quaternion mean and would
    be wrong over big angular ranges."""
    quats = align(quats)
    n = len(quats)
    acc = [sum(q[i] for q in quats) / n for i in range(4)]
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


def quat_rotate(q, v):
    w, x, y, z = q
    vx, vy, vz = v
    tx = 2 * (y * vz - z * vy)
    ty = 2 * (z * vx - x * vz)
    tz = 2 * (x * vy - y * vx)
    return (vx + w * tx + (y * tz - z * ty),
            vy + w * ty + (z * tx - x * tz),
            vz + w * tz + (x * ty - y * tx))


def _norm(v):
    m = math.sqrt(sum(c * c for c in v))
    return tuple(c / m for c in v)


def _cross(a, b):
    return (a[1] * b[2] - a[2] * b[1],
            a[2] * b[0] - a[0] * b[2],
            a[0] * b[1] - a[1] * b[0])


def _dot(a, b):
    return sum(x * y for x, y in zip(a, b))


def matrix_to_quat(m):
    """m is 3 rows. Shepperd's method — picks the numerically stable branch."""
    t = m[0][0] + m[1][1] + m[2][2]
    if t > 0:
        s = math.sqrt(t + 1.0) * 2
        return (0.25 * s, (m[2][1] - m[1][2]) / s,
                (m[0][2] - m[2][0]) / s, (m[1][0] - m[0][1]) / s)
    if m[0][0] > m[1][1] and m[0][0] > m[2][2]:
        s = math.sqrt(1.0 + m[0][0] - m[1][1] - m[2][2]) * 2
        return ((m[2][1] - m[1][2]) / s, 0.25 * s,
                (m[0][1] + m[1][0]) / s, (m[0][2] + m[2][0]) / s)
    if m[1][1] > m[2][2]:
        s = math.sqrt(1.0 + m[1][1] - m[0][0] - m[2][2]) * 2
        return ((m[0][2] - m[2][0]) / s, (m[0][1] + m[1][0]) / s,
                0.25 * s, (m[1][2] + m[2][1]) / s)
    s = math.sqrt(1.0 + m[2][2] - m[0][0] - m[1][1]) * 2
    return ((m[1][0] - m[0][1]) / s, (m[0][2] + m[2][0]) / s,
            (m[1][2] + m[2][1]) / s, 0.25 * s)


def solve_geometric(q_level, q_down):
    """Build the mount from two poses using only PHYSICAL directions.

    Contains no heading by construction, which is the whole point — a heading
    baked into the mount acts as a body-frame yaw, and a body yaw MIXES roll
    into pitch. That is unfixable by any amount of refining.

      * robot UP in sensor coords: rotate world +z back through the level
        attitude. Gravity gives this; it is heading-independent.
      * robot PITCH AXIS in sensor coords: the rotation from level to
        nose-down, expressed in the sensor frame, is a rotation about the
        robot's own +y. Its axis is therefore +y in sensor coords.

    Two axes fix the third, so the frame is fully determined. Returns
    (roll, pitch, yaw) for mount_rpy, plus the tilt angle actually used.
    """
    z_r = _norm(quat_rotate(quat_conj(q_level), (0.0, 0.0, 1.0)))

    d = quat_mul(quat_conj(q_level), q_down)      # level->down, in SENSOR frame
    if d[0] < 0:
        d = tuple(-v for v in d)                  # positive rotation angle
    angle = 2 * math.acos(max(-1.0, min(1.0, d[0])))
    s = math.sqrt(max(1e-12, 1 - d[0] * d[0]))
    y_r = _norm((d[1] / s, d[2] / s, d[3] / s))

    # The two measurements are NOT equally trustworthy, so orthogonalise the
    # weak one against the strong one. Gravity is a precise, repeatable
    # physical reference; a hand-held nose-down tilt always carries a few
    # degrees of out-of-plane wobble. Projecting UP onto the tilt plane (the
    # other way round) lets that wobble tip the whole frame and leaves a
    # standing roll error — measured at 6 deg on real hardware.
    #
    # Keeping z exact makes the level pose read exactly flat, and confines the
    # tilt sloppiness to the in-plane (heading) axis, which balance ignores.
    tilt_out_of_plane = math.degrees(math.asin(max(-1.0, min(1.0, _dot(y_r, z_r)))))
    y_r = _norm(tuple(a - _dot(y_r, z_r) * b for a, b in zip(y_r, z_r)))
    x_r = _norm(_cross(y_r, z_r))                 # x = y cross z (right-handed)

    # rows are the ROBOT basis vectors written in SENSOR coords == R_robot<-sensor
    q = matrix_to_quat([x_r, y_r, z_r])
    return quat_to_rpy(*q), angle, tilt_out_of_plane


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
    # HOW you tilt matters more than how far. The level->down rotation axis is
    # what fixes the in-plane alignment, and a hand tilt carries several degrees
    # of out-of-plane wobble: 6 deg of wobble on a 21 deg tilt throws the axis
    # 16 deg off, which cross-couples roll into pitch. Tipping the robot on its
    # WHEELS constrains the rotation to the real pitch axis mechanically, and
    # costs nothing.
    input('2/2  Now tip it NOSE-DOWN 20-30 deg — ROLL IT FORWARD ON ITS WHEELS\n'
          '     so the axle sets the axis, do not twist it by hand. Enter...')
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


def current_mount(node='/imu_node'):
    """Read the mount_rpy the RUNNING node is actually using.

    Deliberately asks the node, not the yaml — a CLI override beats the file
    and the two disagree often enough that reading the file proves nothing.
    """
    import re
    import subprocess
    try:
        out = subprocess.run(['ros2', 'param', 'get', node, 'mount_rpy'],
                             capture_output=True, text=True, timeout=15).stdout
    except Exception:
        return None
    nums = re.findall(r'-?\d+\.\d+(?:[eE][-+]?\d+)?', out)
    return [float(v) for v in nums[:3]] if len(nums) >= 3 else None


def refine(node, samples, timeout):
    """Fold the remaining error into an ALREADY-APPLIED mount_rpy.

    imu_node publishes q_corrected = q_sensor (x) conj(q_old). Calibrating
    against that measures only the residual q_res, so the corrected mount is
    q_new = q_res (x) q_old -- residual on the LEFT. You cannot add the angle
    triples; rotations do not compose that way, which is the mistake this
    exists to prevent.
    """
    old = current_mount()
    if old is None:
        print('Could not read mount_rpy from /imu_node — is it running?')
        return None
    print(f'current mount_rpy: [{old[0]:.6f}, {old[1]:.6f}, {old[2]:.6f}]')
    print(f'  = [{math.degrees(old[0]):+.2f}, {math.degrees(old[1]):+.2f}, '
          f'{math.degrees(old[2]):+.2f}] deg')
    print(f'\nleave the robot RESTING and STILL, measuring for {timeout}s...')
    quats = collect(node, samples, timeout)
    if not quats:
        print('No messages on /imu.')
        return None

    mean = average(quats)
    spread = max(abs(q[i] - mean[i]) for q in align(quats, mean) for i in range(4))
    r, p, _ = quat_to_rpy(*mean)
    print(f'{len(quats)} samples, max deviation {spread:.4f} '
          f'({"steady" if spread < 0.02 else "MOVING — hold it stiller"})')
    print(f'residual: roll {math.degrees(r):+.3f}  pitch {math.degrees(p):+.3f} deg')

    q_new = quat_mul(mean, quat_from_rpy(*old))
    nr, np_, ny = quat_to_rpy(*q_new)
    return (r, p), (nr, np_, ny), spread


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--samples', type=int, default=100)
    ap.add_argument('--timeout', type=float, default=10.0)
    ap.add_argument('--verify', action='store_true',
                    help='check an already-applied mount_rpy instead')
    ap.add_argument('--refine', action='store_true',
                    help='fold the remaining tilt into the mount_rpy the node '
                         'is ALREADY using. Run this with the correction '
                         'applied — no need to zero it first.')
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

    if a.refine:
        res = refine(node, a.samples, a.timeout)
        if res is None:
            rclpy.shutdown()
            return 1
        (rr, rp), (nr, np_, ny), spread = res
        if max(abs(rr), abs(rp)) < math.radians(0.3):
            print('\nResidual is already under 0.3 deg — nothing worth changing.')
        else:
            print('\npaste into real.yaml under imu_node:\n')
            print(f'    mount_rpy: [{nr:.6f}, {np_:.6f}, {ny:.6f}]')
            print(f'    # = [{math.degrees(nr):+.2f}, {math.degrees(np_):+.2f}, '
                  f'{math.degrees(ny):+.2f}] deg, refined')
            print('\nrestart imu_node, then --verify, then --watch to confirm '
                  'nose-down is still POSITIVE.')
        if spread >= 0.02:
            print('\nWARNING: samples were not steady. Rest the robot on a flat '
                  'surface rather than holding it, and re-run.')
        rclpy.shutdown()
        return 0

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
        q_level, q_down, _, _ = res
        (gr, gp, gy), tilt, wobble = solve_geometric(q_level, q_down)
        q_m = quat_from_rpy(gr, gp, gy)
        lr, lp, _ = quat_to_rpy(*quat_mul(q_level, quat_conj(q_m)))
        dr, dp, _ = quat_to_rpy(*quat_mul(q_down, quat_conj(q_m)))

        print(f'\ntilt between the two poses: {math.degrees(tilt):.1f} deg '
              f'(out-of-plane wobble {wobble:+.1f} deg, discarded)')
        if math.degrees(tilt) < 8:
            print('WARNING: that is a small tilt. The pitch axis estimate gets '
                  'noisy below ~10 deg — re-run with a bigger nose-down.')
        skew = math.degrees(math.atan2(abs(wobble), max(1.0, math.degrees(tilt))))
        if skew > 3.0:
            print(f'WARNING: the tilt axis is {skew:.1f} deg off the pitch axis. '
                  'Level will still be exact, but roll will cross-couple into '
                  f'pitch by ~{math.sin(math.radians(skew)) * 100:.0f}%, and the '
                  'balance loop reacts to pitch. Re-run tipping the robot on '
                  'its WHEELS rather than twisting it by hand.')
        if dp < 0:
            print('\n*** nose-down reads NEGATIVE pitch. You almost certainly '
                  'tilted the robot NOSE-UP. Re-run and tip the FRONT toward '
                  'the floor. ***')
        print('\ngeometric solve (exact, contains NO heading):')
        print(f'  level residual  roll {math.degrees(lr):+.3f}  pitch {math.degrees(lp):+.3f} deg')
        print(f'  nose-down reads pitch {math.degrees(dp):+.2f} deg '
              f'({"POSITIVE — correct" if dp > 0 else "NEGATIVE — WRONG"})')
        print('\npaste into real.yaml under imu_node:\n')
        print(f'    mount_rpy: [{gr:.6f}, {gp:.6f}, {gy:.6f}]')
        print(f'    # = [{math.degrees(gr):+.2f}, {math.degrees(gp):+.2f}, '
              f'{math.degrees(gy):+.2f}] deg, geometric 2-pose solve')
        print('\nThis is the FINAL value — do NOT run --refine on top of it.')
        print('Restart imu_node, then --verify and --watch.')
        rclpy.shutdown()
        return 0

    if a.verify:
        print(f'\nlisten on /imu for up to {a.timeout}s — robot RESTING and still...')
        quats = collect(node, a.samples, a.timeout)
        if not quats:
            print('No messages on /imu — is imu_node running?')
            rclpy.shutdown()
            return 1
        mean = average(quats)
        roll, pitch, yaw = quat_to_rpy(*mean)
        spread = max(abs(q[i] - mean[i]) for q in align(quats, mean) for i in range(4))
        print(f'{len(quats)} samples, max deviation {spread:.4f} '
              f'({"steady" if spread < 0.02 else "MOVING — hold it stiller"})')
        print(f'measured  roll {math.degrees(roll):+7.2f}  '
              f'pitch {math.degrees(pitch):+7.2f}  yaw {math.degrees(yaw):+7.2f}  (deg)')
        # yaw is heading, not a mount property — balance never reads it
        off = max(abs(roll), abs(pitch))
        print(f'\nlevel error (roll/pitch only): {math.degrees(off):.2f} deg')
        if off < math.radians(2):
            print('PASS — mount_rpy is good.')
        else:
            print('FAIL — still tilted. Re-run --solve, tipping on the WHEELS.')
        print('\nNow check the SIGN, which is what actually kills the robot:\n'
              '  python3 tools/imu_calibrate.py --watch\n'
              'Tip the robot NOSE-DOWN (forward). Pitch must go POSITIVE, and\n'
              'roll should stay put. Backwards here and the loop drives INTO\n'
              'the fall.')
        rclpy.shutdown()
        return 0

    # imu_node applies q_corrected = q_sensor (x) conj(q_mount), so making a
    # level robot read identity needs q_mount = q_sensor itself — NOT its
    # conjugate. (It was the conjugate while imu_node left-multiplied.)
    mr, mp, my = quat_to_rpy(w, x, y, z)
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
