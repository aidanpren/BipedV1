"""No-hardware test for balance_controller's YAW path (added 2026-08-02).

    python3 tools/test_yaw_shaping.py

Guards two defects that were live on the robot until 2026-08-02:

  1. yaw_ref reached the control law as a raw 20 Hz staircase while linear.x
     got two stages of shaping, so a stick flick delivered
     k_yaw * scale_angular.yaw = 4.0 * 0.75 = 3.0 Nm of DIFFERENTIAL torque in
     a single tick, out of a max_torque of 8.0 shared with balancing.

  2. left and right were clamped INDEPENDENTLY. At torque 7.0 with t_yaw 3.0
     that gives left 4.0 and right 10.0 -> 8.0, so the common mode holding the
     robot up silently drops from 7.0 to 6.0. Asymmetric saturation paid for
     the turn out of the BALANCE budget, invisibly.

The controller is driven directly (no executor) so the test is deterministic,
with a fake clock so dt is exact rather than whatever the host scheduler gave
us. Everything asserted here is dt-independent.
"""
import math
import os
import sys

import rclpy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu, JointState

ROOT = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..')
sys.path.insert(0, os.path.join(ROOT, 'src', 'robot_base'))

from robot_base.balance_controller import BalanceController      # noqa: E402

DT = 0.01           # 100 Hz, the real IMU rate
K_YAW = 4.0
MAX_TORQUE = 8.0
YAW_ACCEL = 2.0
STICK = 0.75        # scale_angular.yaw at full deflection

failures = []


def check(name, cond, detail=''):
    print(f"  [{'PASS' if cond else 'FAIL'}] {name}" + (f"  {detail}" if detail else ''))
    if not cond:
        failures.append(name)


class Rig:
    """Drives the real BalanceController on a fake clock."""

    def __init__(self, **params):
        args = ['--ros-args']
        base = {'k_yaw': K_YAW, 'max_torque': MAX_TORQUE,
                'yaw_accel_limit': YAW_ACCEL, 'pitch_trim': 0.0,
                'accel_to_lean': 0.0}
        base.update(params)
        for k, v in base.items():
            args += ['-p', f'{k}:={v}']
        rclpy.init(args=args)
        self.ctrl = BalanceController()
        self.ctrl.mode = 'teleop'
        self.out = []
        self.ctrl.publisher.publish = lambda m: self.out.append(tuple(m.data))

        # Fake clock: every call advances by exactly DT, so `age`, the ramp
        # step and the settle timer are all exact. Real sleeps would make the
        # measured slew depend on host jitter.
        self.t = 0.0

        class FakeNow:
            def __init__(self, ns):
                self.nanoseconds = ns

            def __sub__(self, other):
                return FakeNow(self.nanoseconds - other.nanoseconds)

        class FakeClock:
            def __init__(self, rig):
                self.rig = rig

            def now(self):
                return FakeNow(int(self.rig.t * 1e9))

        self.ctrl.node.get_clock = lambda: FakeClock(self)
        self.ctrl.last_cmd_time = FakeNow(0)

    def wheels(self, x=0.0, v=0.0):
        js = JointState()
        js.name = ['left_wheel_joint', 'right_wheel_joint']
        r = self.ctrl.node.get_parameter('wheel_radius').value
        js.position = [x / r, x / r]
        js.velocity = [v / r, v / r]
        self.ctrl.joint_state_callback(js)

    def cmd(self, vx=0.0, wz=0.0):
        t = Twist()
        t.linear.x, t.angular.z = vx, wz
        self.ctrl.cmd_vel_callback(t)

    def tick(self, pitch=0.0, pitch_rate=0.0, wz=0.0):
        self.t += DT
        m = Imu()
        half = pitch / 2.0
        m.orientation.w, m.orientation.y = math.cos(half), math.sin(half)
        m.angular_velocity.y = pitch_rate
        m.angular_velocity.z = wz
        self.ctrl.imu_callback(m)
        return self.out[-1]

    def drive(self, n, wz=0.0, **kw):
        """Tick n times while HOLDING the stick.

        cmd_vel must be republished every tick or the 0.5 s watchdog fires and
        the test silently measures the stale-command ramp instead of the thing
        it meant to measure. That is not hypothetical — it is what the first
        version of this file did, and check 5 caught it.
        """
        return [self.cmd(wz=wz) or self.tick(**kw) for _ in range(n)]

    def settle(self, n=150):
        """Run past the 1 s post-recovery settle so the outer loop engages."""
        self.wheels()
        self.drive(n)
        self.out.clear()

    def close(self):
        rclpy.shutdown()


def t_yaw_of(cmd):
    """Differential half-difference: t_yaw = (right - left) / 2."""
    return (cmd[1] - cmd[0]) / 2.0


def balance_of(cmd):
    """Common mode: the torque actually holding the robot up."""
    return (cmd[1] + cmd[0]) / 2.0


def main():
    print('\n=== 1. a full-stick yaw flick is a ramp, not a step ===')
    rig = Rig()
    rig.settle()
    # Seed with the value BEFORE the flick. Without it the step lands between
    # the establishing phase and the first captured sample, every tick-to-tick
    # delta reads 0.0000 Nm, and the slew checks pass against the very code
    # they exist to reject. Verified by re-running this file against the old
    # controller: without the seed it reports "peak 0.0000 Nm/tick".
    seq = [t_yaw_of(rig.drive(1)[-1])]
    seq += [t_yaw_of(c) for c in rig.drive(80, wz=STICK)]
    rig.close()

    first = abs(seq[1])                # seq[0] is the pre-flick seed
    slew = max(abs(b - a) for a, b in zip(seq, seq[1:]))
    unshaped = K_YAW * STICK           # what the old code delivered in tick 1
    check('first tick is not the whole step', first < 0.15,
          f'{first:.4f} Nm on tick 1 (was {unshaped:.2f} Nm unshaped)')
    check('torque slew is bounded by k_yaw*yaw_accel_limit*dt',
          slew <= K_YAW * YAW_ACCEL * DT + 1e-6,
          f'peak {slew:.4f} Nm/tick (limit {K_YAW * YAW_ACCEL * DT:.4f})')
    check('slew improved by >=10x vs the unshaped step',
          unshaped / max(slew, 1e-9) >= 10.0,
          f'{unshaped / max(slew, 1e-9):.0f}x')
    # yaw_cmd climbs at yaw_accel_limit until it reaches STICK: 0.375 s = 38 ticks
    reached = next((i - 1 for i, s in enumerate(seq)          # -1: skip the seed
                    if i > 0 and abs(s) >= K_YAW * STICK - 0.05), None)
    check('reaches full command in ~0.375 s', reached is not None and 33 <= reached <= 43,
          f'tick {reached} ({reached * DT:.3f} s)' if reached is not None else 'never reached')

    print('\n=== 2. releasing the stick ramps down, it does not step ===')
    rig = Rig()
    rig.settle()
    established = rig.drive(60, wz=STICK)               # full command held
    down = [t_yaw_of(established[-1])]                   # seed: see section 1
    down += [t_yaw_of(c) for c in rig.drive(60, wz=0.0)]  # stick centred
    rig.close()
    check('release slew also bounded',
          max(abs(b - a) for a, b in zip(down, down[1:])) <= K_YAW * YAW_ACCEL * DT + 1e-6,
          f'peak {max(abs(b - a) for a, b in zip(down, down[1:])):.4f} Nm/tick')
    check('yaw actually returns to zero', abs(down[-1]) < 0.05, f'{down[-1]:+.4f} Nm')

    print('\n=== 3. stale cmd_vel decelerates the turn, never steps it ===')
    # CLAUDE.md safety invariant: stale cmd_vel zeros the velocity references
    # and KEEPS BALANCING. It must not slam the differential torque to zero.
    rig = Rig()
    rig.settle()
    rig.drive(60, wz=STICK)            # turn established, command fresh
    rig.out.clear()
    # now the driver station vanishes: no more cmd_vel at all
    stale = [t_yaw_of(rig.tick()) for _ in range(120)]
    rig.close()
    check('no step when the watchdog fires',
          max(abs(b - a) for a, b in zip(stale, stale[1:])) <= K_YAW * YAW_ACCEL * DT + 1e-6,
          f'peak {max(abs(b - a) for a, b in zip(stale, stale[1:])):.4f} Nm/tick')
    check('and the turn does stop', abs(stale[-1]) < 0.05, f'{stale[-1]:+.4f} Nm')

    print('\n=== 4. saturation takes from YAW, never from BALANCE ===')
    # Pitch far enough off target that k3*pitch alone exceeds max_torque,
    # while a full yaw command competes for the same budget.
    rig = Rig()
    rig.settle()
    rig.drive(80, wz=STICK)             # yaw at full command, held
    worst_balance = None
    over = 0
    for pitch in [0.30, 0.45, 0.60, -0.30, -0.45, -0.60]:
        rig.cmd(wz=STICK)
        cmd = rig.tick(pitch=pitch)
        if max(abs(cmd[0]), abs(cmd[1])) > MAX_TORQUE + 1e-6:
            over += 1
        want = max(-MAX_TORQUE, min(MAX_TORQUE, 20.0 * pitch))
        err = abs(balance_of(cmd) - want)
        if worst_balance is None or err > worst_balance:
            worst_balance = err
    rig.close()
    check('neither wheel ever exceeds max_torque', over == 0, f'{over} violations')
    check('balance torque is preserved under saturation', worst_balance < 1e-6,
          f'worst deviation {worst_balance:.2e} Nm '
          f'(independent clamping lost 1.0 Nm here)')

    print('\n=== 5. yaw still works when there is headroom ===')
    rig = Rig()
    rig.settle()
    cmd = rig.drive(80, wz=STICK)[-1]   # upright, so balance uses ~no torque
    rig.close()
    check('full yaw authority when balance is idle',
          abs(t_yaw_of(cmd) - K_YAW * STICK) < 0.05,
          f't_yaw {t_yaw_of(cmd):+.3f} Nm (want {K_YAW * STICK:+.3f})')

    print('\n' + ('ALL CHECKS PASSED' if not failures
                  else f'{len(failures)} FAILED: {failures}'))
    return 1 if failures else 0


if __name__ == '__main__':
    sys.exit(main())
