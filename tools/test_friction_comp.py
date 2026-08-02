"""No-hardware test for odrive_bridge's friction compensation (2026-08-02).

    python3 tools/test_friction_comp.py

Covers the two things added to fight the standstill hunting:
  * friction_feedforward — Coulomb FF, for DRIVING
  * dither              — stiction linearisation, for STANDSTILL

and the one thing that must never break: [0.0, 0.0] means CUT TORQUE, so a
DISABLED robot stays silent even with both effects switched on. balance_
controller keeps publishing at 100 Hz while disabled, so without that contract
dither would energise motors the operator deliberately shut off.

Runs on python-can's in-process 'virtual' bus — no sudo, no vcan, no hardware.
Torques are captured at the ODriveClient boundary, which is the last point
before CAN, so what is asserted is what the motor would actually receive.
"""
import math
import os
import sys
import threading

import can
import rclpy
from rclpy.parameter import Parameter
from std_msgs.msg import Float64MultiArray

ROOT = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..')
sys.path.insert(0, os.path.join(ROOT, 'src', 'robot_base'))
sys.path.insert(0, os.path.join(ROOT, 'tools'))

from fake_odrive import FakeODrive                                   # noqa: E402
from robot_base.odrive_bridge import (                               # noqa: E402
    DITHER_PHASE, ODriveBridge, dither, friction_feedforward,
)

CHANNEL = 'frictionbus'
failures = []


def check(name, cond, detail=''):
    print(f"  [{'PASS' if cond else 'FAIL'}] {name}" + (f"  {detail}" if detail else ''))
    if not cond:
        failures.append(name)


def main():
    print('\n=== 1. friction_feedforward: shape and limits ===')
    check('zero at standstill — this term does NOT fix hunting',
          friction_feedforward(0.0, 0.5, 0.2) == 0.0,
          f'{friction_feedforward(0.0, 0.5, 0.2):.4f} Nm at v=0')
    check('saturates at +tau_c once moving',
          abs(friction_feedforward(5.0, 0.5, 0.2) - 0.5) < 1e-12,
          f'{friction_feedforward(5.0, 0.5, 0.2):+.4f} Nm')
    check('saturates at -tau_c in reverse',
          abs(friction_feedforward(-5.0, 0.5, 0.2) + 0.5) < 1e-12,
          f'{friction_feedforward(-5.0, 0.5, 0.2):+.4f} Nm')
    check('tapers linearly through the dead zone',
          abs(friction_feedforward(0.1, 0.5, 0.2) - 0.25) < 1e-12,
          f'{friction_feedforward(0.1, 0.5, 0.2):+.4f} Nm at half v_eps')
    check('never exceeds tau_c at any velocity',
          max(abs(friction_feedforward(v * 0.01, 0.5, 0.2))
              for v in range(-2000, 2001)) <= 0.5 + 1e-12)
    check('disabled when tau_c = 0 (the default)',
          friction_feedforward(5.0, 0.0, 0.2) == 0.0)
    check('safe when v_eps = 0 — no division blow-up',
          friction_feedforward(5.0, 0.5, 0.0) == 0.0)

    print('\n=== 2. dither: alternates, bounded, and cancels in antiphase ===')
    amp = 0.2

    def longest_run(seq):
        best = run = 1
        for a, b in zip(seq, seq[1:]):
            run = run + 1 if (a > 0) == (b > 0) else 1
            best = max(best, run)
        return best

    # Assert the WAVEFORM, not just its average: the old sin()>=0 form happened
    # to average out at some frequencies while being pure rounding noise at
    # others. Realistic settings only — see the Nyquist check below.
    for hz in (10.0, 15.0, 23.0, 25.0):
        samples = [dither(i / 100.0, amp, hz) for i in range(400)]   # 100 Hz
        mean = sum(samples) / len(samples)
        cap = math.ceil(100.0 / hz / 2.0) + 1
        check(f'{hz:.0f} Hz: bounded by amp',
              max(abs(s) for s in samples) <= amp + 1e-12,
              f'max |{max(abs(s) for s in samples):.3f}|')
        check(f'{hz:.0f} Hz: no DC torque bias', abs(mean) < 0.05 * amp,
              f'mean {mean:+.2e} Nm')
        check(f'{hz:.0f} Hz: square wave, not rounding noise',
              longest_run(samples) <= cap,
              f'longest same-sign run {longest_run(samples)} (cap {cap})')

    # NYQUIST IS DEGENERATE BY CONSTRUCTION — this is documentation, not a bug
    # to fix. At hz = rate/2 every sample lands exactly on the transition
    # boundary (phase == 0.5), so the sign is decided by whether (i/100)*50
    # rounds just under or just over. No square-wave formulation escapes that.
    # It is why the default is 25 Hz and why odrive_bridge warns above 30 Hz.
    # If this check ever FAILS, someone made Nyquist work and the default can
    # be revisited; until then, do not raise dither_hz toward the command rate.
    nyq = [dither(i / 100.0, amp, 50.0) for i in range(400)]
    check('50 Hz (Nyquist) is degenerate, as documented',
          abs(sum(nyq) / len(nyq)) > 1e-12 or longest_run(nyq) > 1,
          f'mean {sum(nyq) / len(nyq):+.2e}, longest run {longest_run(nyq)}')
    check('disabled when amp = 0 (the default)', dither(0.123, 0.0, hz) == 0.0)
    check('disabled when hz = 0', dither(0.123, amp, 0.0) == 0.0)
    check('ANTIPHASE: wheels cancel, so no fore-aft pitch disturbance',
          DITHER_PHASE['left'] + DITHER_PHASE['right'] == 0.0,
          f"L{DITHER_PHASE['left']:+.0f} R{DITHER_PHASE['right']:+.0f}")

    # ── bridge integration ────────────────────────────────────────────────
    rclpy.init(args=['--ros-args', '-p', 'can_interface:=virtual',
                     '-p', f'can_channel:={CHANNEL}', '-p', 'publish_rate:=50.0'])
    for nid in (1, 2):
        bus = can.interface.Bus(interface='virtual', channel=CHANNEL)
        threading.Thread(target=FakeODrive(bus, node_id=nid).run, daemon=True).start()

    bridge = ODriveBridge()
    sent = {}
    for nm, cl in bridge.clients.items():
        cl.set_torque = (lambda n: (lambda t: sent.__setitem__(n, t)))(nm)

    def tune(**kw):
        bridge.node.set_parameters(
            [Parameter(k, Parameter.Type.DOUBLE, float(v)) for k, v in kw.items()])

    def send(left, right):
        sent.clear()
        bridge.command_callback(Float64MultiArray(data=[left, right]))
        return sent

    def robot_frame(name):
        """Undo the motor-side inversion to compare against the command."""
        return sent[name] / bridge.invert[name]

    print('\n=== 3. THE SAFETY CONTRACT: [0,0] cuts torque, even armed ===')
    tune(friction_ff=0.5, friction_v_eps=0.2, dither_torque=0.2, dither_hz=50.0)
    with bridge._lock:                      # pretend the wheels are rolling fast
        bridge.fb['left'] = [0.0, 1.0]
        bridge.fb['right'] = [0.0, -1.0]
    out = send(0.0, 0.0)
    check('both wheels get EXACTLY zero with dither+FF enabled',
          out.get('left') == 0.0 and out.get('right') == 0.0,
          f"L {out.get('left')} R {out.get('right')}")
    # And prove the test would have caught the bug: same state, nonzero command
    out = send(1.0, 1.0)
    check('...while a live command DOES get compensated',
          abs(robot_frame('left') - 1.0) > 1e-9,
          f"left {robot_frame('left'):+.4f} vs commanded 1.0")

    print('\n=== 4. dither reaches the motors in antiphase ===')
    tune(friction_ff=0.0, dither_torque=0.2)
    with bridge._lock:
        bridge.fb['left'] = [0.0, 0.0]
        bridge.fb['right'] = [0.0, 0.0]
    send(1.0, 1.0)
    dl, dr = robot_frame('left') - 1.0, robot_frame('right') - 1.0
    check('each wheel gets the full dither amplitude',
          abs(abs(dl) - 0.2) < 1e-9 and abs(abs(dr) - 0.2) < 1e-9,
          f'L {dl:+.4f} R {dr:+.4f} Nm')
    check('and they sum to zero — no net pitch disturbance',
          abs(dl + dr) < 1e-12, f'sum {dl + dr:+.2e} Nm')

    print('\n=== 5. feedforward follows the WHEEL, through the inversion ===')
    tune(friction_ff=0.5, friction_v_eps=0.2, dither_torque=0.0)
    # Set raw motor-side feedback so BOTH wheels roll FORWARD in robot frame.
    with bridge._lock:
        bridge.fb['left'] = [0.0, 1.0 / bridge.invert['left']]
        bridge.fb['right'] = [0.0, 1.0 / bridge.invert['right']]
    send(1.0, 1.0)
    check('both wheels rolling forward -> both get +tau_c',
          abs(robot_frame('left') - 1.5) < 1e-9
          and abs(robot_frame('right') - 1.5) < 1e-9,
          f"L {robot_frame('left'):+.4f} R {robot_frame('right'):+.4f} (want +1.5)")
    with bridge._lock:                      # now roll BACKWARD in robot frame
        bridge.fb['left'] = [0.0, -1.0 / bridge.invert['left']]
        bridge.fb['right'] = [0.0, -1.0 / bridge.invert['right']]
    send(1.0, 1.0)
    check('both wheels rolling backward -> both get -tau_c',
          abs(robot_frame('left') - 0.5) < 1e-9
          and abs(robot_frame('right') - 0.5) < 1e-9,
          f"L {robot_frame('left'):+.4f} R {robot_frame('right'):+.4f} (want +0.5)")

    print('\n=== 6. defaults are inert — installing this changes nothing ===')
    tune(friction_ff=0.0, dither_torque=0.0)
    with bridge._lock:
        bridge.fb['left'] = [0.0, 1.0]
        bridge.fb['right'] = [0.0, 1.0]
    send(2.0, -3.0)
    check('command passes through untouched at default params',
          abs(robot_frame('left') - 2.0) < 1e-12
          and abs(robot_frame('right') + 3.0) < 1e-12,
          f"L {robot_frame('left'):+.4f} R {robot_frame('right'):+.4f}")

    bridge.running = False
    rclpy.shutdown()
    print('\n' + ('ALL CHECKS PASSED' if not failures
                  else f'{len(failures)} FAILED: {failures}'))
    return 1 if failures else 0


if __name__ == '__main__':
    sys.exit(main())
