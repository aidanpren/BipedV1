"""No-hardware test: can the legs hold the robot's weight, and what does the
damping/integrator actually change?

Runs leg_controller against two fake ODrives carrying a constant load torque
(the robot's weight) on an in-process virtual CAN bus.

    python3 tools/test_leg_hold.py
"""
import math
import os
import sys
import threading
import time

import can
import rclpy
from std_msgs.msg import Float64

ROOT = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..')
sys.path.insert(0, os.path.join(ROOT, 'src', 'robot_base'))
sys.path.insert(0, os.path.join(ROOT, 'tools'))

from fake_odrive import FakeODrive                        # noqa: E402
from robot_base.leg_controller import LegController       # noqa: E402

CHANNEL = 'legbus'
LOAD = 0.15          # Nm at the motor shaft = the robot's weight on a leg
failures = []


def check(name, cond, detail=''):
    print(f"  [{'PASS' if cond else 'FAIL'}] {name}" + (f"  {detail}" if detail else ''))
    if not cond:
        failures.append(name)


def settle(ctrl, seconds):
    end = time.time() + seconds
    while time.time() < end:
        rclpy.spin_once(ctrl.node, timeout_sec=0.005)


def main():
    rclpy.init(args=['--ros-args',
                     '-p', 'can_interface:=virtual',
                     '-p', f'can_channel:={CHANNEL}',
                     '-p', 'publish_rate:=100.0',
                     '-p', 'max_speed:=0.5'])

    fakes = {}
    for name, nid in (('left', 3), ('right', 4)):
        bus = can.interface.Bus(interface='virtual', channel=CHANNEL)
        drv = FakeODrive(bus, node_id=nid)
        threading.Thread(target=drv.run, daemon=True).start()
        fakes[name] = drv

    ctrl = LegController()
    cmd = ctrl.node.create_publisher(Float64, 'leg_position_cmd', 10)
    settle(ctrl, 0.5)

    check('both leg ODrives armed in POSITION mode',
          all(f.axis_state == 8 and f.control_mode == 3 for f in fakes.values()),
          f"left={fakes['left'].control_mode} right={fakes['right'].control_mode}")

    print('\n=== 1. UNLOADED: does it reach the commanded position? ===')
    cmd.publish(Float64(data=0.2))
    settle(ctrl, 3.0)
    sag_unloaded = max(abs(s) for s in ctrl.sag)
    check('reaches target with no load', sag_unloaded < 0.01,
          f'sag {sag_unloaded:.4f} turns')

    print(f'\n=== 2. LOADED ({LOAD} Nm), pure P — expect SAG ===')
    for f in fakes.values():
        f.load_torque = LOAD
        f._vel_integrator = 0.0
        f.vel_integrator_gain = 0.0
    settle(ctrl, 3.0)
    sag_p = max(abs(s) for s in ctrl.sag)
    # Steady state: tau = vel_gain * pos_gain * err  ->  err = load/(vg*pg).
    # That error is in MOTOR turns; ctrl.sag reports OUTPUT turns, so divide
    # by the gear ratio. (Yes — the gear trap, inside the test written to
    # catch the gear trap.)
    gear = 8.0
    predicted = LOAD / (fakes['left'].vel_gain * fakes['left'].pos_gain) / gear
    check('load causes measurable sag', sag_p > 0.01,
          f'sag {sag_p:.4f} turns (theory {predicted:.4f})')
    check('sag matches the P-control prediction',
          abs(sag_p - predicted) < 0.005,
          f'{sag_p:.4f} vs {predicted:.4f}')

    print('\n=== 3. more DAMPING (vel_gain x4) — less sag ===')
    for f in fakes.values():
        f.vel_gain = 0.20
        f._vel_integrator = 0.0
    settle(ctrl, 3.0)
    sag_damped = max(abs(s) for s in ctrl.sag)
    check('stiffer velocity gain reduces sag', sag_damped < sag_p * 0.5,
          f'{sag_damped:.4f} vs {sag_p:.4f} turns')

    print('\n=== 4. INTEGRATOR — sag should go to ~0 ===')
    for f in fakes.values():
        f.vel_integrator_gain = 2.0
    settle(ctrl, 4.0)
    sag_i = max(abs(s) for s in ctrl.sag)
    check('integrator removes steady-state sag', sag_i < 0.01,
          f'sag {sag_i:.4f} turns')

    print('\n=== 5. SAFETY: travel limits and ramping ===')
    cmd.publish(Float64(data=99.0))          # way past pos_max
    settle(ctrl, 0.3)
    check('command clamped to pos_max', abs(ctrl.target - ctrl.pos_max) < 1e-9,
          f'target {ctrl.target} (pos_max {ctrl.pos_max})')

    # Now a full-travel move, sampled BEFORE it can finish: the setpoint must
    # be in transit, not snapped to the target.
    before = ctrl.setpoint
    cmd.publish(Float64(data=-99.0))         # way past pos_min
    settle(ctrl, 0.08)                       # ~8 ticks of a ~1 s move
    check('command clamped to pos_min', abs(ctrl.target - ctrl.pos_min) < 1e-9,
          f'target {ctrl.target} (pos_min {ctrl.pos_min})')
    check('setpoint ramps, does not step',
          abs(ctrl.setpoint - ctrl.target) > 0.05 and abs(ctrl.setpoint - before) > 1e-6,
          f'setpoint {ctrl.setpoint:.4f} in transit from {before:.4f} '
          f'to {ctrl.target:.4f}')

    ctrl.shutdown()
    rclpy.shutdown()
    print('\n' + ('ALL CHECKS PASSED' if not failures
                  else f'{len(failures)} FAILED: {failures}'))
    return 1 if failures else 0


if __name__ == '__main__':
    sys.exit(main())
