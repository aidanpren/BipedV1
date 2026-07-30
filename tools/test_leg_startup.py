"""No-hardware test of leg_controller's STARTUP sequence.

The thing being tested is the one that breaks linkages: at boot the node must
command the position the legs are ALREADY AT, then ramp to home. If it seeds
the ramp at home instead, the first frame on the bus is a full-magnitude
position step and a loaded leg slams.

Runs entirely in one process on python-can's 'virtual' bus, so it needs no
vcan, no root, no CANable and no motors:

    python3 tools/test_leg_startup.py

Everything sniffed off the wire is MOTOR-shaft turns, exactly like hardware.

NOTE on load_torque in these fixtures: the axes are IDLE while establish_zero
takes its reading, so a LOADED fake leg is physically falling as we measure it.
That is real behaviour, not a bug — an unpowered leg does collapse. The cases
that assert exact numbers therefore use an UNLOADED leg, and the loaded case
asserts the ramp-rate invariant instead, which holds regardless of drift.
"""
import os
import struct
import sys
import threading
import time

import can
import rclpy

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                '..', 'src', 'robot_base'))
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from fake_odrive import FakeODrive                                  # noqa: E402
from robot_base.leg_controller import LegController                 # noqa: E402
from robot_base.odrive_can import CMD_SET_INPUT_POS                 # noqa: E402

CHANNEL = 'bipedtest'
GEAR = 8.0
LEFT_ID, RIGHT_ID = 3, 1        # matches real.yaml
RATE = 50.0
MAX_SPEED = 0.15                # output turns/s
STEP = MAX_SPEED / RATE         # the biggest legal jump per update


class Rig:
    """Fake legs + a bus sniffer, torn down together.

    Teardown matters: a FakeODrive left running on the same virtual channel
    keeps answering for its node ID, so the next case would see TWO responders
    and read whichever replied last.
    """

    def __init__(self, raw_left, raw_right, load_torque=0.0):
        self.buses = []
        self.fakes = []
        self.cmds = {LEFT_ID: [], RIGHT_ID: []}     # OUTPUT turns, in order
        self.running = True

        self.sniff = self._bus()
        threading.Thread(target=self._read, daemon=True).start()

        # NOTE the load SIGN. Gravity retracts both legs, i.e. it pushes both
        # in the -LOGICAL direction — but the right leg is inverted, so that is
        # the +RAW direction on that motor. Same raw sign on both would model a
        # robot whose legs fight each other, which is not this robot.
        for node_id, raw, invert in ((LEFT_ID, raw_left, +1.0),
                                     (RIGHT_ID, raw_right, -1.0)):
            drv = FakeODrive(self._bus(), node_id=node_id)
            drv.pos = raw * GEAR                    # wire units are motor turns
            drv.load_torque = load_torque * invert
            drv.vel_integrator_gain = 5.0           # what holds the weight
            threading.Thread(target=drv.run, daemon=True).start()
            self.fakes.append(drv)

    def _bus(self):
        bus = can.interface.Bus(interface='virtual', channel=CHANNEL)
        self.buses.append(bus)
        return bus

    def _read(self):
        while self.running:
            msg = self.sniff.recv(timeout=0.1)
            if msg is None or msg.is_remote_frame:
                continue
            node, cmd = msg.arbitration_id >> 5, msg.arbitration_id & 0x1F
            if cmd == CMD_SET_INPUT_POS and node in self.cmds:
                pos_motor = struct.unpack('<fhh', bytes(msg.data)[:8])[0]
                self.cmds[node].append(pos_motor / GEAR)

    def close(self):
        self.running = False
        for f in self.fakes:
            f.running = False
        time.sleep(0.05)
        for b in self.buses:
            b.shutdown()


def run_controller(params, seconds):
    argv = ['leg_controller', '--ros-args']
    for k, v in params.items():
        argv += ['-p', f'{k}:={v}']
    rclpy.init(args=argv)
    try:
        ctrl = LegController()
    except RuntimeError:
        rclpy.shutdown()
        raise
    deadline = time.time() + seconds
    while time.time() < deadline:
        rclpy.spin_once(ctrl.node, timeout_sec=0.02)
    ctrl.running = False
    ctrl.bus.shutdown()
    rclpy.shutdown()


BASE = {
    'can_interface': 'virtual', 'can_channel': CHANNEL,
    'left_node_id': LEFT_ID, 'right_node_id': RIGHT_ID,
    'gear_ratio': GEAR, 'invert_left': 'false', 'invert_right': 'true',
    'publish_rate': RATE, 'max_speed': MAX_SPEED,
    'pos_min': 0.0, 'pos_max': 0.10, 'home_position': 0.0,
    'current_limit': 5.0, 'vel_limit': 5.0,
    'idle_on_shutdown': 'false',
}

failures = []


def check(label, ok, detail=''):
    print(f'  {"PASS" if ok else "FAIL"}  {label}' + (f'   [{detail}]' if detail else ''))
    if not ok:
        failures.append(label)


def banner(text):
    print(f'\n{text}', flush=True)


# ── 1. the slam test ────────────────────────────────────────────────────────
# Legs sitting at logical +0.10 turns, home is 0.0. A correct node seeds the
# ramp at +0.10 and walks down. The old one commanded 0.0 on the first frame.
banner('[1] boot away from home -> must ramp, not step')
# logical = invert*(raw - zero). Left invert +1, right invert -1, so the same
# logical +0.10 sits at opposite RAW signs. This also checks the sign handling.
rig = Rig(raw_left=+0.10, raw_right=-0.10)
run_controller(dict(BASE, use_measured_zero='true',
                    zero_raw_left=0.0, zero_raw_right=0.0), seconds=3.0)
left, right = rig.cmds[LEFT_ID], rig.cmds[RIGHT_ID]
rig.close()

check('commanded something', len(left) > 20, f'{len(left)} frames')
if left:
    # update() ramps THEN sends, so the very first frame is already one legal
    # step along. Anything more than that is a step, which is the bug.
    check('first command is at the MEASURED position, not home',
          abs(left[0] - 0.10) <= STEP * 1.01,
          f'first={left[0]:+.5f} expected +0.10000 +/- one step {STEP:.5f}')
    jumps = [abs(b - a) for a, b in zip(left, left[1:])]
    check('no command ever jumps more than one ramp step',
          max(jumps) <= STEP * 1.5, f'max jump={max(jumps):.5f}')
    check('converged to home', abs(left[-1]) < 1e-3, f'final={left[-1]:+.5f}')
if right:
    check('inverted leg mirrors it in RAW', abs(right[0] + 0.10) <= STEP * 1.01,
          f'first={right[0]:+.5f} expected -0.10000 +/- one step')
    check('inverted leg converged to home', abs(right[-1]) < 1e-3,
          f'final={right[-1]:+.5f}')

# ── 2. the same, but carrying the robot's weight ────────────────────────────
banner("[2] same boot, loaded leg -> ramp rate still respected")
rig = Rig(raw_left=+0.10, raw_right=-0.10, load_torque=0.15)
run_controller(dict(BASE, use_measured_zero='true',
                    zero_raw_left=0.0, zero_raw_right=0.0), seconds=3.0)
left = rig.cmds[LEFT_ID]
rig.close()
if left:
    jumps = [abs(b - a) for a, b in zip(left, left[1:])]
    check('no command jumps more than one ramp step under load',
          max(jumps) <= STEP * 1.5, f'max jump={max(jumps):.5f}')
    # it sagged while IDLE, so it seeds BELOW +0.10 — but never above it, and
    # never at home. Between the two is the whole point.
    check('seeded between home and the parked position',
          0.0 < left[0] <= 0.10 + 1e-4, f'first={left[0]:+.5f}')
    check('converged to home under load', abs(left[-1]) < 1e-3,
          f'final={left[-1]:+.5f}')

# ── 3. a non-zero measured zero must shift the whole frame ──────────────────
banner('[3] non-zero zero_raw_* offsets the commanded raw position')
# raw +0.30 with zero at +0.30 means the leg IS at logical 0.0 = home already.
rig = Rig(raw_left=+0.30, raw_right=-0.30)
run_controller(dict(BASE, use_measured_zero='true',
                    zero_raw_left=0.30, zero_raw_right=-0.30), seconds=1.5)
left = rig.cmds[LEFT_ID]
rig.close()
if left:
    check('already at home -> commands the RAW zero, not 0.0',
          abs(left[0] - 0.30) < 1e-4, f'first={left[0]:+.5f} expected +0.30000')
    check('and never moves', max(abs(v - 0.30) for v in left) < 1e-4)

# ── 3b. the encoder wrap ────────────────────────────────────────────────────
# The killer case. A single-turn absolute encoder reports modulo ONE MOTOR TURN
# (1/8 = 0.125 output turns here), and the window boundary sits at an arbitrary
# rotor angle — in practice inside our travel. A leg truly at logical +0.05 with
# zero at +0.10 has raw 0.15, which WRAPS to 0.025. Plain subtraction would read
# that as -0.075: the wrong end of the stroke, and it would drive there.
banner('[3b] boot reading past the encoder wrap -> unwrapped correctly')
WINDOW = 1.0 / GEAR
TRUE_LOGICAL = 0.05
ZERO_L, ZERO_R = 0.10, -0.10
rig = Rig(raw_left=ZERO_L + TRUE_LOGICAL - WINDOW,      # +0.025, wrapped
          raw_right=ZERO_R - TRUE_LOGICAL + WINDOW)     # -0.025, wrapped
run_controller(dict(BASE, use_measured_zero='true', pos_max=0.10,
                    zero_raw_left=ZERO_L, zero_raw_right=ZERO_R), seconds=2.5)
left, right = rig.cmds[LEFT_ID], rig.cmds[RIGHT_ID]
rig.close()
if left:
    # seeded at the TRUE logical position, expressed in the wrapped raw frame
    check('unwrapped to the correct end of the stroke',
          abs(left[0] - (ZERO_L + TRUE_LOGICAL - WINDOW)) <= STEP * 1.01,
          f'first={left[0]:+.5f} expected {ZERO_L + TRUE_LOGICAL - WINDOW:+.5f}')
    jumps = [abs(b - a) for a, b in zip(left, left[1:])]
    check('still no jump bigger than one ramp step', max(jumps) <= STEP * 1.5,
          f'max jump={max(jumps):.5f}')
    check('inverted leg unwrapped too',
          abs(right[0] - (ZERO_R - TRUE_LOGICAL + WINDOW)) <= STEP * 1.01,
          f'first={right[0]:+.5f} expected {ZERO_R - TRUE_LOGICAL + WINDOW:+.5f}')

# ── 4. refuse to arm with no feedback ───────────────────────────────────────
banner('[4] no encoder feedback -> refuse to arm')
try:
    run_controller(dict(BASE, can_channel='emptybus', arm_timeout=0.5), seconds=0.1)
    check('raised RuntimeError', False, 'it armed anyway')
except RuntimeError as exc:
    check('raised RuntimeError', True, str(exc)[:55] + '...')

# ── 5. refuse to arm when the legs disagree ─────────────────────────────────
banner('[5] legs disagree at boot -> refuse to arm')
rig = Rig(raw_left=+0.10, raw_right=-0.02)      # 0.08 logical turns apart
try:
    run_controller(dict(BASE, use_measured_zero='true', zero_raw_left=0.0,
                        zero_raw_right=0.0, max_leg_mismatch=0.03), seconds=0.1)
    check('raised RuntimeError', False, 'it armed anyway')
except RuntimeError as exc:
    check('raised RuntimeError', True, str(exc)[:55] + '...')
rig.close()

# ── 6. soft travel limits still clamp ───────────────────────────────────────
banner('[6] home_position beyond pos_max is clamped, not obeyed')
rig = Rig(raw_left=0.0, raw_right=0.0)
run_controller(dict(BASE, use_measured_zero='true', zero_raw_left=0.0,
                    zero_raw_right=0.0, home_position=0.5), seconds=3.0)
left = rig.cmds[LEFT_ID]
rig.close()
if left:
    check('never commanded past pos_max', max(left) <= 0.10 + 1e-6,
          f'max={max(left):+.5f} limit=0.10')

print(f'\n{"ALL PASS" if not failures else f"{len(failures)} FAILURE(S): {failures}"}')
sys.exit(1 if failures else 0)
