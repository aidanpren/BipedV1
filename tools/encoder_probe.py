"""Read ODrive encoder positions over CAN. READ-ONLY — never arms, never
commands torque, never changes a mode. Safe to run with the robot on a stand,
on the ground, or half-assembled.

Its main job is the POWER-CYCLE TEST: find out what pos_estimate does across a
reboot, which decides whether the legs need a homing routine at every power-on
or can rely on the MA732's absolute reading.

    # 1. park a leg somewhere, then record where it is
    python3 tools/encoder_probe.py --snapshot midstroke-before

    # 2. power cycle the MOTORS ONLY. Do not move the leg. Then:
    python3 tools/encoder_probe.py --snapshot midstroke-after

    # 3. repeat for a few positions, then ask what it means
    python3 tools/encoder_probe.py --compare

Other modes:
    python3 tools/encoder_probe.py                 # one reading, all leg nodes
    python3 tools/encoder_probe.py --watch         # live, for finding hard stops
    python3 tools/encoder_probe.py --nodes 0 1 2 3 # wheels too

Everything on the wire is MOTOR-shaft turns. Output turns = motor / gear_ratio.
"""
import argparse
import json
import os
import sys
import time

import can

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                '..', 'src', 'robot_base'))

from robot_base.odrive_can import (                                 # noqa: E402
    CMD_GET_ENCODER_ESTIMATES, ODriveClient,
)

STORE = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                     '.encoder_snapshots.json')
DEFAULT_NODES = [1, 3]          # the two hips; see real.yaml
NAMES = {0: 'right wheel', 1: 'right hip', 2: 'left wheel', 3: 'left hip'}


def read_once(bus, nodes, timeout=1.0, samples=5):
    """RTR-poll each node. Returns {node_id: motor_turns}. Missing nodes absent.

    gear_ratio=1.0 on purpose: this tool reports RAW MOTOR turns, because the
    single-turn wrap we are hunting happens at the motor, not the output.
    """
    clients = {n: ODriveClient(bus, node_id=n, gear_ratio=1.0) for n in nodes}
    out = {}
    deadline = time.time() + timeout
    got = 0
    while time.time() < deadline and got < samples * len(nodes):
        for c in clients.values():
            c.request(CMD_GET_ENCODER_ESTIMATES)
        time.sleep(0.05)
        while True:
            msg = bus.recv(timeout=0.0)
            if msg is None:
                break
            for n, c in clients.items():
                d = c.decode(msg)
                if d and d[0] == 'encoder':
                    out[n] = d[1][0]        # motor turns
                    got += 1
                    break
    return out


def fmt(readings, gear):
    lines = []
    for n in sorted(readings):
        m = readings[n]
        lines.append(f'  node {n} ({NAMES.get(n, "?"):<11}) '
                     f'motor {m:+10.5f} turns   output {m / gear:+9.5f} turns   '
                     f'(mod 1 motor turn: {m % 1.0:.5f})')
    return '\n'.join(lines) if lines else '  (no response from any node)'


def load():
    if os.path.exists(STORE):
        with open(STORE) as f:
            return json.load(f)
    return {}


def save(store):
    with open(STORE, 'w') as f:
        json.dump(store, f, indent=2)


def compare(store, gear):
    """Pair up <label>-before / <label>-after snapshots and say what happened."""
    labels = sorted({k.rsplit('-', 1)[0] for k in store
                     if k.endswith('-before') or k.endswith('-after')})
    if not labels:
        print('No -before/-after pairs recorded yet. Snapshot with names like\n'
              '  --snapshot midstroke-before   (then power cycle)\n'
              '  --snapshot midstroke-after')
        return

    verdicts = []
    for label in labels:
        before = store.get(f'{label}-before', {}).get('readings', {})
        after = store.get(f'{label}-after', {}).get('readings', {})
        if not before or not after:
            print(f'\n{label}: incomplete pair, skipping')
            continue
        print(f'\n{label}:')
        for n in sorted(set(before) & set(after), key=int):
            b, a = before[n], after[n]
            d = a - b
            turns_from_zero = abs(b)
            if abs(a) < 1e-3 and turns_from_zero > 0.05:
                verdict = 'ZEROED at boot'
            elif abs(d) < 0.01:
                verdict = 'PERSISTED (multi-turn survived)'
            elif abs(d - round(d)) < 0.01 and abs(round(d)) >= 1:
                verdict = f'WRAPPED by {round(d):+d} motor turn(s)'
            else:
                verdict = 'INCONSISTENT — drifted, or the leg moved'
            flag = '' if turns_from_zero > 1.0 else '   [<1 motor turn: not conclusive]'
            print(f'  node {n} ({NAMES.get(int(n), "?"):<11}) '
                  f'before {b:+9.5f}  after {a:+9.5f}  delta {d:+9.5f}   '
                  f'{verdict}{flag}')
            if turns_from_zero > 1.0:
                verdicts.append(verdict.split()[0])

    print('\n' + '=' * 72)
    if not verdicts:
        print('NO CONCLUSIVE SAMPLES. Every pair was recorded less than one motor\n'
              'turn from the encoder origin, where wrapped and persisted readings\n'
              'look identical. Redo at least one pair with the leg parked MORE\n'
              f'than 1.0 motor turn ({1.0 / gear:.4f} output turns) from the stop.')
    elif all(v == 'PERSISTED' for v in verdicts):
        print('VERDICT: multi-turn count PERSISTS across a reboot.\n'
              'The single-turn reasoning does not bind. You can raise\n'
              'leg_controller pos_max toward 0.19 (stay under the 0.1937 toggle\n'
              'point) and take the full 0.302 m of lift back.')
    elif all(v == 'WRAPPED' for v in verdicts):
        print('VERDICT: pos_estimate is re-derived from the SINGLE-TURN absolute\n'
              'reading at boot. The design in leg_controller is correct as-is:\n'
              'keep pos_max at 0.10, measure zero_raw_* once, set\n'
              'use_measured_zero, and it never homes again.')
    elif all(v == 'ZEROED' for v in verdicts):
        print('VERDICT: no absolute restore — position is lost on every boot.\n'
              'Neither the zero constant nor the unwrap can help. The legs need\n'
              'a real homing routine (drive gently to the retracted stop at low\n'
              'current, detect stall, zero there).')
    else:
        print(f'VERDICT: MIXED results {set(verdicts)} — do not design against\n'
              'this yet. Re-run; most likely the leg moved between snapshots, or\n'
              'the axes are configured differently from each other.')
    print('=' * 72)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--interface', default='socketcan')
    ap.add_argument('--channel', default='can0')
    ap.add_argument('--nodes', type=int, nargs='+', default=DEFAULT_NODES)
    ap.add_argument('--gear-ratio', type=float, default=8.0)
    ap.add_argument('--watch', action='store_true',
                    help='live readout — use it to find the hard stops by hand')
    ap.add_argument('--snapshot', metavar='LABEL',
                    help='record a reading under LABEL, e.g. midstroke-before')
    ap.add_argument('--compare', action='store_true',
                    help='interpret the recorded -before/-after pairs')
    ap.add_argument('--clear', action='store_true', help='delete all snapshots')
    a = ap.parse_args()

    if a.clear:
        if os.path.exists(STORE):
            os.remove(STORE)
        print('snapshots cleared')
        return 0

    store = load()
    if a.compare:
        compare(store, a.gear_ratio)
        return 0

    bus = can.interface.Bus(interface=a.interface, channel=a.channel,
                            bitrate=500000)
    try:
        if a.watch:
            print(f'watching nodes {a.nodes} on {a.channel} — Ctrl-C to stop\n')
            while True:
                r = read_once(bus, a.nodes, timeout=0.4, samples=2)
                print('\033[2J\033[H' + fmt(r, a.gear_ratio), flush=True)
        r = read_once(bus, a.nodes)
        print(fmt(r, a.gear_ratio))
        missing = [n for n in a.nodes if n not in r]
        if missing:
            print(f'\nWARNING: no reply from node(s) {missing}. Check the bus is '
                  'up and the node IDs are right before trusting this.')
        if a.snapshot:
            if missing:
                print('Refusing to snapshot an incomplete reading.')
                return 1
            store[a.snapshot] = {'t': time.time(),
                                 'readings': {str(k): v for k, v in r.items()}}
            save(store)
            print(f'\nsaved as "{a.snapshot}" ({len(store)} snapshots stored)')
    except KeyboardInterrupt:
        pass
    finally:
        bus.shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
