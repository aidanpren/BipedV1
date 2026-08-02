#!/usr/bin/env python3
"""Offline analysis of a station-keeping rosbag: who owns the slow rock?

Three hypotheses, one dataset. Record ~90 s of the robot station keeping
(enable RELEASED, so the watchdog holds the commanded targets at hard zero):

    ros2 bag record /imu /joint_states /wheel_effort_controller/commands /cmd_vel -o rock_A

then run:

    python3 tools/analyze_rock.py rock_A [--a1 -0.07 --a2 -0.18 --trim 0.06]

The gain flags MUST match what the controller was actually running (check with
`ros2 param get /balance_controller a1` etc.) — the reconstruction in panel B
is only as honest as these numbers.

What each verdict means:

  A. MECHANISM — a friction/backlash dead zone between commanded wheel torque
     and wheel motion. Signature: the wheels DWELL at v=0 far more than a
     smooth oscillation allows, while commanded torque ramps through a band
     around zero; the torque-vs-acceleration cross-plot has a flat middle.
     Note the "plant" here INCLUDES odrive_bridge's dither/friction_ff — the
     commands topic is upstream of them, which is exactly what we want: it
     measures the plant as the balance law sees it.
  B. LAW — measured pitch faithfully tracks the reconstructed pitch_target
     through every reversal, yet the ensemble still orbits. Then the plant is
     doing everything it is asked and the defect is upstream: the law or its
     inputs. Poor tracking instead points back at A.
  C. SENSOR FUSION — the BNO085's fused pitch contains position-correlated
     error (magnetometer steering and/or accel-derived false tilt). Signature:
     the (fused pitch − integrated gyro) residual tracks x at the rock
     frequency. The gyro cannot lie about short-horizon CHANGES in pitch, so
     whatever the residual holds is what fusion ADDED.

Writes <bag>_analysis.png next to the bag (no GUI window — the VS Code snap
breaks Qt) and prints verdict hints. Needs only numpy + matplotlib for
--selftest; reading a real bag additionally needs the ROS 2 Jazzy python libs.

    python3 tools/analyze_rock.py --selftest

synthesizes a rock with a planted fusion-residual correlation and checks the
pipeline detects it — run it once before trusting the tool with robot time.
"""
import argparse
import math
import sys
from pathlib import Path

import numpy as np
import matplotlib
matplotlib.use('Agg')          # PNG only; Qt windows crash in the VS Code snap
import matplotlib.pyplot as plt

DEG = 180.0 / math.pi
DT = 0.01                      # uniform analysis grid, s
ROCK_LO, ROCK_HI = 0.03, 0.4   # Hz band that brackets the ~0.11 Hz rock
DWELL_V = 0.003                # m/s; "the wheels are not moving" threshold

# dataviz reference palette (light mode) — see the skill's palette.md
INK, MUTED, GRID_C = '#0b0b0b', '#898781', '#e1e0d9'
BLUE, ORANGE, AQUA = '#2a78d6', '#eb6834', '#1baf7a'
SURFACE, BASELINE = '#fcfcfb', '#c3c2b7'


# ── zero-phase filtering, dependency-free ────────────────────────────────────
# First-order low-pass run forward then backward: the two passes' phase lags
# cancel exactly, which matters because panel B is a PHASE comparison and a
# causal filter would manufacture the very lag we are trying to measure.
def _fwd(sig, alpha):
    out = np.empty_like(sig)
    acc = sig[0]
    for i in range(len(sig)):
        acc += alpha * (sig[i] - acc)
        out[i] = acc
    return out


def lowpass(sig, fc, dt=DT):
    alpha = dt / (1.0 / (2.0 * math.pi * fc) + dt)
    return _fwd(_fwd(sig, alpha)[::-1], alpha)[::-1]


def bandpass(sig, f_lo=ROCK_LO, f_hi=ROCK_HI, dt=DT):
    return lowpass(sig, f_hi, dt) - lowpass(sig, f_lo, dt)


def norm_xcorr(a, b, max_lag=3.0):
    """Peak normalized cross-correlation and its lag.

    POSITIVE lag means `a` LAGS `b` (a's features happen later).
    """
    a = a - a.mean()
    b = b - b.mean()
    denom = math.sqrt(np.dot(a, a) * np.dot(b, b))
    if denom == 0.0:
        return 0.0, 0.0
    n = int(max_lag / DT)
    mid = len(a) - 1
    window = np.correlate(a, b, 'full')[mid - n: mid + n + 1] / denom
    k = int(np.argmax(np.abs(window))) - n
    return float(window[k + n]), k * DT


# ── bag reading (ROS imports stay in here so --selftest needs no ROS) ────────
def read_bag(path, wheel_radius):
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message

    last_exc = None
    for storage in ('', 'mcap', 'sqlite3'):
        reader = rosbag2_py.SequentialReader()
        try:
            reader.open(rosbag2_py.StorageOptions(uri=str(path), storage_id=storage),
                        rosbag2_py.ConverterOptions('', ''))
            break
        except Exception as exc:
            last_exc = exc
    else:
        sys.exit(f'cannot open bag {path}: {last_exc}')

    types = {t.name: t.type for t in reader.get_all_topics_and_types()}
    wanted = ('/imu', '/joint_states', '/wheel_effort_controller/commands', '/cmd_vel')
    classes = {n: get_message(types[n]) for n in wanted if n in types}
    for n in ('/imu', '/joint_states', '/wheel_effort_controller/commands'):
        if n not in classes:
            sys.exit(f'bag has no {n} — re-record with all four topics')

    raw = {n: [] for n in classes}
    while reader.has_next():
        topic, buf, t_ns = reader.read_next()
        if topic in classes:
            raw[topic].append((t_ns * 1e-9, deserialize_message(buf, classes[topic])))

    d = {'cv_n': 0, 'cv_nonzero': 0, 'cv_max_vx': 0.0}

    imu_t, pitch, gyro_y = [], [], []
    for t, m in raw['/imu']:
        q = m.orientation
        sinp = max(-1.0, min(1.0, 2.0 * (q.w * q.y - q.z * q.x)))
        imu_t.append(t)
        pitch.append(math.asin(sinp))          # same extraction as the controller
        gyro_y.append(m.angular_velocity.y)

    js_t, jx, jv = [], [], []
    for t, m in raw['/joint_states']:
        try:
            l = m.name.index('left_wheel_joint')
            r = m.name.index('right_wheel_joint')
        except ValueError:
            continue
        js_t.append(t)
        jx.append(wheel_radius * (m.position[l] + m.position[r]) / 2.0)
        jv.append(wheel_radius * (m.velocity[l] + m.velocity[r]) / 2.0)

    cmd_t, tau_c = [], []
    for t, m in raw['/wheel_effort_controller/commands']:
        if len(m.data) >= 2:
            cmd_t.append(t)
            tau_c.append((m.data[0] + m.data[1]) / 2.0)   # antiphase dither cancels here

    for t, m in raw.get('/cmd_vel', []):
        d['cv_n'] += 1
        if abs(m.linear.x) > 1e-6 or abs(m.angular.z) > 1e-6:
            d['cv_nonzero'] += 1
            d['cv_max_vx'] = max(d['cv_max_vx'], abs(m.linear.x))

    for name, tt in (('imu', imu_t), ('joint_states', js_t), ('commands', cmd_t)):
        if len(tt) < 100:
            sys.exit(f'only {len(tt)} {name} messages — bag too short or topic dead')

    d.update(imu_t=np.array(imu_t), pitch=np.array(pitch), gyro_y=np.array(gyro_y),
             js_t=np.array(js_t), jx=np.array(jx), jv=np.array(jv),
             cmd_t=np.array(cmd_t), tau_c=np.array(tau_c))
    return d


# ── synthetic dataset for --selftest ─────────────────────────────────────────
def synth(args):
    """A 90 s rock with a PLANTED fusion residual (0.3 deg, in phase with x).

    Not a physics simulation — it exists so the pipeline (bag-free path,
    filters, reconstruction, correlations, plotting) can be proven end-to-end
    before any robot time is spent, and so panel C has a known-positive to
    detect: the residual is added to fused pitch but NOT to the gyro, exactly
    how a real fusion artifact enters.
    """
    rng = np.random.default_rng(1)
    w = 2.0 * math.pi / 8.8

    t_i = np.arange(0.0, 90.0, 0.01)                     # imu, 100 Hz
    x_i = 0.13 * np.sin(w * t_i)
    v_i = 0.13 * w * np.cos(w * t_i)
    target = args.a1 * x_i + args.a2 * v_i + args.trim
    lag = int(0.10 / 0.01)                               # plant tracks with 100 ms lag
    true_pitch = np.roll(target, lag)
    true_pitch[:lag] = target[0]
    fused = true_pitch + 0.3 / DEG * (x_i / 0.13) + rng.normal(0, 2e-4, len(t_i))
    gyro = np.gradient(true_pitch, 0.01) + 0.002 + rng.normal(0, 5e-4, len(t_i))

    t_j = np.arange(0.0, 90.0, 0.02)                     # joint_states, 50 Hz
    x_j = 0.13 * np.sin(w * t_j) + rng.normal(0, 1e-4, len(t_j))
    v_j = 0.13 * w * np.cos(w * t_j) + rng.normal(0, 2e-3, len(t_j))

    tau = 0.4 * np.sin(w * t_i + 0.4) + rng.normal(0, 0.01, len(t_i))

    return {'imu_t': t_i, 'pitch': fused, 'gyro_y': gyro,
            'js_t': t_j, 'jx': x_j, 'jv': v_j,
            'cmd_t': t_i.copy(), 'tau_c': tau,
            'cv_n': 0, 'cv_nonzero': 0, 'cv_max_vx': 0.0}


# ── plotting helpers ─────────────────────────────────────────────────────────
def style(ax, title, ylabel, xlabel=None):
    ax.set_facecolor(SURFACE)
    ax.grid(color=GRID_C, linewidth=0.8)
    ax.set_axisbelow(True)
    for side in ('top', 'right'):
        ax.spines[side].set_visible(False)
    for side in ('left', 'bottom'):
        ax.spines[side].set_color(BASELINE)
    ax.tick_params(colors=MUTED, labelsize=8)
    ax.set_title(title, color=INK, fontsize=10, loc='left', fontweight='bold')
    ax.set_ylabel(ylabel, color=MUTED, fontsize=8)
    if xlabel:
        ax.set_xlabel(xlabel, color=MUTED, fontsize=8)


def legend(ax):
    ax.legend(loc='upper right', frameon=False, fontsize=8, labelcolor=INK)


# ── the analysis ─────────────────────────────────────────────────────────────
def analyze(d, args, out_png):
    tbase = min(d['imu_t'][0], d['js_t'][0], d['cmd_t'][0])
    for key in ('imu_t', 'js_t', 'cmd_t'):
        d[key] = d[key] - tbase       # keep polyfit/plots well conditioned

    span = [max(d['imu_t'][0], d['js_t'][0], d['cmd_t'][0]),
            min(d['imu_t'][-1], d['js_t'][-1], d['cmd_t'][-1])]
    if span[1] - span[0] < 30.0:
        sys.exit(f'only {span[1] - span[0]:.0f} s of overlapping data — record ~90 s')
    tg = np.arange(span[0], span[1], DT)

    # v_f replayed exactly as balance_controller computes it (its own
    # timestamps, its own tau) so panel B compares against what the law SAW.
    v_f = np.empty_like(d['jv'])
    v_f[0] = d['jv'][0]
    for i in range(1, len(v_f)):
        dtl = min(d['js_t'][i] - d['js_t'][i - 1], 0.1)
        a = dtl / (args.v_filter_tau + dtl)
        v_f[i] = v_f[i - 1] + a * (d['jv'][i] - v_f[i - 1])

    X = np.interp(tg, d['js_t'], d['jx'])
    V = np.interp(tg, d['js_t'], d['jv'])
    VF = np.interp(tg, d['js_t'], v_f)
    PITCH = np.interp(tg, d['imu_t'], d['pitch'])
    GY = np.interp(tg, d['imu_t'], d['gyro_y'])
    TAU = np.interp(tg, d['cmd_t'], d['tau_c'])

    m = (tg > tg[0] + 3.0) & (tg < tg[-1] - 3.0)    # trim filter warm-up edges

    # ── rock characterisation ──
    xb = bandpass(X - X.mean())
    win = np.hanning(len(xb))
    spec = np.abs(np.fft.rfft(xb * win))
    freqs = np.fft.rfftfreq(len(xb), DT)
    sel = (freqs > ROCK_LO) & (freqs < 0.5)
    f0 = freqs[sel][np.argmax(spec[sel])]
    period = 1.0 / f0
    amp_x = np.percentile(np.abs(xb[m]), 95)
    amp_pitch = np.percentile(np.abs(bandpass(PITCH - PITCH.mean())[m]), 95)

    # ── A: dead zone ──
    tau2 = lowpass(TAU, 2.0)
    v2 = lowpass(V, 2.0)
    acc = lowpass(np.gradient(v2, DT), 2.0)
    dwell = np.abs(v2[m]) < DWELL_V
    dwell_frac = float(np.mean(dwell))
    amp_v = np.percentile(np.abs(bandpass(V - V.mean())[m]), 95)
    # what a clean sinusoid of the same speed amplitude would spend below DWELL_V
    dwell_sine = (2.0 / math.pi) * math.asin(min(1.0, DWELL_V / max(amp_v, 1e-9)))

    def slope(lo, hi):
        sel_s = m & (np.abs(tau2) >= lo) & (np.abs(tau2) < hi)
        if np.count_nonzero(sel_s) < 200:
            return None
        return float(np.polyfit(tau2[sel_s], acc[sel_s], 1)[0])
    s_in, s_out = slope(0.0, 0.25), slope(0.25, 0.8)

    # ── B: does pitch track its reconstructed target? ──
    x_home = X[m].mean()      # valid for a symmetric rock with enable released
    TARGET = args.a1 * (X - x_home) + args.a2 * VF + args.trim
    pb, qb = bandpass(PITCH - PITCH.mean())[m], bandpass(TARGET - TARGET.mean())[m]
    r_track, lag_track = norm_xcorr(pb, qb)
    rms_err = float(np.sqrt(np.mean((pb - qb) ** 2)))
    rms_tgt = float(np.sqrt(np.mean(qb ** 2)))

    # ── C: what did fusion add beyond the gyro? ──
    gy_int = np.concatenate(([0.0], np.cumsum(0.5 * (GY[1:] + GY[:-1]) * DT)))
    resid = PITCH - (PITCH[0] + gy_int)
    resid = resid - np.polyval(np.polyfit(tg, resid, 2), tg)   # gyro bias ramp
    rb = bandpass(resid)[m]
    r_fusion, _ = norm_xcorr(rb, xb[m])
    amp_resid = np.percentile(np.abs(rb), 95)

    # ── figure ──
    fig = plt.figure(figsize=(13, 15), facecolor=SURFACE)
    gs = fig.add_gridspec(4, 2, hspace=0.5, wspace=0.22,
                          height_ratios=[1.0, 1.6, 1.0, 1.0])

    ax = fig.add_subplot(gs[0, :])
    style(ax, f'The rock — period {period:.1f} s, amplitude ±{amp_x:.2f} m '
              f'(pitch ±{amp_pitch * DEG:.2f}°)', 'x (m)')
    ax.plot(tg, X - X.mean(), color=MUTED, lw=0.8, label='x (raw)')
    ax.plot(tg, xb, color=BLUE, lw=1.8, label='x (rock band)')
    legend(ax)

    sub = gs[1, 0].subgridspec(2, 1, hspace=0.12)
    ax_t = fig.add_subplot(sub[0])
    style(ax_t, 'A: commanded torque vs wheel motion', 'torque (Nm)')
    ax_t.plot(tg, tau2, color=BLUE, lw=1.4, label='common torque (2 Hz LP)')
    plt.setp(ax_t.get_xticklabels(), visible=False)
    legend(ax_t)
    ax_v = fig.add_subplot(sub[1], sharex=ax_t)
    style(ax_v, '', 'v (m/s)', 't (s)')
    ax_v.plot(tg, v2, color=AQUA, lw=1.4, label='wheel v (2 Hz LP)')
    ax_v.axhspan(-DWELL_V, DWELL_V, color=GRID_C, alpha=0.6, lw=0)
    dwell_full = np.abs(v2) < DWELL_V
    ax_v.fill_between(tg, *ax_v.get_ylim(), where=dwell_full,
                      color=ORANGE, alpha=0.15, lw=0, label='dwell (|v|<3 mm/s)')
    legend(ax_v)

    ax = fig.add_subplot(gs[1, 1])
    style(ax, 'A: cross-plot — flat middle = dead zone',
          'wheel accel (m/s²)', 'common torque (Nm)')
    ax.scatter(tau2[m], acc[m], s=3, color=BLUE, alpha=0.12, edgecolors='none')
    edges = np.linspace(np.percentile(tau2[m], 1), np.percentile(tau2[m], 99), 22)
    mids, meds = [], []
    for lo, hi in zip(edges[:-1], edges[1:]):
        pick = m & (tau2 >= lo) & (tau2 < hi)
        if np.count_nonzero(pick) >= 20:
            mids.append((lo + hi) / 2.0)
            meds.append(float(np.median(acc[pick])))
    ax.plot(mids, meds, color=ORANGE, lw=2.2, label='binned median')
    legend(ax)

    ax = fig.add_subplot(gs[2, :])
    style(ax, f'B: pitch vs reconstructed target — corr {r_track:+.2f}, '
              f'pitch lags {lag_track * 1e3:+.0f} ms, '
              f'tracking error {rms_err / max(rms_tgt, 1e-9) * 100:.0f}% of target',
          'degrees', 't (s)')
    ax.plot(tg[m], qb * DEG, color=ORANGE, lw=1.8, label='pitch_target (reconstructed)')
    ax.plot(tg[m], pb * DEG, color=BLUE, lw=1.8, label='pitch (measured)')
    legend(ax)

    ax = fig.add_subplot(gs[3, :])
    style(ax, f'C: fusion residual vs x — corr {r_fusion:+.2f}, '
              f'residual ±{amp_resid * DEG:.2f}°', 'normalized (σ)', 't (s)')
    for sig, color, name in ((rb, BLUE, 'fused pitch − ∫gyro (residual)'),
                             (xb[m], ORANGE, 'x')):
        sd = sig.std()
        ax.plot(tg[m], sig / sd if sd > 0 else sig, color=color, lw=1.8, label=name)
    legend(ax)

    fig.suptitle(f'{out_png.stem}  —  a1 {args.a1}  a2 {args.a2}  trim {args.trim}',
                 color=MUTED, fontsize=9, y=0.995)
    fig.savefig(out_png, dpi=130, facecolor=SURFACE, bbox_inches='tight')
    plt.close(fig)

    # ── report ──
    rate = lambda t: len(t) / (t[-1] - t[0])
    print(f'\n═══ {out_png.stem} ═══')
    print(f'rates: imu {rate(d["imu_t"]):.0f} Hz, joint_states {rate(d["js_t"]):.0f} Hz, '
          f'commands {rate(d["cmd_t"]):.0f} Hz')
    if rate(d['imu_t']) < 80:
        print('  ⚠ imu well under 100 Hz — if you recorded over wifi, re-record ON THE PI')
    print(f'rock: period {period:.1f} s, ±{amp_x:.2f} m, pitch ±{amp_pitch * DEG:.2f}°')

    print(f'\nA  dead zone: wheel dwell {dwell_frac * 100:.1f}% of the time '
          f'(a clean sinusoid of this size would dwell {dwell_sine * 100:.1f}%)')
    if s_in is not None and s_out is not None:
        print(f'   accel-per-torque slope: {s_in:.2f} inside ±0.25 Nm vs {s_out:.2f} '
              f'outside → ratio {s_in / s_out:.2f}' if s_out else '')
        print('   → ratio well under 1 AND dwell far above the sinusoid figure = dead zone')
    else:
        print('   not enough torque excursion to fit both slopes — read the cross-plot by eye')

    print(f'\nB  tracking: corr {r_track:+.2f}, pitch lags target {lag_track * 1e3:+.0f} ms, '
          f'error {rms_err / max(rms_tgt, 1e-9) * 100:.0f}% of target rms')
    print('   → high corr + small error: plant obeys, the orbit is being COMMANDED')
    print('   → low corr or error ≈ 100%: the plant is not delivering sub-degree tracking (see A)')

    print(f'\nC  fusion: corr(residual, x) {r_fusion:+.2f}, residual ±{amp_resid * DEG:.2f}°')
    print('   → |corr| > 0.6 with ≥ 0.1°: fusion injects position-correlated pitch error')
    print(f'   → residual ≈ 2× pitch amplitude ({2 * amp_pitch * DEG:.2f}°): '
          'gyro and quaternion frames disagree — check mount_rpy handling')

    if d['cv_n']:
        print(f'\ncmd_vel: {d["cv_n"]} msgs, {d["cv_nonzero"]} nonzero, '
              f'max |linear.x| {d["cv_max_vx"]:.3f} — nonzero traffic invalidates '
              'panel B\'s v_cmd=0 assumption AND is a suspect in its own right')
    else:
        print('\ncmd_vel: silent — stick drift exonerated, v_cmd=0 assumption holds')

    print(f'\nwrote {out_png}')
    return {'period': period, 'amp_x': amp_x, 'r_fusion': r_fusion,
            'r_track': r_track, 'dwell_frac': dwell_frac}


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('bag', nargs='?', help='rosbag2 directory')
    ap.add_argument('--selftest', action='store_true',
                    help='run on synthetic data, verify the pipeline, no ROS needed')
    # MUST match what the controller was running when the bag was recorded.
    ap.add_argument('--a1', type=float, default=-0.07)
    ap.add_argument('--a2', type=float, default=-0.18)
    ap.add_argument('--trim', type=float, default=0.06)
    ap.add_argument('--v-filter-tau', type=float, default=0.06)
    ap.add_argument('--wheel-radius', type=float, default=0.105)
    args = ap.parse_args()

    if args.selftest:
        res = analyze(synth(args), args, Path('selftest_analysis.png'))
        ok = (7.8 < res['period'] < 9.8 and 0.11 < res['amp_x'] < 0.15
              and res['r_fusion'] > 0.5 and res['r_track'] > 0.8)
        print(f'\nSELFTEST {"PASS" if ok else "FAIL"} '
              f'(period {res["period"]:.1f} s, amp {res["amp_x"]:.2f} m, '
              f'fusion corr {res["r_fusion"]:+.2f}, tracking corr {res["r_track"]:+.2f})')
        sys.exit(0 if ok else 1)

    if not args.bag:
        ap.error('give a bag directory, or --selftest')
    bag = Path(args.bag).resolve()
    analyze(read_bag(bag, args.wheel_radius), args,
            bag.parent / f'{bag.name}_analysis.png')


if __name__ == '__main__':
    main()
