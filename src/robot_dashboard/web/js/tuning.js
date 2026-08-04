// ---------------------------------------------------------------------------
// Live tuning: curated knobs, safe ranges, and the coupled drive-feel presets.
//
// WHY A CURATED TABLE INSTEAD OF JUST LISTING EVERY PARAMETER
// The generic `params` tile already lists everything, as free text. That is the
// right tool for poking at something once. It is the WRONG tool for tuning
// while a robot is balancing, for two reasons:
//
//   1. A slider needs a range, and the honest range for max_torque is not
//      "any float". Typing 400 into a text box is one keystroke away from 40;
//      dragging a slider that stops at 20 is not.
//   2. The knobs that matter are COUPLED. a1 and a2 are two views of one
//      second-order loop, and tuning either alone always disappoints — the
//      hint text under each slider says so, with the formula.
//
// The ranges below are wider than any value you should need and narrower than
// anything that would be dangerous. They are UI limits only: they clamp what
// this page can send, and change nothing about the node.
// ---------------------------------------------------------------------------

import * as R from './ros.js';
import { el, fmt } from './widgets.js';

export const KNOBS = {
  balance_controller: {
    k3: { min: 5, max: 40, step: 0.5,
      hint: 'Inner-loop stiffness, Nm per rad of pitch error. Also sets where the controller saturates: max_torque / k3.' },
    k4: { min: 0, max: 8, step: 0.1,
      hint: 'Inner-loop damping, Nm per rad/s of pitch rate.' },
    a1: { min: -0.2, max: 0, step: 0.005,
      hint: 'Position gain. omega_n = sqrt(g·|a1|) — raise for a tighter station hold, at the cost of a busier robot.' },
    a2: { min: -0.2, max: 0, step: 0.005,
      hint: 'Velocity gain. Driving bandwidth is g·|a2| rad/s and a balancer is non-minimum-phase, capping it near 1.6 — so |a2| above about 0.163 asks for more than the physics allows.' },
    accel_limit: { min: 0.1, max: 2.0, step: 0.05,
      hint: 'How hard the robot is allowed to accelerate, m/s². Costs accel_limit/9.81 rad of lean while accelerating — check that against max_lean.' },
    jerk_tau: { min: 0.05, max: 0.6, step: 0.01,
      hint: 'Throttle softness, seconds. LOWER = crisper and more responsive; higher = lazier. This is the knob that makes driving feel dead.' },
    accel_to_lean: { min: 0, max: 0.2, step: 0.002,
      hint: '1/g = 0.102. Feedforward, not a gain — leave it unless you are testing what it does.' },
    max_pos_error: { min: 0.05, max: 1.5, step: 0.05,
      hint: 'Anti-windup: how far the reference may lead the robot, in metres.' },
    max_torque: { min: 2, max: 20, step: 0.5,
      hint: 'Output-shaft ceiling, Nm, SHARED between balancing and yaw. The measured hardware ceiling is 41.3, so this is a deliberate limit, not a physical one. The single biggest lever on "not powerful".' },
    max_lean: { min: 0.1, max: 0.5, step: 0.01,
      hint: 'How far the outer loop may lean the robot from its trim, rad.' },
    pitch_trim: { min: -0.2, max: 0.2, step: 0.001,
      hint: 'Chassis pitch where the CoM sits over the contact patch. Drifts backward → increase. Re-measure after ANY mechanical work on the legs.' },
    k_yaw: { min: 0, max: 10, step: 0.25,
      hint: 'Differential torque per rad/s of yaw error.' },
    yaw_accel_limit: { min: 0.5, max: 15, step: 0.5,
      hint: 'How fast a turn is allowed to build, rad/s². Reach for this before raising the yaw scale — it changes how abruptly the turn is asked for, not how sharp it ends up.' },
    v_filter_tau: { min: 0.01, max: 0.3, step: 0.01,
      hint: 'Wheel-velocity low-pass, seconds. Raise if standstill rocking worsens; lower if driving feels laggy.' },
    cutoff_pitch: { min: 0.3, max: 1.2, step: 0.05,
      hint: 'Past this pitch the robot is down and torque is cut, rad.' },
  },
  odrive_bridge: {
    friction_ff: { min: 0, max: 1.0, step: 0.01,
      hint: 'Coulomb feedforward, Nm at the wheel. Measured breakaway is ~0.5; stay at or under 70% of it. This is POSITIVE velocity feedback — overshoot makes the robot creep or surge.' },
    friction_v_eps: { min: 0.05, max: 1.0, step: 0.05,
      hint: 'Taper width through zero, rad/s. Only touch this if low-speed driving feels notchy.' },
    dither_torque: { min: 0, max: 0.8, step: 0.01,
      hint: 'Standstill stiction breaker, Nm at the wheel. 0.4 visibly shakes the frame; 0.25 was the sweet spot with feedforward carrying the moving regime.' },
    dither_hz: { min: 5, max: 30, step: 1,
      hint: 'Dither frequency. Do NOT raise toward 50 — at half the ~100 Hz command rate the square wave degenerates into a DC torque bias.' },
  },
  teleop_twist_joy_node: {
    'scale_linear.x': { min: 0.1, max: 2.0, step: 0.05,
      hint: 'Top speed at full stick, m/s. The most direct answer to "not fast".' },
    'scale_linear_turbo.x': { min: 0.1, max: 3.0, step: 0.05,
      hint: 'Top speed with the turbo button held, m/s.' },
    'scale_angular.yaw': { min: 0.1, max: 2.5, step: 0.05,
      hint: 'Top turn rate, rad/s.' },
  },
};

// ---------------------------------------------------------------------------
// DRIVE-FEEL PRESETS
//
// Each is a COUPLED SET, which is the point. Raising top speed alone gives you
// a robot that takes longer to reach a speed it now overshoots; raising
// acceleration alone leans it further for the same lazy throttle. These change
// the whole chain at once — top speed, how hard it may accelerate, how sharply
// that acceleration is allowed to arrive, how much torque there is to do it
// with, and the outer loop that has to stay stable through all of it.
//
// The numbers are not taste. They come from the relationships already written
// into real.yaml:
//   * driving bandwidth  = g·|a2|,  ceiling ≈ 1.6 rad/s  ⇒ |a2| ≤ 0.163
//   * damping            zeta = (|a2|/2)·sqrt(g/|a1|),  kept near 0.7–0.8
//   * cruise lean        = accel_limit / 9.81 rad, kept well inside max_lean
//   * torque ceiling     = 41.3 Nm measured, so 12 Nm is still conservative
//
// NORMAL is exactly what real.yaml ships today, so "put it back" is one click.
// ---------------------------------------------------------------------------
export const PRESETS = {
  calm: {
    label: 'Calm',
    desc: 'Gentler than today. For a first run after mechanical work, or a visitor driving.',
    values: {
      balance_controller: {
        a1: -0.06, a2: -0.10, accel_limit: 0.35, jerk_tau: 0.35,
        max_torque: 8.0, yaw_accel_limit: 4.0,
      },
      teleop_twist_joy_node: {
        'scale_linear.x': 0.4, 'scale_linear_turbo.x': 0.6, 'scale_angular.yaw': 0.6,
      },
    },
  },
  normal: {
    label: 'Normal',
    desc: 'Exactly what real.yaml ships today — the smooth-but-gentle tune from bring-up. Use it to get back to a known state.',
    values: {
      balance_controller: {
        a1: -0.07, a2: -0.12, accel_limit: 0.45, jerk_tau: 0.3,
        max_torque: 8.0, yaw_accel_limit: 5.0,
      },
      teleop_twist_joy_node: {
        'scale_linear.x': 0.5, 'scale_linear_turbo.x': 0.75, 'scale_angular.yaw': 0.75,
      },
    },
  },
  sport: {
    label: 'Sport',
    desc: 'Faster, harder, crisper. Nearly double the top speed, twice the acceleration, half the throttle lag, +50% torque. Drive it on open floor first.',
    values: {
      balance_controller: {
        // 1.47 rad/s of driving bandwidth — just inside the 1.6 ceiling.
        a1: -0.09, a2: -0.15,
        // 0.9 m/s² needs 0.092 rad (5.3°) of lean; max_lean is 0.3.
        accel_limit: 0.9,
        // The bring-up value of 0.15 was raised to 0.3 for softness. That
        // softness IS the lazy throttle.
        jerk_tau: 0.15,
        // +50% authority for balance AND the yaw that shares the budget.
        max_torque: 12.0,
        yaw_accel_limit: 7.0,
      },
      teleop_twist_joy_node: {
        'scale_linear.x': 0.9, 'scale_linear_turbo.x': 1.3, 'scale_angular.yaw': 1.1,
      },
    },
  },
};

// ── dirty tracking ─────────────────────────────────────────────────────────
// "Dirty" = changed live since the page loaded and therefore NOT in the YAML.
// It exists so a good tune cannot be lost to a reboot without warning.
export const tuning = new EventTarget();
export const dirty = new Set();

export function markDirty(node, name) {
  dirty.add(`${node}.${name}`);
  tuning.dispatchEvent(new CustomEvent('dirty'));
}

export function clearDirty() {
  dirty.clear();
  tuning.dispatchEvent(new CustomEvent('dirty'));
}

/**
 * A panel of sliders for one node.
 *
 * Writes are DEBOUNCED. Dragging a slider fires an event per pixel; each one
 * is a service call over a WebSocket to a node inside the balance loop, and
 * un-debounced that is a few hundred set_parameters calls per second aimed at
 * the thing holding the robot up.
 */
export class KnobPanel {
  constructor(node, names, opts = {}) {
    this.node = node;
    this.names = names;
    this.rows = new Map();
    this.note = el('p', { class: 'note', text: 'loading…' });
    this.el = el('div', {});
    this.body = el('div', {});
    this.el.appendChild(this.body);
    if (opts.showNote !== false) this.el.appendChild(this.note);
    this.pending = new Map();
    this.timer = null;
    this.load();
  }

  async load() {
    try {
      const values = await R.getParams(this.node, this.names);
      this.body.textContent = '';
      this.rows.clear();
      for (const name of this.names) {
        const entry = values[name];
        if (!entry || entry.value === null) continue;
        this.body.appendChild(this.row(name, entry));
      }
      this.note.textContent = 'edits apply immediately';
      this.note.className = 'note';
    } catch (err) {
      this.note.textContent = `✕ ${err.message} — is ${this.node} running?`;
      this.note.className = 'note bad';
    }
  }

  row(name, entry) {
    const spec = (KNOBS[this.node] || {})[name] || { min: -10, max: 10, step: 0.01 };
    const valueBox = el('input', { type: 'number', step: spec.step, value: entry.value });
    const slider = el('input', {
      type: 'range', min: spec.min, max: spec.max, step: spec.step, value: entry.value,
    });
    const root = el('div', { class: 'knob' }, [
      el('span', { class: 'k-name', text: name }),
      el('span', { class: 'k-val' }, [valueBox]),
      slider,
      spec.hint ? el('span', { class: 'k-hint', text: spec.hint }) : null,
    ]);

    const push = (raw) => {
      const v = Number(raw);
      if (!Number.isFinite(v)) return;
      slider.value = v; valueBox.value = v;
      root.classList.add('dirty');
      markDirty(this.node, name);
      this.queue(name, entry.type, v);
    };
    slider.addEventListener('input', (e) => push(e.target.value));
    valueBox.addEventListener('change', (e) => push(e.target.value));

    this.rows.set(name, { root, slider, valueBox, entry });
    return root;
  }

  queue(name, type, value) {
    this.pending.set(name, { name, type, value });
    if (this.timer) return;
    this.timer = setTimeout(() => {
      this.timer = null;
      const entries = [...this.pending.values()];
      this.pending.clear();
      this.send(entries);
    }, 80);
  }

  async send(entries) {
    try {
      const results = await R.setParams(this.node, entries);
      const bad = results.filter((r) => !r.successful);
      if (bad.length) {
        // A rejected set_parameters returns success=false and changes nothing.
        // Not surfacing that is how a slider ends up looking like it works.
        this.note.textContent = `✕ ${bad.map((b) => `${b.name}: ${b.reason}`).join('; ')}`;
        this.note.className = 'note bad';
      } else {
        this.note.textContent = `✓ ${entries.map((e) => `${e.name}=${fmt(e.value, 3)}`).join('  ')}`;
        this.note.className = 'note ok';
      }
    } catch (err) {
      this.note.textContent = `✕ ${err.message}`;
      this.note.className = 'note bad';
    }
  }

  /** Update the sliders to match values applied from somewhere else (a preset). */
  reflect(values) {
    for (const [name, v] of Object.entries(values)) {
      const row = this.rows.get(name);
      if (!row) continue;
      row.slider.value = v;
      row.valueBox.value = v;
      row.root.classList.add('dirty');
    }
  }
}

/**
 * Apply a preset. Reads the current values first so the confirmation can show a
 * real before/after — "apply Sport?" is not an informed question, and this
 * changes max_torque on a robot that may be standing up.
 */
export async function previewPreset(key) {
  const preset = PRESETS[key];
  const rows = [];
  for (const [node, values] of Object.entries(preset.values)) {
    const names = Object.keys(values);
    let current = {};
    try { current = await R.getParams(node, names); } catch (err) { /* node down */ }
    for (const name of names) {
      const now = current[name] ? current[name].value : null;
      rows.push({
        node, name, from: now, to: values[name],
        type: current[name] ? current[name].type : R.P_DOUBLE,
        missing: !current[name],
      });
    }
  }
  return rows;
}

export async function applyPreset(rows) {
  const byNode = new Map();
  for (const r of rows) {
    if (r.missing) continue;
    if (!byNode.has(r.node)) byNode.set(r.node, []);
    byNode.get(r.node).push({ name: r.name, type: r.type, value: r.to });
    markDirty(r.node, r.name);
  }
  const failures = [];
  for (const [node, entries] of byNode) {
    try {
      const results = await R.setParams(node, entries);
      for (const res of results) if (!res.successful) failures.push(`${node}.${res.name}: ${res.reason}`);
    } catch (err) {
      failures.push(`${node}: ${err.message}`);
    }
  }
  return failures;
}
