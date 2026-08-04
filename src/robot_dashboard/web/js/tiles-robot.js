// ---------------------------------------------------------------------------
// The robot-specific tiles.
//
// A tile earns a place here only when a generic one would show the wrong thing:
// a quaternion is not a pitch angle, four separate bus currents are not a
// battery, and a slider labelled "a2" needs the formula next to it or it is
// just a mystery number.
// ---------------------------------------------------------------------------

import * as R from './ros.js';
import { register } from './tiles-core.js';
import { TimePlot, Meter, el, fmt, seriesColor } from './widgets.js';
import { PRESETS, KnobPanel, previewPreset, applyPreset, dirty, tuning, clearDirty } from './tuning.js';

const TELEM = 'robot_interfaces/msg/MotorTelemetry';

// Colour follows the ENTITY. right_wheel is blue whether or not the hips are
// on the bus, so a motor that drops off the CAN bus never repaints the others.
const AXIS_SLOT = { right_wheel: 0, right_hip: 1, left_wheel: 2, left_hip: 3 };
const slotFor = (name, i) => (name in AXIS_SLOT ? AXIS_SLOT[name] : i);

const AXIS_STATE = {
  0: 'offline', 1: 'idle', 2: 'startup', 3: 'calibrating', 4: 'motor cal',
  6: 'index search', 7: 'offset cal', 8: 'closed loop',
};

// ── mode ───────────────────────────────────────────────────────────────────
const MODE_STYLE = {
  disabled: { glyph: '■', cls: 'g-muted' },
  teleop: { glyph: '●', cls: 'g-good' },
  autonomous: { glyph: '◆', cls: 'g-warning' },
};

register({
  type: 'mode',
  title: 'Mode',
  group: 'Robot',
  desc: 'Current mode and the buttons to change it.',
  w: 4, h: 4,
  config: [],
  create() {
    const glyph = el('span', { class: 'glyph g-muted', style: { fontSize: '1.1rem' }, text: '■' });
    const name = el('span', { style: { fontSize: '1.4rem', fontWeight: '600' }, text: '—' });
    const result = el('p', { class: 'note', text: '—' });
    const buttons = ['disabled', 'teleop', 'autonomous'].map((m) => el('button', {
      text: m[0].toUpperCase() + m.slice(1),
      'aria-pressed': 'false',
      onclick: async () => {
        result.textContent = `requesting ${m}…`;
        result.className = 'note';
        try {
          const res = await R.callService('/set_mode', 'robot_interfaces/srv/SetMode', { mode: m });
          result.textContent = `${res.success ? '✓' : '✕ rejected —'} ${res.message}`;
          result.className = `note ${res.success ? 'ok' : 'bad'}`;
        } catch (err) {
          result.textContent = `✕ call failed: ${err.message}`;
          result.className = 'note bad';
        }
      },
    }));

    const stop = R.subscribe('/mode', 'std_msgs/msg/String', (msg) => {
      const style = MODE_STYLE[msg.data] || { glyph: '?', cls: 'g-muted' };
      glyph.textContent = style.glyph;
      glyph.className = `glyph ${style.cls}`;
      name.textContent = msg.data;
      // Reflect the ROBOT's state, never the click. A rejected request must
      // never look accepted.
      buttons.forEach((b) => b.setAttribute('aria-pressed',
        String(b.textContent.toLowerCase() === msg.data)));
    });

    return {
      el: el('div', {}, [
        el('div', { style: { display: 'flex', alignItems: 'center', gap: '.5rem', marginBottom: '.6rem' } },
          [glyph, name]),
        el('div', { class: 'btn-row' }, buttons),
        result,
      ]),
      title: 'Mode',
      destroy: stop,
    };
  },
});

// ── power ──────────────────────────────────────────────────────────────────
const CHEM = {
  lipo: { full: 4.20, empty: 3.30, label: 'LiPo' },
  liion: { full: 4.15, empty: 3.10, label: 'Li-ion' },
  lifepo4: { full: 3.55, empty: 2.80, label: 'LiFePO₄' },
};

register({
  type: 'power',
  title: 'Power',
  group: 'Robot',
  desc: 'Pack voltage, total current and watts, straight off the ODrives. No extra sensor involved.',
  w: 4, h: 5,
  config: [
    { key: 'cells', label: 'Cells in series', type: 'number', def: 6, min: 1, max: 16, step: 1 },
    { key: 'chem', label: 'Chemistry', type: 'select', def: 'lipo',
      options: [
        { value: 'lipo', label: 'LiPo (4.20–3.30 V/cell)' },
        { value: 'liion', label: 'Li-ion (4.15–3.10 V/cell)' },
        { value: 'lifepo4', label: 'LiFePO₄ (3.55–2.80 V/cell)' },
        { value: 'none', label: 'Bench supply — no charge bar' },
      ] },
    { key: 'source', label: 'Source label', type: 'text', def: '6S LiPo' },
  ],
  create(ctx) {
    const cells = ctx.cfg.cells || 6;
    const chem = CHEM[ctx.cfg.chem || 'lipo'];

    const volts = el('span', { class: 'hero dash', text: '—' });
    const meter = new Meter();
    const perCell = el('p', { class: 'note', text: '' });
    const amps = el('span', { class: 'value dash', text: '—' });
    const watts = el('span', { class: 'value dash', text: '—' });
    const source = el('span', { class: 'value', style: { fontSize: '.95rem' }, text: ctx.cfg.source || '—' });
    const reporting = el('p', { class: 'note', text: 'waiting for /motor_telemetry…' });

    const root = el('div', {}, [
      el('div', {}, [volts, el('span', { class: 'hero-unit', text: 'V' })]),
      chem ? meter.el : null,
      chem ? perCell : null,
      el('div', { class: 'stat-row', style: { marginTop: '.5rem' } }, [
        el('div', { class: 'stat' }, [el('div', { class: 'label', text: 'Current' }),
          el('div', {}, [amps, el('span', { class: 'unit', text: 'A' })])]),
        el('div', { class: 'stat' }, [el('div', { class: 'label', text: 'Power' }),
          el('div', {}, [watts, el('span', { class: 'unit', text: 'W' })])]),
        el('div', { class: 'stat' }, [el('div', { class: 'label', text: 'Source' }), source]),
      ]),
      reporting,
    ]);

    const stop = R.subscribe('/motor_telemetry', TELEM, (msg) => {
      if (!msg.axes_reporting) {
        volts.textContent = '—'; volts.classList.add('dash');
        amps.textContent = '—'; watts.textContent = '—';
        reporting.textContent = 'no axis is answering — check CAN and power';
        reporting.className = 'note bad';
        return;
      }
      volts.textContent = fmt(msg.pack_voltage, 1);
      volts.classList.remove('dash');
      amps.textContent = fmt(msg.pack_current, 2); amps.classList.remove('dash');
      watts.textContent = fmt(msg.pack_power, 1); watts.classList.remove('dash');
      reporting.textContent = `${msg.axes_reporting} of ${msg.name.length} axes reporting`;
      reporting.className = 'note';

      if (chem) {
        const v = msg.pack_voltage / cells;
        const frac = (v - chem.empty) / (chem.full - chem.empty);
        // Thresholds in volts PER CELL, which is the only figure that transfers
        // between packs. 3.5 V/cell is the usual "land it now" line for LiPo.
        const status = v < chem.empty + 0.1 ? 'critical' : (v < chem.empty + 0.3 ? 'warning' : 'good');
        meter.set(frac, status);
        const word = status === 'good' ? 'ok' : (status === 'warning' ? 'getting low' : 'LAND IT');
        // Voltage sags under load, so this is a hint and is labelled as one —
        // calling it a state of charge would be a lie with a decimal point.
        perCell.textContent = `${fmt(v, 2)} V/cell · ${chem.label} ${cells}S · ${word} (sags under load)`;
        perCell.className = `note${status === 'good' ? '' : ' bad'}`;
      }
    });

    return { el: root, title: 'Power', destroy: stop };
  },
});

// ── per-motor currents ─────────────────────────────────────────────────────
register({
  type: 'motors',
  title: 'Motors',
  group: 'Robot',
  desc: 'Every ODrive axis: phase current, output torque, state and error code.',
  w: 6, h: 5,
  config: [
    { key: 'view', label: 'Show', type: 'select', def: 'both',
      options: [{ value: 'both', label: 'Table + bars' }, { value: 'table', label: 'Table only' }, { value: 'bars', label: 'Bars only' }] },
    { key: 'full', label: 'Bar full scale (A)', type: 'number', def: 12, min: 1, max: 60, step: 1 },
  ],
  create(ctx) {
    const view = ctx.cfg.view || 'both';
    const bars = el('div', { style: { marginBottom: '.5rem' } });
    const body = el('tbody');
    const table = el('table', { class: 'data' }, [
      el('thead', {}, [el('tr', {}, [
        el('th', { text: 'axis' }), el('th', { text: 'state' }),
        el('th', { text: 'Iq A' }), el('th', { text: 'Nm' }),
        el('th', { text: 'bus V' }), el('th', { text: 'bus A' }),
      ])]),
      body,
    ]);
    const note = el('p', { class: 'note', text: 'waiting for /motor_telemetry…' });
    const root = el('div', {}, [
      view !== 'table' ? bars : null,
      view !== 'bars' ? table : null,
      note,
    ]);
    const full = ctx.cfg.full || 12;

    const stop = R.subscribe('/motor_telemetry', TELEM, (msg) => {
      const faults = [];
      if (view !== 'table') bars.textContent = '';
      if (view !== 'bars') body.textContent = '';

      msg.name.forEach((name, i) => {
        const online = msg.online[i];
        const iq = msg.iq_measured[i];
        const state = AXIS_STATE[msg.axis_state[i]] ?? `state ${msg.axis_state[i]}`;
        if (msg.axis_error[i]) faults.push(`${name}: error 0x${msg.axis_error[i].toString(16)}`);

        if (view !== 'bars') {
          body.appendChild(el('tr', {}, [
            el('td', {}, [
              el('span', { class: 'swatch', style: { background: seriesColor(slotFor(name, i)), display: 'inline-block', marginRight: '.35rem' } }),
              el('span', { text: name }),
            ]),
            el('td', {
              style: { color: online ? (msg.axis_error[i] ? 'var(--critical)' : 'var(--ink)') : 'var(--muted)' },
              text: online ? state : 'offline',
            }),
            el('td', { text: online ? fmt(iq, 2) : '—' }),
            el('td', { text: online ? fmt(msg.torque_est[i], 2) : '—' }),
            el('td', { text: online ? fmt(msg.bus_voltage[i], 1) : '—' }),
            el('td', { text: online ? fmt(msg.bus_current[i], 2) : '—' }),
          ]));
        }

        if (view !== 'table') {
          bars.appendChild(el('div', { style: { marginBottom: '.3rem' } }, [
            el('div', { style: { display: 'flex', justifyContent: 'space-between', fontSize: '.72rem' } }, [
              el('span', { text: name }),
              el('span', { style: { fontVariantNumeric: 'tabular-nums' }, text: online ? `${fmt(iq, 2)} A` : 'offline' }),
            ]),
            el('div', { style: { height: '6px', background: 'var(--grid)', borderRadius: '3px', overflow: 'hidden' } }, [
              el('div', {
                style: {
                  height: '100%', borderRadius: '3px',
                  width: `${Math.min(100, (Math.abs(iq) / full) * 100)}%`,
                  background: seriesColor(slotFor(name, i)),
                },
              }),
            ]),
          ]));
        }
      });

      if (faults.length) {
        note.textContent = `✕ ${faults.join(' · ')}`;
        note.className = 'note bad';
      } else {
        note.textContent = `${msg.axes_reporting} axis(es) reporting · bars full scale ${full} A`;
        note.className = 'note';
      }
    });

    return { el: root, title: 'Motors', destroy: stop };
  },
});

// ── motor current over time ────────────────────────────────────────────────
register({
  type: 'motorplot',
  title: 'Motor current plot',
  group: 'Robot',
  desc: 'Phase current per axis against time — where the torque actually goes.',
  w: 6, h: 5,
  config: [
    { key: 'span', label: 'Seconds shown', type: 'number', def: 20, min: 5, max: 300, step: 5 },
    { key: 'max', label: 'Y max (A)', type: 'number', def: 6, min: 1, max: 60, step: 1 },
  ],
  create(ctx) {
    const names = ['right_wheel', 'right_hip', 'left_wheel', 'left_hip'];
    const plot = new TimePlot({
      names, slots: names.map((n) => AXIS_SLOT[n]),
      span: ctx.cfg.span || 20,
      min: -(ctx.cfg.max || 6), max: ctx.cfg.max || 6,
    });
    plot.el.style.flex = '1 1 auto';
    const legend = el('div', { class: 'legend' }, names.map((n) => el('span', {}, [
      el('span', { class: 'swatch', style: { background: seriesColor(AXIS_SLOT[n]) } }),
      el('span', { text: n }),
    ])));
    const root = el('div', { style: { display: 'flex', flexDirection: 'column', height: '100%' } },
      [plot.el, legend]);

    const stop = R.subscribe('/motor_telemetry', TELEM, (msg) => {
      plot.push(names.map((n) => {
        const i = msg.name.indexOf(n);
        return i === -1 || !msg.online[i] ? NaN : msg.iq_measured[i];
      }));
    });
    return { el: root, title: 'Motor current', resized: () => plot.resized(), destroy: stop };
  },
});

// ── pitch ──────────────────────────────────────────────────────────────────
register({
  type: 'pitch',
  title: 'Pitch',
  group: 'Robot',
  desc: 'Chassis pitch from the IMU quaternion, with the last 15 s.',
  w: 5, h: 5,
  config: [
    { key: 'degrees', label: 'Show degrees', type: 'bool', def: false },
    { key: 'scale', label: 'Full scale (rad)', type: 'number', def: 0.4, min: 0.05, max: 1.5, step: 0.05 },
  ],
  create(ctx) {
    const deg = ctx.cfg.degrees;
    const scale = ctx.cfg.scale || 0.4;
    const value = el('span', { class: 'hero dash', text: '—' });
    const rate = el('span', { class: 'value dash', text: '—' });
    const plot = new TimePlot({ names: ['pitch'], span: 15, min: -scale, max: scale });
    plot.el.style.flex = '1 1 auto';

    const root = el('div', { style: { display: 'flex', flexDirection: 'column', height: '100%' } }, [
      el('div', {}, [value, el('span', { class: 'hero-unit', text: deg ? '°' : 'rad' })]),
      el('div', { class: 'note' }, [el('span', { text: 'rate ' }), rate,
        el('span', { class: 'unit', text: deg ? '°/s' : 'rad/s' })]),
      plot.el,
    ]);

    // The SAME math balance_controller runs, ported to JS. If this disagrees
    // with the robot, one of the two is wrong and it matters which.
    const stop = R.subscribe('/imu', 'sensor_msgs/msg/Imu', (msg) => {
      const q = msg.orientation;
      let sinp = 2.0 * (q.w * q.y - q.z * q.x);
      sinp = Math.max(-1, Math.min(1, sinp));
      const pitch = Math.asin(sinp);
      const k = deg ? 180 / Math.PI : 1;
      value.textContent = fmt(pitch * k, deg ? 1 : 3);
      value.classList.remove('dash');
      rate.textContent = fmt(msg.angular_velocity.y * k, 2);
      rate.classList.remove('dash');
      plot.push([pitch]);
    });

    return { el: root, title: 'Pitch', resized: () => plot.resized(), destroy: stop };
  },
});

// ── commanded vs actual speed ──────────────────────────────────────────────
register({
  type: 'drivestate',
  title: 'Speed tracking',
  group: 'Robot',
  desc: 'Commanded speed against measured wheel speed. The picture that tells you whether the robot is lazy or just slow.',
  w: 6, h: 5,
  config: [
    { key: 'radius', label: 'Wheel radius (m)', type: 'number', def: 0.105, step: 0.001 },
    { key: 'span', label: 'Seconds shown', type: 'number', def: 20, min: 5, max: 120, step: 5 },
    { key: 'max', label: 'Y max (m/s)', type: 'number', def: 1.2, min: 0.2, max: 4, step: 0.1 },
  ],
  create(ctx) {
    const radius = ctx.cfg.radius || 0.105;
    const max = ctx.cfg.max || 1.2;
    const names = ['commanded', 'measured'];
    const plot = new TimePlot({ names, span: ctx.cfg.span || 20, min: -max, max });
    plot.el.style.flex = '1 1 auto';
    const legend = el('div', { class: 'legend' }, names.map((n, i) => el('span', {}, [
      el('span', { class: 'swatch', style: { background: seriesColor(i) } }), el('span', { text: n }),
    ])));
    const lag = el('p', { class: 'note', text: '' });
    const root = el('div', { style: { display: 'flex', flexDirection: 'column', height: '100%' } },
      [plot.el, legend, lag]);

    let cmd = 0, meas = 0;
    const stopCmd = R.subscribe('/cmd_vel', 'geometry_msgs/msg/Twist', (m) => { cmd = m.linear.x; });
    const stopJs = R.subscribe('/joint_states', 'sensor_msgs/msg/JointState', (m) => {
      const l = m.name.indexOf('left_wheel_joint');
      const r = m.name.indexOf('right_wheel_joint');
      if (l === -1 || r === -1) return;
      meas = radius * (m.velocity[l] + m.velocity[r]) / 2;
      plot.push([cmd, meas]);
      lag.textContent = `asked ${fmt(cmd, 2)} · doing ${fmt(meas, 2)} · shortfall ${fmt(cmd - meas, 2)} m/s`;
    });

    return {
      el: root, title: 'Speed tracking', resized: () => plot.resized(),
      destroy: () => { stopCmd(); stopJs(); },
    };
  },
});

// ── drive feel: presets + the knobs that matter ────────────────────────────
register({
  type: 'drive',
  title: 'Drive feel',
  group: 'Tuning',
  desc: 'Calm / Normal / Sport presets, then the individual knobs. Start here for "not powerful, not fast, not responsive".',
  w: 6, h: 9,
  config: [],
  create() {
    const note = el('p', { class: 'note', text: 'presets change several coupled values at once' });
    const buttons = el('div', { class: 'btn-row' }, Object.entries(PRESETS).map(([key, preset]) =>
      el('button', {
        text: preset.label, title: preset.desc,
        onclick: async () => {
          const rows = await previewPreset(key);
          const lines = rows.map((r) => r.missing
            ? `  ${r.node}.${r.name}: NODE NOT RUNNING — skipped`
            : `  ${r.node}.${r.name}: ${fmt(r.from, 3)} → ${fmt(r.to, 3)}`);
          // An informed confirmation, not a yes/no. This changes max_torque on
          // a robot that may be standing up right now.
          if (!window.confirm(`Apply "${preset.label}"?\n\n${preset.desc}\n\n${lines.join('\n')}`)) return;
          const failures = await applyPreset(rows);
          note.textContent = failures.length ? `✕ ${failures.join('; ')}` : `✓ ${preset.label} applied — unsaved until you Save to YAML`;
          note.className = `note ${failures.length ? 'bad' : 'ok'}`;
          balancePanel.load();
          teleopPanel.load();
        },
      })));

    const balancePanel = new KnobPanel('balance_controller',
      ['accel_limit', 'jerk_tau', 'max_torque', 'a2', 'a1', 'yaw_accel_limit', 'k_yaw']);
    const teleopPanel = new KnobPanel('teleop_twist_joy_node',
      ['scale_linear.x', 'scale_linear_turbo.x', 'scale_angular.yaw']);

    const root = el('div', {}, [
      buttons, note,
      el('h3', { style: { fontSize: '.68rem', textTransform: 'uppercase', letterSpacing: '.09em', color: 'var(--muted)', margin: '.9rem 0 .4rem' }, text: 'Stick → speed' }),
      teleopPanel.el,
      el('h3', { style: { fontSize: '.68rem', textTransform: 'uppercase', letterSpacing: '.09em', color: 'var(--muted)', margin: '.9rem 0 .4rem' }, text: 'How that speed is delivered' }),
      balancePanel.el,
    ]);
    return { el: root, title: 'Drive feel' };
  },
});

// ── balance / friction knob panels ─────────────────────────────────────────
function knobTile(type, title, desc, node, names, h) {
  register({
    type, title, desc, group: 'Tuning', w: 5, h,
    config: [],
    create() {
      const panel = new KnobPanel(node, names);
      const refresh = el('button', {
        text: 'Re-read from robot', style: { marginTop: '.4rem' },
        onclick: () => panel.load(),
      });
      return { el: el('div', {}, [panel.el, refresh]), title };
    },
  });
}

knobTile('balance', 'Balance tuning',
  'The inner pitch loop and the outer position loop.',
  'balance_controller',
  ['k3', 'k4', 'a1', 'a2', 'pitch_trim', 'max_lean', 'max_pos_error', 'v_filter_tau', 'accel_to_lean'], 10);

knobTile('friction', 'Friction compensation',
  'Dither for standstill, Coulomb feedforward for driving. Two regimes, not two options.',
  'odrive_bridge',
  ['dither_torque', 'dither_hz', 'friction_ff', 'friction_v_eps'], 8);

// ── save tuning to YAML ────────────────────────────────────────────────────
register({
  type: 'save',
  title: 'Save tuning',
  group: 'Deploy',
  desc: 'Write the values the robot is running right now back into real.yaml, so a reboot does not lose them.',
  w: 5, h: 6,
  config: [],
  create() {
    const count = el('p', { class: 'note', text: 'nothing changed yet' });
    const out = el('pre', { class: 'log', style: { maxHeight: '10rem' }, text: '' });

    const call = async (preview) => {
      out.textContent = preview ? 'checking…' : 'writing…';
      try {
        const res = await R.callService('/dashboard/save_params',
          'robot_interfaces/srv/SaveParams', { nodes: [], preview }, 30000);
        out.textContent = `${res.success ? '✓' : '✕'} ${res.message}\n` +
          (res.changed.length ? res.changed.join('\n') : '(no differences)');
        if (res.success && !preview) clearDirty();
      } catch (err) {
        out.textContent = `✕ ${err.message}\n\nIs dashboard_backend running? It is what owns the filesystem side.`;
      }
    };

    const update = () => {
      count.textContent = dirty.size
        ? `${dirty.size} live value(s) not yet in the YAML: ${[...dirty].join(', ')}`
        : 'no live changes pending';
      count.className = `note${dirty.size ? ' bad' : ''}`;
    };
    tuning.addEventListener('dirty', update);
    update();

    return {
      el: el('div', {}, [
        count,
        el('div', { class: 'btn-row', style: { marginTop: '.4rem' } }, [
          el('button', { text: 'Preview diff', onclick: () => call(true) }),
          el('button', { class: 'primary', text: 'Save to YAML', onclick: () => call(false) }),
        ]),
        el('p', { class: 'note', text: 'Reads the LIVE values off the running nodes, keeps every comment, and leaves a .bak beside each file.' }),
        out,
      ]),
      title: 'Save tuning',
      destroy: () => tuning.removeEventListener('dirty', update),
    };
  },
});

// ── deploy: pull / build / restart ─────────────────────────────────────────
register({
  type: 'deploy',
  title: 'Deploy',
  group: 'Deploy',
  desc: 'Pull from GitHub, rebuild, restart the stack. Needs the Pi on a network with internet — not its own hotspot.',
  w: 7, h: 9,
  config: [],
  create() {
    const log = el('pre', { class: 'log', text: '' });
    const status = el('p', { class: 'note', text: 'waiting for dashboard_backend…' });
    let mode = null;

    const btn = (label, service, opts = {}) => el('button', {
      text: label, title: opts.title || '',
      onclick: async () => {
        if (opts.confirm && !window.confirm(opts.confirm)) return;
        status.textContent = `calling ${service}…`;
        status.className = 'note';
        try {
          const res = await R.callService(`/dashboard/${service}`, 'std_srvs/srv/Trigger', {});
          status.textContent = `${res.success ? '✓' : '✕'} ${res.message}`;
          status.className = `note ${res.success ? 'ok' : 'bad'}`;
        } catch (err) {
          status.textContent = `✕ ${err.message}`;
          status.className = 'note bad';
        }
      },
    });

    const bStatus = btn('1 · Check', 'git_status', { title: 'git fetch, then show what is waiting to come down' });
    const bPull = btn('2 · Pull', 'git_pull', { title: 'git pull --ff-only' });
    const bBuild = btn('3 · Build', 'build',
      { title: 'colcon build --symlink-install', confirm: 'Rebuild the workspace? The robot must be DISABLED.' });
    const bRestart = btn('4 · Restart', 'restart_stack',
      { title: 'systemctl restart biped-stack', confirm: 'Restart the stack? This kills the balance loop and idles the wheel axes.' });

    const gate = el('p', { class: 'note', text: '' });
    const setGate = () => {
      const safe = mode === 'disabled';
      bBuild.disabled = !safe;
      bRestart.disabled = !safe;
      gate.textContent = safe
        ? 'mode is DISABLED — build and restart are unlocked'
        : `build and restart are locked: mode is ${mode ?? 'unknown'}. They drop a balancing robot.`;
      gate.className = `note${safe ? ' ok' : ' bad'}`;
    };
    setGate();

    const append = (line) => {
      const cls = line.startsWith('!!') ? 'err' : (line.startsWith('$') ? 'cmd' : (line.startsWith('--') ? 'end' : ''));
      log.appendChild(el('span', { class: cls, text: `${line}\n` }));
      while (log.childNodes.length > 2000) log.removeChild(log.firstChild);
      log.scrollTop = log.scrollHeight;
    };

    // Prime from the backend's retained tail, so a page reloaded halfway
    // through a build still shows the build.
    R.callService('/dashboard/get_log', 'std_srvs/srv/Trigger', {}, 10000)
      .then((res) => { if (res.message) res.message.split('\n').forEach(append); })
      .catch(() => {});

    const stopLog = R.subscribe('/dashboard/job_log', 'std_msgs/msg/String', (m) => append(m.data));
    const stopStatus = R.subscribe('/dashboard/status', 'std_msgs/msg/String', (m) => {
      let s;
      try { s = JSON.parse(m.data); } catch (err) { return; }
      mode = s.mode;
      setGate();
      const busy = s.job && s.job.running;
      [bStatus, bPull].forEach((b) => { b.disabled = busy; });
      if (busy) { bBuild.disabled = true; bRestart.disabled = true; }
      if (s.job) {
        status.textContent = s.job.running
          ? `${s.job.name} running · ${s.job.elapsed}s`
          : `${s.job.name} exited ${s.job.exit_code} after ${s.job.elapsed}s`;
        status.className = `note ${s.job.running ? '' : (s.job.exit_code === 0 ? 'ok' : 'bad')}`;
      }
    });

    const root = el('div', { style: { display: 'flex', flexDirection: 'column', height: '100%' } }, [
      el('div', { class: 'btn-row' }, [bStatus, bPull, bBuild, bRestart]),
      gate, status,
      el('div', { style: { flex: '1 1 auto', minHeight: '4rem', marginTop: '.4rem' } }, [log]),
    ]);

    return { el: root, title: 'Deploy', destroy: () => { stopLog(); stopStatus(); } };
  },
});

// ── legs ───────────────────────────────────────────────────────────────────
register({
  type: 'legs',
  title: 'Legs',
  group: 'Robot',
  desc: 'Leg position command and feedback. Does nothing unless the stack was launched with legs:=true.',
  w: 5, h: 6,
  config: [
    { key: 'min', label: 'Minimum (turns)', type: 'number', def: 0, step: 0.005 },
    { key: 'max', label: 'Maximum (turns)', type: 'number', def: 0.1, step: 0.005 },
  ],
  create(ctx) {
    // Defaults track leg_controller's pos_min / pos_max in real.yaml. A slider
    // wider than the mechanism drives the legs into their hard stops under
    // position control, which does not give up.
    const LO = ctx.cfg.min ?? 0.0;
    const HI = ctx.cfg.max ?? 0.10;

    const cmdOut = el('span', { class: 'value', text: LO.toFixed(3) });
    const slider = el('input', { type: 'range', min: LO, max: HI, step: 0.005, value: LO, style: { width: '100%' } });
    const left = el('span', { class: 'value dash', text: '—' });
    const right = el('span', { class: 'value dash', text: '—' });
    const sag = el('span', { class: 'value dash', text: '—' });

    let target = LO, lastSend = 0;
    const send = (v, force) => {
      v = Math.max(LO, Math.min(HI, v));
      target = v; slider.value = v; cmdOut.textContent = v.toFixed(3);
      const now = Date.now();
      if (!force && now - lastSend < 50) return;   // a drag fires far faster than the bus needs
      lastSend = now;
      R.publish('/leg_position_cmd', 'std_msgs/msg/Float64', { data: v });
    };
    slider.addEventListener('input', (e) => send(parseFloat(e.target.value)));
    slider.addEventListener('change', (e) => send(parseFloat(e.target.value), true));

    const stop = R.subscribe('/leg_states', 'sensor_msgs/msg/JointState', (msg) => {
      const l = msg.name.indexOf('left_leg_joint');
      const r = msg.name.indexOf('right_leg_joint');
      if (l === -1 || r === -1) return;
      left.textContent = fmt(msg.position[l], 3); left.classList.remove('dash');
      right.textContent = fmt(msg.position[r], 3); right.classList.remove('dash');
      sag.textContent = fmt(Math.max(Math.abs(target - msg.position[l]),
        Math.abs(target - msg.position[r])), 3);
      sag.classList.remove('dash');
    });

    return {
      el: el('div', {}, [
        el('div', { class: 'stat' }, [el('div', { class: 'label', text: 'Commanded' }),
          el('div', {}, [cmdOut, el('span', { class: 'unit', text: 'turns' })])]),
        slider,
        el('div', { class: 'btn-row' }, [
          el('button', { text: 'Retract', onclick: () => send(LO, true) }),
          el('button', { text: 'Mid', onclick: () => send((LO + HI) / 2, true) }),
          el('button', { text: 'Extend', onclick: () => send(HI, true) }),
        ]),
        el('div', { class: 'stat-row', style: { marginTop: '.6rem' } }, [
          el('div', { class: 'stat' }, [el('div', { class: 'label', text: 'Left' }), left]),
          el('div', { class: 'stat' }, [el('div', { class: 'label', text: 'Right' }), right]),
          el('div', { class: 'stat' }, [el('div', { class: 'label', text: 'Error (at rest = sag)' }), sag]),
        ]),
      ]),
      title: 'Legs',
      destroy: stop,
    };
  },
});
