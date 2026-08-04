// ---------------------------------------------------------------------------
// The GENERIC tiles: point them at any topic and any field.
//
// These are what make the dashboard outlive the next feature. Adding a node
// that publishes something new should not require editing the dashboard —
// drop a Plot tile on the page, pick the topic from the list rosapi returns,
// pick the field, done. The robot-specific tiles in tiles-robot.js exist only
// where a raw field would be the wrong thing to show (a quaternion is not a
// pitch angle; four separate currents are not a battery).
// ---------------------------------------------------------------------------

import * as R from './ros.js';
import { TimePlot, Meter, el, fmt, seriesColor, statusChip } from './widgets.js';

export const registry = new Map();
export function register(def) { registry.set(def.type, def); }

// ── value ──────────────────────────────────────────────────────────────────
register({
  type: 'value',
  title: 'Value',
  group: 'Generic',
  desc: 'One number from one topic field, with an optional sparkline.',
  w: 3, h: 3,
  config: [
    { key: 'topic', label: 'Topic', type: 'topic' },
    { key: 'field', label: 'Field', type: 'field' },
    { key: 'unit', label: 'Unit', type: 'text', placeholder: 'm/s' },
    { key: 'digits', label: 'Decimals', type: 'number', min: 0, max: 6, step: 1, def: 2 },
    { key: 'spark', label: 'Show sparkline', type: 'bool', def: true },
  ],
  create(ctx) {
    const { topic, type, field, unit, digits, spark } = ctx.cfg;
    const value = el('span', { class: 'hero dash', text: '—' });
    const head = el('div', {}, [value, unit ? el('span', { class: 'hero-unit', text: unit }) : null]);
    const root = el('div', { style: { display: 'flex', flexDirection: 'column', height: '100%' } }, [head]);

    let plot = null;
    if (spark !== false) {
      plot = new TimePlot({ span: 20, auto: true, names: [field || 'value'] });
      plot.el.style.flex = '1 1 auto';
      plot.el.style.marginTop = '.3rem';
      root.appendChild(plot.el);
    }

    const stop = topic && type ? R.subscribe(topic, type, (msg) => {
      const v = R.pluck(msg, field);
      value.textContent = fmt(v, digits ?? 2);
      value.classList.toggle('dash', v === undefined || v === null);
      if (plot && typeof v === 'number') plot.push([v]);
    }) : null;

    return {
      el: root,
      title: ctx.cfg.title || `${field || '?'} · ${topic || 'no topic'}`,
      resized: () => plot && plot.resized(),
      destroy: () => stop && stop(),
    };
  },
});

// ── plot ───────────────────────────────────────────────────────────────────
register({
  type: 'plot',
  title: 'Plot',
  group: 'Generic',
  desc: 'Up to four numeric fields from one topic against time.',
  w: 6, h: 5,
  config: [
    { key: 'topic', label: 'Topic', type: 'topic' },
    { key: 'fields', label: 'Fields (up to 4)', type: 'fields', max: 4 },
    { key: 'auto', label: 'Auto y-scale', type: 'bool', def: true },
    { key: 'min', label: 'Y min (fixed scale)', type: 'number', def: -1, step: 'any' },
    { key: 'max', label: 'Y max (fixed scale)', type: 'number', def: 1, step: 'any' },
    { key: 'span', label: 'Seconds shown', type: 'number', def: 15, min: 2, max: 300, step: 1 },
  ],
  create(ctx) {
    const { topic, type, auto, min, max, span } = ctx.cfg;
    const fields = (ctx.cfg.fields || []).slice(0, 4);
    const plot = new TimePlot({
      names: fields, span: span || 15, auto: auto !== false,
      min: min ?? -1, max: max ?? 1,
    });
    plot.el.style.flex = '1 1 auto';

    // A legend is present for two or more series, always — identity must never
    // rest on colour alone. One series needs none: the title names it.
    const legend = el('div', { class: 'legend' },
      fields.length > 1 ? fields.map((f, i) => el('span', {}, [
        el('span', { class: 'swatch', style: { background: seriesColor(i) } }),
        el('span', { text: f }),
      ])) : []);

    const root = el('div', { style: { display: 'flex', flexDirection: 'column', height: '100%' } },
      [plot.el, legend]);

    const stop = topic && type ? R.subscribe(topic, type, (msg) => {
      plot.push(fields.map((f) => {
        const v = R.pluck(msg, f);
        return typeof v === 'number' ? v : NaN;
      }));
    }) : null;

    return {
      el: root,
      title: ctx.cfg.title || `${topic || 'no topic'}`,
      resized: () => plot.resized(),
      destroy: () => stop && stop(),
    };
  },
});

// ── gauge ──────────────────────────────────────────────────────────────────
register({
  type: 'gauge',
  title: 'Gauge',
  group: 'Generic',
  desc: 'A value against a range, with warning and critical thresholds.',
  w: 3, h: 3,
  config: [
    { key: 'topic', label: 'Topic', type: 'topic' },
    { key: 'field', label: 'Field', type: 'field' },
    { key: 'unit', label: 'Unit', type: 'text' },
    { key: 'min', label: 'Minimum', type: 'number', def: 0, step: 'any' },
    { key: 'max', label: 'Maximum', type: 'number', def: 100, step: 'any' },
    { key: 'warn', label: 'Warning above', type: 'number', step: 'any' },
    { key: 'crit', label: 'Critical above', type: 'number', step: 'any' },
  ],
  create(ctx) {
    const { topic, type, field, unit, min, max, warn, crit } = ctx.cfg;
    const value = el('span', { class: 'hero dash', text: '—' });
    const meter = new Meter();
    const note = el('p', { class: 'note', text: '' });
    const root = el('div', {}, [
      el('div', {}, [value, unit ? el('span', { class: 'hero-unit', text: unit }) : null]),
      meter.el, note,
    ]);

    const stop = topic && type ? R.subscribe(topic, type, (msg) => {
      const v = R.pluck(msg, field);
      if (typeof v !== 'number') { value.textContent = '—'; value.classList.add('dash'); return; }
      value.textContent = fmt(v, 2);
      value.classList.remove('dash');
      const lo = min ?? 0, hi = max ?? 100;
      let status = 'good';
      if (crit !== undefined && crit !== null && v >= crit) status = 'critical';
      else if (warn !== undefined && warn !== null && v >= warn) status = 'warning';
      meter.set((v - lo) / ((hi - lo) || 1), status);
      // Word, not colour alone.
      note.textContent = status === 'good' ? 'normal' : status;
      note.className = `note${status === 'good' ? '' : ' bad'}`;
    }) : null;

    return {
      el: root,
      title: ctx.cfg.title || `${field || '?'}`,
      destroy: () => stop && stop(),
    };
  },
});

// ── bars over an array field ───────────────────────────────────────────────
register({
  type: 'bars',
  title: 'Array bars',
  group: 'Generic',
  desc: 'One bar per element of an array field — per-motor currents, per-joint effort.',
  w: 4, h: 4,
  config: [
    { key: 'topic', label: 'Topic', type: 'topic' },
    { key: 'field', label: 'Array field', type: 'field' },
    { key: 'labels', label: 'Label field (optional)', type: 'field' },
    { key: 'unit', label: 'Unit', type: 'text' },
    { key: 'max', label: 'Full scale (0 = auto)', type: 'number', def: 0, step: 'any' },
  ],
  create(ctx) {
    const { topic, type, field, labels, unit, max } = ctx.cfg;
    const root = el('div', {});
    const stop = topic && type ? R.subscribe(topic, type, (msg) => {
      const arr = R.pluck(msg, field);
      if (!arr || typeof arr.length !== 'number') { root.textContent = '—'; return; }
      const names = labels ? R.pluck(msg, labels) : null;
      const values = Array.from(arr, Number);
      const scale = max && max > 0 ? max : Math.max(1e-6, ...values.map(Math.abs));
      root.textContent = '';
      values.forEach((v, i) => {
        const name = (names && names[i]) || `[${i}]`;
        root.appendChild(el('div', { style: { marginBottom: '.35rem' } }, [
          el('div', { style: { display: 'flex', justifyContent: 'space-between', fontSize: '.72rem' } }, [
            el('span', { text: String(name) }),
            el('span', { style: { fontVariantNumeric: 'tabular-nums' }, text: `${fmt(v, 2)}${unit ? ' ' + unit : ''}` }),
          ]),
          el('div', { style: { height: '6px', background: 'var(--grid)', borderRadius: '3px', overflow: 'hidden' } }, [
            el('div', {
              style: {
                height: '100%', borderRadius: '3px',
                width: `${Math.min(100, (Math.abs(v) / scale) * 100)}%`,
                background: seriesColor(i),
              },
            }),
          ]),
        ]));
      });
    }) : null;
    return { el: root, title: ctx.cfg.title || field || 'array', destroy: () => stop && stop() };
  },
});

// ── text / state ───────────────────────────────────────────────────────────
register({
  type: 'text',
  title: 'Text',
  group: 'Generic',
  desc: 'A string field shown large — modes, states, names.',
  w: 3, h: 2,
  config: [
    { key: 'topic', label: 'Topic', type: 'topic' },
    { key: 'field', label: 'Field', type: 'field', def: 'data' },
  ],
  create(ctx) {
    const { topic, type, field } = ctx.cfg;
    const value = el('div', { class: 'hero dash', style: { fontSize: '1.5rem' }, text: '—' });
    const stop = topic && type ? R.subscribe(topic, type, (msg) => {
      const v = R.pluck(msg, field || 'data');
      value.textContent = v === undefined ? '—' : String(v);
      value.classList.toggle('dash', v === undefined);
    }) : null;
    return { el: value, title: ctx.cfg.title || topic || 'text', destroy: () => stop && stop() };
  },
});

// ── every field of one message, as a table ─────────────────────────────────
register({
  type: 'fields',
  title: 'All fields',
  group: 'Generic',
  desc: 'Every leaf field of a topic in one table. The fastest way to see what a new topic actually contains.',
  w: 4, h: 5,
  config: [
    { key: 'topic', label: 'Topic', type: 'topic' },
    { key: 'digits', label: 'Decimals', type: 'number', def: 3, min: 0, max: 6, step: 1 },
  ],
  create(ctx) {
    const { topic, type, digits } = ctx.cfg;
    const body = el('tbody');
    const table = el('table', { class: 'data' }, [
      el('thead', {}, [el('tr', {}, [el('th', { text: 'field' }), el('th', { text: 'value' })])]),
      body,
    ]);

    const flatten = (obj, prefix, out) => {
      for (const [k, v] of Object.entries(obj || {})) {
        const path = prefix ? `${prefix}.${k}` : k;
        if (v && typeof v === 'object' && !Array.isArray(v) && typeof v.length !== 'number') {
          flatten(v, path, out);
        } else if (Array.isArray(v) || (v && typeof v.length === 'number' && typeof v !== 'string')) {
          out.push([path, `[${Array.from(v).map((x) => fmt(x, digits ?? 3)).join(', ')}]`]);
        } else {
          out.push([path, fmt(v, digits ?? 3)]);
        }
      }
    };

    const stop = topic && type ? R.subscribe(topic, type, (msg) => {
      const rows = [];
      flatten(msg, '', rows);
      body.textContent = '';
      for (const [k, v] of rows) {
        body.appendChild(el('tr', {}, [el('td', { text: k }), el('td', { text: v })]));
      }
    }) : null;

    return { el: table, title: ctx.cfg.title || topic || 'fields', destroy: () => stop && stop() };
  },
});

// ── topic browser ──────────────────────────────────────────────────────────
register({
  type: 'topics',
  title: 'Topic browser',
  group: 'Generic',
  desc: 'Every topic on the graph, with its type. Handy when a tile shows nothing and you need to know whether the topic exists at all.',
  w: 5, h: 6,
  config: [],
  create(ctx) {
    const body = el('tbody');
    const filter = el('input', { type: 'text', placeholder: 'filter…', style: { width: '100%', marginBottom: '.4rem' } });
    const root = el('div', {}, [
      filter,
      el('table', { class: 'data' }, [
        el('thead', {}, [el('tr', {}, [el('th', { text: 'topic' }), el('th', { text: 'type' })])]),
        body,
      ]),
    ]);

    let all = [];
    const render = () => {
      const q = filter.value.toLowerCase();
      body.textContent = '';
      for (const t of all) {
        if (q && !t.name.toLowerCase().includes(q) && !t.type.toLowerCase().includes(q)) continue;
        body.appendChild(el('tr', {}, [
          el('td', { text: t.name }),
          el('td', { style: { color: 'var(--muted)', fontSize: '.7rem' }, text: t.type.replace(/^.*\//, '') }),
        ]));
      }
    };
    filter.addEventListener('input', render);

    const refresh = () => R.getTopics(true).then((t) => { all = t; render(); }).catch(() => {});
    refresh();
    const timer = setInterval(refresh, 10000);
    return { el: root, title: 'Topics', destroy: () => clearInterval(timer) };
  },
});

// ── raw message ────────────────────────────────────────────────────────────
register({
  type: 'echo',
  title: 'Raw message',
  group: 'Generic',
  desc: 'The latest message on a topic as JSON — the dashboard equivalent of ros2 topic echo.',
  w: 4, h: 5,
  config: [{ key: 'topic', label: 'Topic', type: 'topic' }],
  create(ctx) {
    const { topic, type } = ctx.cfg;
    const pre = el('pre', { class: 'log', text: 'waiting…' });
    let last = 0;
    const stop = topic && type ? R.subscribe(topic, type, (msg) => {
      // Throttled: JSON.stringify of a 100 Hz message is a good way to make a
      // phone unusable, and nobody can read ten updates a second anyway.
      const now = performance.now();
      if (now - last < 250) return;
      last = now;
      pre.textContent = JSON.stringify(msg, null, 1);
    }) : null;
    return { el: pre, title: ctx.cfg.title || topic || 'echo', destroy: () => stop && stop() };
  },
});

// ── generic Trigger service button ─────────────────────────────────────────
register({
  type: 'trigger',
  title: 'Service button',
  group: 'Generic',
  desc: 'Calls any std_srvs/Trigger service and shows what came back.',
  w: 3, h: 3,
  config: [
    { key: 'service', label: 'Service name', type: 'text', placeholder: '/dashboard/git_status' },
    { key: 'label', label: 'Button label', type: 'text', def: 'Call' },
    { key: 'confirm', label: 'Ask before calling', type: 'bool', def: false },
  ],
  create(ctx) {
    const out = el('p', { class: 'note', text: '—' });
    const btn = el('button', {
      text: ctx.cfg.label || 'Call',
      onclick: async () => {
        if (ctx.cfg.confirm && !window.confirm(`Call ${ctx.cfg.service}?`)) return;
        btn.disabled = true;
        out.textContent = 'calling…';
        out.className = 'note';
        try {
          const res = await R.callService(ctx.cfg.service, 'std_srvs/srv/Trigger', {});
          out.textContent = `${res.success ? '✓' : '✕'} ${res.message || ''}`;
          out.className = `note ${res.success ? 'ok' : 'bad'}`;
        } catch (err) {
          out.textContent = `✕ ${err.message}`;
          out.className = 'note bad';
        } finally { btn.disabled = false; }
      },
    });
    return { el: el('div', {}, [btn, out]), title: ctx.cfg.title || ctx.cfg.service || 'service' };
  },
});

// ── generic parameter tuner ────────────────────────────────────────────────
register({
  type: 'params',
  title: 'Parameters',
  group: 'Tuning',
  desc: 'Every parameter of any node, editable live. The same call ros2 param set makes.',
  w: 5, h: 6,
  config: [
    { key: 'node', label: 'Node', type: 'node' },
    { key: 'filter', label: 'Only names containing', type: 'text' },
  ],
  create(ctx) {
    const root = el('div', {});
    const note = el('p', { class: 'note', text: 'loading…' });
    root.appendChild(note);
    const list = el('div', {});
    root.appendChild(list);

    const load = async () => {
      const node = ctx.cfg.node;
      if (!node) { note.textContent = 'pick a node in settings'; return; }
      try {
        let names = await R.listParams(node);
        if (ctx.cfg.filter) names = names.filter((n) => n.includes(ctx.cfg.filter));
        const values = await R.getParams(node, names);
        note.textContent = `${names.length} parameter(s) — edits apply immediately`;
        list.textContent = '';
        for (const name of names) {
          const entry = values[name];
          list.appendChild(paramRow(node, name, entry, note));
        }
      } catch (err) {
        note.textContent = `✕ ${err.message} — is ${node} running?`;
        note.className = 'note bad';
      }
    };
    load();
    return { el: root, title: ctx.cfg.node ? `params · ${ctx.cfg.node}` : 'Parameters' };
  },
});

/** One editable parameter. Text entry, because a generic tile cannot know a
 *  sensible slider range and a slider with the wrong range is worse than none. */
function paramRow(node, name, entry, note) {
  const input = el('input', {
    type: 'text', value: String(entry.value),
    style: { width: '7rem', textAlign: 'right', fontFamily: 'var(--mono)', fontSize: '.75rem' },
  });
  const apply = async () => {
    let value = input.value;
    if (entry.type === R.P_DOUBLE || entry.type === R.P_INT) value = Number(value);
    else if (entry.type === R.P_BOOL) value = /^(true|1|yes)$/i.test(value);
    try {
      const [res] = await R.setParams(node, [{ name, type: entry.type, value }]);
      note.textContent = res.successful ? `✓ ${name} = ${value}` : `✕ ${name}: ${res.reason}`;
      note.className = `note ${res.successful ? 'ok' : 'bad'}`;
      if (res.successful) entry.value = value;
    } catch (err) {
      note.textContent = `✕ ${err.message}`;
      note.className = 'note bad';
    }
  };
  input.addEventListener('change', apply);
  return el('div', {
    style: { display: 'flex', justifyContent: 'space-between', alignItems: 'center', gap: '.4rem', marginBottom: '.25rem' },
  }, [el('span', { style: { fontSize: '.75rem' }, text: name }), input]);
}

// ── connection / diagnostics ───────────────────────────────────────────────
register({
  type: 'diag',
  title: 'Connection',
  group: 'System',
  desc: 'rosbridge link, node count, and how stale the key topics are.',
  w: 4, h: 4,
  config: [
    { key: 'watch', label: 'Topics to age-check (comma separated)', type: 'text',
      def: '/imu,/joint_states,/motor_telemetry,/mode' },
  ],
  create(ctx) {
    const conn = el('div', {});
    const body = el('tbody');
    const root = el('div', {}, [conn, el('table', { class: 'data' }, [
      el('thead', {}, [el('tr', {}, [el('th', { text: 'topic' }), el('th', { text: 'age' })])]), body])]);

    const watch = (ctx.cfg.watch || '/imu,/joint_states,/motor_telemetry,/mode')
      .split(',').map((s) => s.trim()).filter(Boolean);

    // Subscribe so there is something to measure the age OF. Shared refcounting
    // means this costs nothing if another tile already watches these.
    const stops = [];
    R.getTopics().then((topics) => {
      for (const name of watch) {
        const t = topics.find((x) => x.name === name);
        if (t) stops.push(R.subscribe(t.name, t.type, () => {}));
      }
    }).catch(() => {});

    const tick = () => {
      conn.textContent = '';
      conn.appendChild(statusChip(R.state.connected ? 'good' : 'critical',
        R.state.connected ? `connected · ${R.state.host}` : 'disconnected — retrying'));
      body.textContent = '';
      R.getTopics().then((topics) => {
        for (const name of watch) {
          const t = topics.find((x) => x.name === name);
          const age = t ? R.topicAge(t.name, t.type) : null;
          const label = age === null ? 'never' : `${age.toFixed(1)} s`;
          const bad = age === null || age > 2;
          body.appendChild(el('tr', {}, [
            el('td', { text: name }),
            el('td', { style: { color: bad ? 'var(--critical)' : 'var(--ink)' }, text: label }),
          ]));
        }
      }).catch(() => {});
    };
    tick();
    const timer = setInterval(tick, 1000);
    return { el: root, title: 'Connection', destroy: () => { clearInterval(timer); stops.forEach((s) => s()); } };
  },
});
