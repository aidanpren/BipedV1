// ---------------------------------------------------------------------------
// Bootstrap: the top bar, named layouts, the add-tile palette, and the
// per-tile settings form.
//
// Layouts live in localStorage, which is a deliberate choice and has one
// consequence worth knowing: a layout belongs to the BROWSER, not to the robot.
// The phone and the laptop each keep their own, which is usually what you want
// (a phone layout is not a desk layout). Export/Import moves one between them.
// ---------------------------------------------------------------------------

import * as R from './ros.js';
import { Grid, compact, findSlot } from './grid.js';
import { registry } from './tiles-core.js';
import './tiles-robot.js';
import { el } from './widgets.js';

const STORE_LAYOUTS = 'biped.dash.layouts';
const STORE_CURRENT = 'biped.dash.current';
const STORE_THEME = 'biped.dash.theme';

// The layout a fresh browser gets. Chosen to answer, top to bottom: what is the
// robot doing, is it healthy, how is it driving, and what can I change about it.
const DEFAULT_LAYOUT = [
  { type: 'mode', x: 0, y: 0, w: 3, h: 4 },
  { type: 'power', x: 3, y: 0, w: 4, h: 5 },
  { type: 'pitch', x: 7, y: 0, w: 5, h: 5 },
  { type: 'motors', x: 0, y: 5, w: 6, h: 5 },
  { type: 'drivestate', x: 6, y: 5, w: 6, h: 5 },
  { type: 'drive', x: 0, y: 10, w: 6, h: 9 },
  { type: 'deploy', x: 6, y: 10, w: 6, h: 9 },
  { type: 'save', x: 0, y: 19, w: 5, h: 6 },
  { type: 'diag', x: 5, y: 19, w: 4, h: 5 },
];

let uid = 0;
const newId = () => `t${Date.now().toString(36)}${(uid += 1)}`;

// ── persistence ────────────────────────────────────────────────────────────
function loadLayouts() {
  try {
    const raw = JSON.parse(localStorage.getItem(STORE_LAYOUTS) || 'null');
    if (raw && typeof raw === 'object' && Object.keys(raw).length) return raw;
  } catch (err) { /* corrupt storage is not worth crashing over */ }
  return { Default: DEFAULT_LAYOUT.map((t) => ({ ...t, id: newId(), cfg: {} })) };
}

let layouts = loadLayouts();
let current = localStorage.getItem(STORE_CURRENT);
if (!current || !layouts[current]) current = Object.keys(layouts)[0];

function save() {
  layouts[current] = grid.items.map((i) => ({ ...i }));
  localStorage.setItem(STORE_LAYOUTS, JSON.stringify(layouts));
  localStorage.setItem(STORE_CURRENT, current);
}

// ── theme ──────────────────────────────────────────────────────────────────
const storedTheme = localStorage.getItem(STORE_THEME);
if (storedTheme) document.documentElement.setAttribute('data-theme', storedTheme);

function cycleTheme() {
  const now = document.documentElement.getAttribute('data-theme');
  const next = now === 'dark' ? 'light' : (now === 'light' ? '' : 'dark');
  if (next) {
    document.documentElement.setAttribute('data-theme', next);
    localStorage.setItem(STORE_THEME, next);
  } else {
    document.documentElement.removeAttribute('data-theme');
    localStorage.removeItem(STORE_THEME);
  }
  themeBtn.textContent = next === 'dark' ? '🌙' : (next === 'light' ? '☀' : '◐');
  themeBtn.title = `Theme: ${next || 'follow system'}`;
}

// ── grid ───────────────────────────────────────────────────────────────────
const grid = new Grid(document.getElementById('grid'));
grid.addEventListener('change', save);
grid.addEventListener('remove', (ev) => grid.remove(ev.detail.id));
grid.addEventListener('settings', (ev) => openSettings(ev.detail));

function mount(item) {
  const def = registry.get(item.type);
  if (!def) {
    grid.add(item, { el: el('p', { class: 'note bad', text: `unknown tile type "${item.type}"` }), title: item.type });
    return;
  }
  let view;
  try {
    view = def.create({ cfg: item.cfg || {}, item });
  } catch (err) {
    // One broken tile must not take the page down with it — that would make a
    // bad config unrecoverable without clearing localStorage by hand.
    console.error(item.type, err);
    view = { el: el('pre', { class: 'log err', text: String(err && err.stack || err) }), title: `${item.type} — error` };
  }
  grid.add(item, view);
}

function render() {
  grid.clear();
  const items = (layouts[current] || []).map((t) => ({ ...t, id: t.id || newId(), cfg: t.cfg || {} }));
  compact(items);
  for (const item of items) mount(item);
  grid.layout();
}

function addTile(type) {
  const def = registry.get(type);
  const slot = findSlot(grid.items, def.w || 4, def.h || 4);
  const item = { id: newId(), type, cfg: {}, w: def.w || 4, h: def.h || 4, ...slot };
  mount(item);
  compact(grid.items);
  grid.layout();
  save();
  return item;
}

function reloadTile(item) {
  // Rebuild in place: destroy the old view (releasing its subscriptions) and
  // create a new one with the new config, keeping the same cell.
  const index = grid.items.findIndex((i) => i.id === item.id);
  const box = { ...grid.items[index] };
  grid.remove(item.id);
  const fresh = { ...box, cfg: item.cfg };
  mount(fresh);
  compact(grid.items);
  grid.layout();
  save();
}

// ── top bar ────────────────────────────────────────────────────────────────
const connChip = document.getElementById('conn');
const modeChip = document.getElementById('modeChip');
const layoutSel = document.getElementById('layoutSel');
const editBtn = document.getElementById('editBtn');
const themeBtn = document.getElementById('themeBtn');

function setConn(connected) {
  connChip.innerHTML = '';
  connChip.appendChild(el('span', {
    class: `glyph ${connected ? 'g-good' : 'g-critical'}`, text: connected ? '●' : '✕',
  }));
  connChip.appendChild(el('span', { text: connected ? R.state.host : 'reconnecting…' }));
}
setConn(false);
R.bus.addEventListener('conn', (ev) => setConn(ev.detail.connected));

const MODE_GLYPH = { disabled: ['■', 'g-muted'], teleop: ['●', 'g-good'], autonomous: ['◆', 'g-warning'] };
R.subscribe('/mode', 'std_msgs/msg/String', (msg) => {
  const [glyph, cls] = MODE_GLYPH[msg.data] || ['?', 'g-muted'];
  modeChip.innerHTML = '';
  modeChip.appendChild(el('span', { class: `glyph ${cls}`, text: glyph }));
  modeChip.appendChild(el('span', { text: msg.data }));
});

function refreshLayoutSelect() {
  layoutSel.innerHTML = '';
  for (const name of Object.keys(layouts)) {
    layoutSel.appendChild(el('option', { value: name, selected: name === current ? '' : null, text: name }));
  }
}
refreshLayoutSelect();

layoutSel.addEventListener('change', () => {
  save();
  current = layoutSel.value;
  localStorage.setItem(STORE_CURRENT, current);
  render();
});

editBtn.addEventListener('click', () => {
  const on = editBtn.getAttribute('aria-pressed') !== 'true';
  editBtn.setAttribute('aria-pressed', String(on));
  grid.setEditing(on);
  document.getElementById('editHint').hidden = !on;
});

themeBtn.addEventListener('click', cycleTheme);
themeBtn.textContent = storedTheme === 'dark' ? '🌙' : (storedTheme === 'light' ? '☀' : '◐');

document.getElementById('addBtn').addEventListener('click', openPalette);

document.getElementById('menuBtn').addEventListener('click', () => {
  const menu = document.getElementById('menu');
  menu.hidden = !menu.hidden;
});

document.getElementById('newLayout').addEventListener('click', () => {
  const name = window.prompt('Name for the new layout');
  if (!name) return;
  save();
  layouts[name] = [];
  current = name;
  refreshLayoutSelect();
  render();
  save();
});

document.getElementById('dupLayout').addEventListener('click', () => {
  const name = window.prompt('Name for the copy', `${current} copy`);
  if (!name) return;
  save();
  layouts[name] = layouts[current].map((t) => ({ ...t, id: newId() }));
  current = name;
  refreshLayoutSelect();
  render();
  save();
});

document.getElementById('delLayout').addEventListener('click', () => {
  if (Object.keys(layouts).length < 2) { window.alert('That is the only layout.'); return; }
  if (!window.confirm(`Delete layout "${current}"?`)) return;
  delete layouts[current];
  current = Object.keys(layouts)[0];
  refreshLayoutSelect();
  render();
  save();
});

document.getElementById('resetLayout').addEventListener('click', () => {
  if (!window.confirm(`Reset "${current}" to the default tiles?`)) return;
  layouts[current] = DEFAULT_LAYOUT.map((t) => ({ ...t, id: newId(), cfg: {} }));
  render();
  save();
});

document.getElementById('exportBtn').addEventListener('click', () => {
  save();
  // A data: URL, not a fetch to some paste service — the robot has no internet
  // when it is running as its own access point, which is exactly when you are
  // most likely to want to move a layout to your phone.
  const blob = new Blob([JSON.stringify(layouts, null, 2)], { type: 'application/json' });
  const a = el('a', { href: URL.createObjectURL(blob), download: 'biped-dashboard-layouts.json' });
  document.body.appendChild(a); a.click(); a.remove();
});

document.getElementById('importFile').addEventListener('change', (ev) => {
  const file = ev.target.files[0];
  if (!file) return;
  file.text().then((text) => {
    const incoming = JSON.parse(text);
    layouts = { ...layouts, ...incoming };
    current = Object.keys(incoming)[0] || current;
    refreshLayoutSelect();
    render();
    save();
  }).catch((err) => window.alert(`Could not read that file: ${err.message}`));
});

// ── add-tile palette ───────────────────────────────────────────────────────
function openPalette() {
  const groups = new Map();
  for (const def of registry.values()) {
    if (!groups.has(def.group)) groups.set(def.group, []);
    groups.get(def.group).push(def);
  }
  const body = el('div', {});
  for (const [group, defs] of groups) {
    body.appendChild(el('div', { class: 'palette-group' }, [
      el('h3', { text: group }),
      el('div', { class: 'cards' }, defs.map((def) => el('button', {
        class: 'card',
        onclick: () => {
          closeSheet();
          const item = addTile(def.type);
          // A tile that needs a topic is useless until it has one, so go
          // straight to its settings instead of adding an empty box.
          if ((def.config || []).some((f) => f.type === 'topic' || f.type === 'node')) openSettings(item);
        },
      }, [el('strong', { text: def.title }), el('span', { text: def.desc })]))),
    ]));
  }
  showSheet('Add a tile', body, []);
}

// ── settings form ──────────────────────────────────────────────────────────
async function openSettings(item) {
  const def = registry.get(item.type);
  if (!def) return;
  const cfg = { ...(item.cfg || {}) };
  const body = el('div', {});

  // Every tile gets a title override — a page with three Plot tiles all called
  // "Plot" is not a dashboard.
  body.appendChild(formField('Tile title', 'Blank uses the automatic title.',
    el('input', { type: 'text', value: cfg.title || '', oninput: (e) => { cfg.title = e.target.value; } })));

  let topics = [];
  try { topics = await R.getTopics(); } catch (err) { /* offline: fall back to typing */ }

  const fieldSelectors = [];

  for (const spec of (def.config || [])) {
    if (cfg[spec.key] === undefined && spec.def !== undefined) cfg[spec.key] = spec.def;

    if (spec.type === 'topic') {
      const select = el('select', {
        onchange: (e) => {
          cfg[spec.key] = e.target.value;
          const found = topics.find((t) => t.name === e.target.value);
          cfg.type = found ? found.type : cfg.type;
          fieldSelectors.forEach((fn) => fn(cfg.type));
        },
      });
      select.appendChild(el('option', { value: '', text: '— pick a topic —' }));
      for (const t of topics) {
        select.appendChild(el('option', {
          value: t.name, selected: t.name === cfg[spec.key] ? '' : null,
          text: `${t.name}   (${t.type.replace(/^.*\//, '')})`,
        }));
      }
      // A topic that is not publishing right now does not appear in the list,
      // so keep a way to name one anyway. It still has to resolve the TYPE —
      // forgetting that leaves cfg.type pointing at whatever was chosen before,
      // and the tile then subscribes with the wrong type and silently receives
      // nothing at all.
      const manual = el('input', {
        type: 'text', placeholder: 'or type a topic name', value: cfg[spec.key] || '',
        onchange: (e) => {
          cfg[spec.key] = e.target.value;
          const found = topics.find((t) => t.name === e.target.value);
          if (found) cfg.type = found.type;
          fieldSelectors.forEach((fn) => fn(cfg.type));
        },
      });
      body.appendChild(formField(spec.label, 'Type comes from the graph automatically.', select, manual));
    } else if (spec.type === 'field' || spec.type === 'fields') {
      const multi = spec.type === 'fields';
      const select = el('select', multi ? { multiple: '', size: 6 } : {});
      const fill = async (type) => {
        select.innerHTML = '';
        if (!type) { select.appendChild(el('option', { value: '', text: '— pick a topic first —' })); return; }
        let fields = [];
        try { fields = await R.getFields(type); } catch (err) { /* leave empty */ }
        if (!multi) select.appendChild(el('option', { value: '', text: '— whole message —' }));
        const chosen = multi ? (cfg[spec.key] || []) : [cfg[spec.key]];
        for (const f of fields) {
          select.appendChild(el('option', {
            value: f.path, selected: chosen.includes(f.path) ? '' : null,
            text: `${f.path}  ·  ${f.type}${f.array ? '[]' : ''}`,
          }));
        }
      };
      select.addEventListener('change', () => {
        cfg[spec.key] = multi
          ? [...select.selectedOptions].map((o) => o.value).slice(0, spec.max || 4)
          : select.value;
      });
      fieldSelectors.push(fill);
      fill(cfg.type);
      body.appendChild(formField(spec.label,
        multi ? 'Ctrl/⌘-click for more than one.' : '', select));
    } else if (spec.type === 'node') {
      const select = el('select', { onchange: (e) => { cfg[spec.key] = e.target.value; } });
      select.appendChild(el('option', { value: '', text: '— pick a node —' }));
      try {
        for (const n of await R.getNodes()) {
          const bare = n.replace(/^\//, '');
          select.appendChild(el('option', { value: bare, selected: bare === cfg[spec.key] ? '' : null, text: bare }));
        }
      } catch (err) { /* offline */ }
      body.appendChild(formField(spec.label, '', select));
    } else if (spec.type === 'bool') {
      const box = el('input', {
        type: 'checkbox', checked: cfg[spec.key] ? '' : null,
        onchange: (e) => { cfg[spec.key] = e.target.checked; },
      });
      body.appendChild(el('div', { class: 'field' }, [
        el('label', {}, [box, el('span', { text: ` ${spec.label}` })]),
      ]));
    } else if (spec.type === 'select') {
      const select = el('select', { onchange: (e) => { cfg[spec.key] = e.target.value; } });
      for (const opt of spec.options) {
        select.appendChild(el('option', {
          value: opt.value, selected: opt.value === cfg[spec.key] ? '' : null, text: opt.label,
        }));
      }
      body.appendChild(formField(spec.label, '', select));
    } else {
      const input = el('input', {
        type: spec.type === 'number' ? 'number' : 'text',
        step: spec.step, min: spec.min, max: spec.max,
        placeholder: spec.placeholder || '',
        value: cfg[spec.key] ?? '',
        oninput: (e) => {
          cfg[spec.key] = spec.type === 'number'
            ? (e.target.value === '' ? undefined : Number(e.target.value))
            : e.target.value;
        },
      });
      body.appendChild(formField(spec.label, spec.hint || '', input));
    }
  }

  showSheet(`${def.title} settings`, body, [
    el('button', { text: 'Cancel', onclick: closeSheet }),
    el('button', {
      class: 'primary', text: 'Apply',
      onclick: () => { closeSheet(); item.cfg = cfg; reloadTile(item); },
    }),
  ]);
}

function formField(label, hint, ...controls) {
  return el('div', { class: 'field' }, [
    el('label', { text: label }),
    ...controls,
    hint ? el('div', { class: 'hint', text: hint }) : null,
  ]);
}

// ── sheet (palette + settings share one) ───────────────────────────────────
const scrim = document.getElementById('scrim');

function showSheet(title, body, footer) {
  document.getElementById('sheetTitle').textContent = title;
  const host = document.getElementById('sheetBody');
  host.innerHTML = '';
  host.appendChild(body);
  const foot = document.getElementById('sheetFoot');
  foot.innerHTML = '';
  for (const b of footer) foot.appendChild(b);
  if (!footer.length) foot.appendChild(el('button', { text: 'Close', onclick: closeSheet }));
  scrim.hidden = false;
}

function closeSheet() { scrim.hidden = true; }

scrim.addEventListener('click', (ev) => { if (ev.target === scrim) closeSheet(); });
document.getElementById('sheetClose').addEventListener('click', closeSheet);
document.addEventListener('keydown', (ev) => { if (ev.key === 'Escape') closeSheet(); });
document.addEventListener('click', (ev) => {
  const menu = document.getElementById('menu');
  if (!menu.hidden && !menu.contains(ev.target) && ev.target.id !== 'menuBtn') menu.hidden = true;
});

render();
