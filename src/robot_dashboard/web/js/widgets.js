// ---------------------------------------------------------------------------
// Shared drawing pieces. Tiles compose these rather than each inventing its own
// chart, so every plot on the page shares one set of conventions: thin marks, a
// recessive grid, a crosshair tooltip, and colours taken by SLOT so a series
// keeps its identity when its neighbours come and go.
// ---------------------------------------------------------------------------

export const SERIES = ['--s1', '--s2', '--s3', '--s4', '--s5', '--s6', '--s7', '--s8'];

/** Categorical slot -> CSS colour. Assign by ENTITY, never by array position. */
export function seriesColor(slot) {
  return `var(${SERIES[slot % SERIES.length]})`;
}

export function el(tag, props = {}, kids = []) {
  const node = document.createElement(tag);
  for (const [k, v] of Object.entries(props)) {
    if (k === 'class') node.className = v;
    else if (k === 'html') node.innerHTML = v;
    else if (k === 'text') node.textContent = v;
    else if (k === 'style') Object.assign(node.style, v);
    else if (k.startsWith('on')) node.addEventListener(k.slice(2).toLowerCase(), v);
    else if (v !== null && v !== undefined) node.setAttribute(k, v);
  }
  for (const kid of [].concat(kids)) {
    if (kid) node.appendChild(typeof kid === 'string' ? document.createTextNode(kid) : kid);
  }
  return node;
}

export function svg(tag, props = {}) {
  const node = document.createElementNS('http://www.w3.org/2000/svg', tag);
  for (const [k, v] of Object.entries(props)) {
    if (v !== null && v !== undefined) node.setAttribute(k, v);
  }
  return node;
}

/** Numbers for reading, not for maximum precision. */
export function fmt(value, digits = 2) {
  if (value === null || value === undefined || Number.isNaN(value)) return '—';
  if (typeof value === 'boolean') return value ? 'true' : 'false';
  if (typeof value !== 'number') return String(value);
  if (!Number.isFinite(value)) return '∞';
  const abs = Math.abs(value);
  if (abs !== 0 && (abs < 1e-3 || abs >= 1e6)) return value.toExponential(1);
  return value.toFixed(digits);
}

/**
 * A scrolling multi-series line plot.
 *
 * Y-SCALE DEFAULTS TO FIXED, not auto. Auto-scaling a balancing robot's pitch
 * makes a motionless robot's 0.002 rad of sensor noise fill the whole tile and
 * look like a crisis, and then makes a real 0.2 rad lean look identical. A
 * fixed scale is the difference between a plot you can glance at and one you
 * have to read the axis of every time. Auto is available per tile for signals
 * whose range genuinely is not known in advance.
 */
export class TimePlot {
  constructor(opts = {}) {
    this.span = opts.span || 15;          // seconds of history
    this.min = opts.min ?? -1;
    this.max = opts.max ?? 1;
    this.auto = opts.auto || false;
    this.zeroLine = opts.zeroLine !== false;
    this.names = opts.names || [];
    this.slots = opts.slots || this.names.map((_, i) => i);

    this.samples = [];                    // {t, v: [..]}
    this.dirty = false;
    this.raf = null;

    this.el = el('div', { class: 'plot', style: { position: 'relative', height: '100%', minHeight: '60px' } });
    this.svg = svg('svg', { width: '100%', height: '100%', preserveAspectRatio: 'none' });
    this.svg.style.display = 'block';
    this.el.appendChild(this.svg);

    this.gGrid = svg('g'); this.svg.appendChild(this.gGrid);
    this.paths = this.names.map((_, i) => {
      const p = svg('path', {
        fill: 'none', stroke: seriesColor(this.slots[i]),
        'stroke-width': 2, 'stroke-linejoin': 'round', 'stroke-linecap': 'round',
        'vector-effect': 'non-scaling-stroke',
      });
      this.svg.appendChild(p);
      return p;
    });
    this.cursor = svg('line', { stroke: 'var(--axis)', 'stroke-width': 1, opacity: 0 });
    this.svg.appendChild(this.cursor);

    this.tip = el('div', {
      style: {
        position: 'absolute', pointerEvents: 'none', opacity: '0',
        background: 'var(--raised)', border: '1px solid var(--border)',
        borderRadius: '6px', padding: '.25rem .4rem', fontSize: '.7rem',
        fontVariantNumeric: 'tabular-nums', whiteSpace: 'nowrap', zIndex: '5',
        boxShadow: '0 2px 8px rgba(0,0,0,.18)',
      },
    });
    this.el.appendChild(this.tip);

    // An HTML chart IS interactive; a line plot without a crosshair is throwing
    // away the one advantage it has over a printed one.
    this.el.addEventListener('pointermove', (e) => this.hover(e));
    this.el.addEventListener('pointerleave', () => this.unhover());
  }

  push(values, t = performance.now() / 1000) {
    this.samples.push({ t, v: Array.isArray(values) ? values : [values] });
    const cutoff = t - this.span;
    while (this.samples.length && this.samples[0].t < cutoff) this.samples.shift();
    this.schedule();
  }

  schedule() {
    if (this.raf) return;
    // Coalesce to one draw per animation frame. /imu arrives at 100 Hz and a
    // redraw per message pins a phone's CPU for no readability gain whatsoever.
    this.raf = requestAnimationFrame(() => { this.raf = null; this.draw(); });
  }

  bounds() {
    if (!this.auto) return [this.min, this.max];
    let lo = Infinity, hi = -Infinity;
    for (const s of this.samples) {
      for (const v of s.v) {
        if (typeof v === 'number' && Number.isFinite(v)) { lo = Math.min(lo, v); hi = Math.max(hi, v); }
      }
    }
    if (!Number.isFinite(lo)) return [this.min, this.max];
    if (hi - lo < 1e-9) { lo -= 0.5; hi += 0.5; }
    const pad = (hi - lo) * 0.12;
    return [lo - pad, hi + pad];
  }

  draw() {
    const W = this.el.clientWidth || 300;
    const H = this.el.clientHeight || 80;
    if (!W || !H) return;
    this.svg.setAttribute('viewBox', `0 0 ${W} ${H}`);
    const [lo, hi] = this.bounds();
    this.lo = lo; this.hi = hi;
    const now = this.samples.length ? this.samples[this.samples.length - 1].t : 0;
    const x = (t) => W - ((now - t) / this.span) * W;
    const y = (v) => H - ((v - lo) / (hi - lo || 1)) * H;
    this.xOf = x; this.yOf = y; this.W = W; this.H = H; this.now = now;

    // Recessive chrome: one hairline at zero if zero is in view, nothing else.
    // Gridlines every N units would compete with the data at this tile size.
    this.gGrid.textContent = '';
    if (this.zeroLine && lo < 0 && hi > 0) {
      this.gGrid.appendChild(svg('line', {
        x1: 0, x2: W, y1: y(0), y2: y(0), stroke: 'var(--axis)', 'stroke-width': 1,
      }));
    }

    this.paths.forEach((path, i) => {
      let d = '';
      let pen = false;
      for (const s of this.samples) {
        const v = s.v[i];
        if (typeof v !== 'number' || !Number.isFinite(v)) { pen = false; continue; }
        d += `${pen ? 'L' : 'M'}${x(s.t).toFixed(1)},${y(v).toFixed(1)}`;
        pen = true;
      }
      path.setAttribute('d', d);
    });
  }

  hover(ev) {
    if (!this.samples.length || !this.xOf) return;
    const rect = this.el.getBoundingClientRect();
    const px = ev.clientX - rect.left;
    let best = null, bestDist = Infinity;
    for (const s of this.samples) {
      const dist = Math.abs(this.xOf(s.t) - px);
      if (dist < bestDist) { bestDist = dist; best = s; }
    }
    if (!best) return;
    const bx = this.xOf(best.t);
    this.cursor.setAttribute('x1', bx); this.cursor.setAttribute('x2', bx);
    this.cursor.setAttribute('y1', 0); this.cursor.setAttribute('y2', this.H);
    this.cursor.setAttribute('opacity', 1);

    const rows = this.names.map((n, i) =>
      `<span style="color:${seriesColor(this.slots[i])}">■</span> ${n} ${fmt(best.v[i], 3)}`);
    this.tip.innerHTML = `${(this.now - best.t).toFixed(1)}s ago<br>${rows.join('<br>')}`;
    this.tip.style.opacity = '1';
    const tw = this.tip.offsetWidth;
    this.tip.style.left = `${Math.max(0, Math.min(this.W - tw, bx + 8))}px`;
    this.tip.style.top = '2px';
  }

  unhover() {
    this.cursor.setAttribute('opacity', 0);
    this.tip.style.opacity = '0';
  }

  resized() { this.schedule(); }
}

/**
 * A horizontal meter with a status band.
 *
 * Used for battery state of charge. The FILL carries magnitude and the status
 * colour carries state, and the tile beside it always prints the number and a
 * word — a meter that has gone red is useless if you cannot tell whether that
 * means 20% or 2%.
 */
export class Meter {
  constructor() {
    this.el = el('div', {
      style: {
        height: '10px', borderRadius: '5px', background: 'var(--grid)',
        overflow: 'hidden', margin: '.35rem 0 .1rem',
      },
    });
    this.fill = el('div', { style: { height: '100%', width: '0%', background: 'var(--good)', borderRadius: '5px', transition: 'width .3s ease' } });
    this.el.appendChild(this.fill);
  }

  set(frac, status = 'good') {
    const pct = Math.max(0, Math.min(1, frac || 0)) * 100;
    this.fill.style.width = `${pct}%`;
    this.fill.style.background = `var(--${status})`;
  }
}

/** colour + SHAPE + WORD, in that order of reliability. */
export const STATUS_GLYPH = {
  good: '●', warning: '▲', serious: '▲', critical: '✕', muted: '■',
};

export function statusChip(status, text) {
  return el('span', { class: 'chip' }, [
    el('span', { class: `glyph g-${status}`, text: STATUS_GLYPH[status] || '■' }),
    el('span', { text }),
  ]);
}
