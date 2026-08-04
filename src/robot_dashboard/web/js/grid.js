// ---------------------------------------------------------------------------
// The grid: drag, resize, snap, collide, persist.
//
// Layout is stored in a FIXED 12-COLUMN SPACE, always, on every device. What
// changes on a phone is only how that layout is DRAWN — narrow screens stack
// the tiles in reading order at full width. This separation is the whole trick:
// if narrow rendering wrote back to the model, turning a phone sideways or
// dragging a desktop window narrow would quietly flatten a layout you spent
// twenty minutes building, and there would be no undo.
//
// Collision behaviour is "vertical compact", the same rule FRC Shuffleboard and
// react-grid-layout use: tiles fall upward into free space, and a tile dropped
// onto an occupied cell pushes the occupant down rather than overlapping it.
// It is predictable in a way that free-floating boxes are not — you can never
// end up with two tiles on top of each other or a layout you cannot untangle.
// ---------------------------------------------------------------------------

export const COLS = 12;
export const ROW_H = 46;      // px per grid row
export const GAP = 10;        // px between tiles
const NARROW = 700;           // px; below this the layout stacks

function hits(a, b, ay = a.y, ax = a.x) {
  return ax < b.x + b.w && ax + a.w > b.x && ay < b.y + b.h && ay + a.h > b.y;
}

/**
 * Push everything into a legal, gap-free arrangement.
 *
 * `pinnedId` is the tile the user is currently holding: it keeps the exact cell
 * the pointer put it in, and everything else arranges itself around that. Without
 * a pin, a dragged tile would be compacted upward out from under the cursor the
 * instant it was dropped anywhere below the top row.
 */
export function compact(items, pinnedId = null) {
  const pinned = pinnedId ? items.find((i) => i.id === pinnedId) : null;
  const placed = [];
  if (pinned) { pinned.y = Math.max(0, pinned.y); placed.push(pinned); }

  const rest = items.filter((i) => i !== pinned)
    .sort((a, b) => (a.y - b.y) || (a.x - b.x));

  for (const it of rest) {
    let y = Math.max(0, it.y);
    // float up into any free space directly above...
    while (y > 0 && !placed.some((p) => hits(it, p, y - 1))) y -= 1;
    // ...then sink until this row is actually free.
    while (placed.some((p) => hits(it, p, y))) y += 1;
    it.y = y;
    placed.push(it);
  }
  return items;
}

/** First free slot big enough for a w x h tile, scanning row by row. */
export function findSlot(items, w, h) {
  for (let y = 0; y < 500; y += 1) {
    for (let x = 0; x + w <= COLS; x += 1) {
      const probe = { x, y, w, h };
      if (!items.some((it) => hits(probe, it))) return { x, y };
    }
  }
  return { x: 0, y: 0 };
}

export class Grid extends EventTarget {
  constructor(container) {
    super();
    this.el = container;
    this.el.classList.add('grid');
    this.items = [];
    this.views = new Map();      // id -> {root, body, item}
    this.editing = false;
    this.drag = null;

    this.placeholder = document.createElement('div');
    this.placeholder.className = 'placeholder';
    this.placeholder.hidden = true;
    this.el.appendChild(this.placeholder);

    this._onResize = () => this.layout();
    window.addEventListener('resize', this._onResize);
  }

  get narrow() { return this.el.clientWidth < NARROW; }

  get cellW() {
    return (this.el.clientWidth - GAP * (COLS - 1)) / COLS;
  }

  setEditing(on) {
    this.editing = on;
    this.el.classList.toggle('editing', on);
  }

  // ── model ───────────────────────────────────────────────────────────────
  add(item, view) {
    this.items.push(item);
    const root = document.createElement('section');
    root.className = 'tile';
    root.dataset.id = item.id;
    root.innerHTML = `
      <header data-drag>
        <span class="t-title"></span>
        <span class="t-actions">
          <button class="t-btn" data-act="settings" title="Settings" aria-label="Tile settings">⚙</button>
          <button class="t-btn" data-act="remove" title="Remove" aria-label="Remove tile">✕</button>
        </span>
      </header>
      <div class="t-body"></div>
      <div class="t-resize" data-resize aria-hidden="true"></div>`;
    const body = root.querySelector('.t-body');
    body.appendChild(view.el);
    this.el.appendChild(root);
    this.views.set(item.id, { root, body, view, item });
    this.setTitle(item.id, view.title || item.type);

    root.addEventListener('pointerdown', (ev) => this.onPointerDown(ev, item));
    root.querySelector('[data-act=remove]').addEventListener('click', () => {
      this.dispatchEvent(new CustomEvent('remove', { detail: item }));
    });
    root.querySelector('[data-act=settings]').addEventListener('click', () => {
      this.dispatchEvent(new CustomEvent('settings', { detail: item }));
    });
    this.layout();
  }

  setTitle(id, text) {
    const entry = this.views.get(id);
    if (entry) entry.root.querySelector('.t-title').textContent = text;
  }

  remove(id) {
    const entry = this.views.get(id);
    if (!entry) return;
    // destroy() is what releases the tile's shared subscriptions. Skipping it
    // is the leak described at the top of ros.js.
    if (entry.view.destroy) { try { entry.view.destroy(); } catch (e) { console.error(e); } }
    entry.root.remove();
    this.views.delete(id);
    this.items = this.items.filter((i) => i.id !== id);
    compact(this.items);
    this.layout();
    this.changed();
  }

  clear() {
    for (const id of [...this.views.keys()]) {
      const entry = this.views.get(id);
      if (entry.view.destroy) { try { entry.view.destroy(); } catch (e) { console.error(e); } }
      entry.root.remove();
    }
    this.views.clear();
    this.items = [];
    this.layout();
  }

  changed() { this.dispatchEvent(new CustomEvent('change')); }

  // ── rendering ───────────────────────────────────────────────────────────
  layout() {
    const narrow = this.narrow;
    const cw = this.cellW;
    let bottom = 0;

    if (narrow) {
      // Stacked view. Heights are kept (a plot still needs to be tall) but
      // widths go full-bleed and the x coordinate is ignored.
      const order = [...this.items].sort((a, b) => (a.y - b.y) || (a.x - b.x));
      let top = 0;
      for (const it of order) {
        const entry = this.views.get(it.id);
        if (!entry) continue;
        const h = it.h * ROW_H + (it.h - 1) * GAP;
        Object.assign(entry.root.style, {
          left: '0px', top: `${top}px`,
          width: `${this.el.clientWidth}px`, height: `${h}px`,
        });
        top += h + GAP;
      }
      bottom = top;
    } else {
      for (const it of this.items) {
        const entry = this.views.get(it.id);
        if (!entry) continue;
        if (this.drag && this.drag.id === it.id) continue;   // positioned by the drag
        this.place(entry.root, it, cw);
        bottom = Math.max(bottom, (it.y + it.h) * (ROW_H + GAP));
      }
    }
    this.el.style.height = `${Math.max(bottom, ROW_H)}px`;
    for (const entry of this.views.values()) {
      if (entry.view.resized) { try { entry.view.resized(); } catch (e) { /* ignore */ } }
    }
  }

  place(el, box, cw = this.cellW) {
    Object.assign(el.style, {
      left: `${box.x * (cw + GAP)}px`,
      top: `${box.y * (ROW_H + GAP)}px`,
      width: `${box.w * cw + (box.w - 1) * GAP}px`,
      height: `${box.h * ROW_H + (box.h - 1) * GAP}px`,
    });
  }

  // ── drag & resize ───────────────────────────────────────────────────────
  onPointerDown(ev, item) {
    if (!this.editing || this.narrow) return;
    const handle = ev.target.closest('[data-drag],[data-resize]');
    if (!handle) return;
    // A button inside the header must still be clickable while editing.
    if (ev.target.closest('.t-btn')) return;
    ev.preventDefault();

    const mode = handle.hasAttribute('data-resize') ? 'resize' : 'move';
    const entry = this.views.get(item.id);
    entry.root.classList.add('dragging');
    entry.root.setPointerCapture(ev.pointerId);

    this.drag = {
      id: item.id, mode, pointerId: ev.pointerId,
      startX: ev.clientX, startY: ev.clientY,
      origin: { x: item.x, y: item.y, w: item.w, h: item.h },
      cw: this.cellW,
    };
    this.placeholder.hidden = false;
    this.place(this.placeholder, item);

    const move = (e) => this.onPointerMove(e);
    const up = (e) => {
      entry.root.removeEventListener('pointermove', move);
      entry.root.removeEventListener('pointerup', up);
      entry.root.removeEventListener('pointercancel', up);
      this.onPointerUp(e);
    };
    entry.root.addEventListener('pointermove', move);
    entry.root.addEventListener('pointerup', up);
    entry.root.addEventListener('pointercancel', up);
  }

  onPointerMove(ev) {
    const d = this.drag;
    if (!d || ev.pointerId !== d.pointerId) return;
    const item = this.items.find((i) => i.id === d.id);
    const entry = this.views.get(d.id);
    const dx = ev.clientX - d.startX;
    const dy = ev.clientY - d.startY;
    const stepX = d.cw + GAP;
    const stepY = ROW_H + GAP;

    if (d.mode === 'move') {
      // The tile follows the pointer in PIXELS while the placeholder shows the
      // CELL it would land in. Snapping the tile itself would make a drag feel
      // like it is fighting you; showing no target would make the drop a
      // surprise. Both together is what reads as "direct manipulation".
      Object.assign(entry.root.style, {
        left: `${d.origin.x * stepX + dx}px`,
        top: `${d.origin.y * stepY + dy}px`,
      });
      item.x = Math.max(0, Math.min(COLS - item.w, Math.round(d.origin.x + dx / stepX)));
      item.y = Math.max(0, Math.round(d.origin.y + dy / stepY));
    } else {
      item.w = Math.max(2, Math.min(COLS - item.x, Math.round(d.origin.w + dx / stepX)));
      item.h = Math.max(2, Math.round(d.origin.h + dy / stepY));
      this.place(entry.root, item);
    }

    compact(this.items, d.id);
    this.place(this.placeholder, item);
    this.layout();
  }

  onPointerUp(ev) {
    const d = this.drag;
    if (!d) return;
    const entry = this.views.get(d.id);
    entry.root.classList.remove('dragging');
    this.placeholder.hidden = true;
    this.drag = null;
    compact(this.items);
    this.layout();
    this.changed();
  }

  destroy() {
    window.removeEventListener('resize', this._onResize);
    this.clear();
  }
}
