# robot_dashboard — the web dashboard

A drag-and-drop tile dashboard, in the spirit of FRC Shuffleboard: a 12-column
grid you arrange yourself, from a palette of tiles that can point at any topic,
any field, and any node's parameters.

Open it at **`http://<robot>:8000/`** from a laptop or a phone. Nothing about it
is required for the robot to run.

```
robot_dashboard/
├── dashboard_backend.py   ROS node: git/build/restart + saving tuning to YAML
├── yaml_patch.py          comment-preserving YAML value rewriter (no ROS, testable)
└── web/
    ├── index.html         the shell: top bar, grid container, modal
    ├── css/dashboard.css  palette, tile chrome, layout
    ├── js/ros.js          rosbridge: shared subscriptions, services, params, introspection
    ├── js/grid.js         the grid: drag, resize, collide, compact
    ├── js/widgets.js      TimePlot, Meter, colour slots, formatting
    ├── js/tuning.js       knob ranges, hints, and the drive-feel presets
    ├── js/tiles-core.js   generic tiles + the tile registry
    ├── js/tiles-robot.js  robot-specific tiles
    └── js/app.js          layouts, palette, settings form
```

## Three processes, and which failure looks like what

`dashboard.launch.py` starts all three. When something is wrong, this table
saves you from debugging the wrong one:

| symptom | the process that is down |
|---|---|
| page does not load at all | `http.server` (:8000) |
| page loads, top bar says "reconnecting…" | `rosbridge_websocket` (:9090) |
| everything works except **Deploy** and **Save** | `dashboard_backend` |
| tile settings show no topics or no fields | `rosapi` (starts with rosbridge) |

## Using it

**Edit** unlocks dragging. Drag a tile by its title bar, resize from the bottom
right, ⚙ for settings, ✕ to remove. Tiles fall upward into free space and push
each other down, so you cannot leave a gap or an overlap. Editing is locked by
default so a tile cannot be dragged by accident while you are driving.

Below 700 px wide the tiles stack full-width in reading order. **That is a view,
not a change** — the stored layout is always in 12-column space, so opening the
page on a phone can never flatten the layout you built on a laptop.

Layouts live in the browser's `localStorage`, so the phone and the laptop each
keep their own. **⋯ → Export/Import** moves one between them.

## The tiles

**Generic** — point them at anything. `Value`, `Plot` (up to 4 fields),
`Gauge`, `Array bars`, `Text`, `All fields`, `Raw message`, `Topic browser`,
`Service button`, `Parameters` (any node, every parameter).

**Robot** — `Mode`, `Power`, `Motors`, `Motor current plot`, `Pitch`,
`Speed tracking`, `Legs`.

**Tuning** — `Drive feel` (presets + the knobs behind them), `Balance tuning`,
`Friction compensation`.

**Deploy** — `Deploy` (pull/build/restart), `Save tuning`.

The topic and field pickers are populated from `rosapi` at runtime, so a node
you add next month is selectable without touching this package.

## Live tuning

Sliders call `set_parameters` — the *same* call `ros2 param set` makes, so
anything you can tune from a terminal you can tune from the sofa, live, while
the robot balances. Rejections are shown; a slider that appears to work but
changes nothing is the failure this deliberately makes loud.

Ranges come from a curated table in `js/tuning.js` with the reasoning in the
hint under each slider. They are **UI limits only** — they clamp what the page
can send and change nothing about the node.

Live changes die on restart. The **Save tuning** tile writes them back into
`real.yaml`, reading the values off the *running node* rather than trusting what
the page thinks it sent. `yaml_patch.py` swaps the value text in place: every
comment survives, types are preserved (a double stays `12.0`, never `12`), keys
that are not already in the file are reported rather than invented, and a `.bak`
is left beside each file.

## Deploy

Four staged buttons: **Check** (`git fetch` + what is incoming), **Pull**
(`git pull --ff-only`), **Build** (`colcon build --symlink-install`), **Restart**
(`systemctl restart biped-stack`). Output streams live and survives a page
reload.

The Pi must be on a network with internet — **not** running as its own access
point. Build and Restart are **locked unless the mode is DISABLED**, because
both kill `balance_controller` and idle the wheel axes, which on a balancing
robot means dropping it. An unknown mode counts as unsafe. Restart needs the
sudoers drop-in from `deploy/install.sh`.

## Editing the page

The installed files are symlinks into `src/`, so **editing an existing file just
needs a browser refresh**. *Adding* a file needs `colcon build` first, and a new
subdirectory also needs its own line in `setup.py` — `glob()` does not recurse,
and a missing line drops the files silently.

Serve it over http. Opening `index.html` off the filesystem fails: ES modules
are blocked on `file://`.

## Adding a tile

Add a `register({...})` block. `create(ctx)` returns `{el, title, destroy}`.

```js
register({
  type: 'mytile', title: 'My tile', group: 'Robot',
  desc: 'What it shows.', w: 4, h: 3,
  config: [{ key: 'topic', label: 'Topic', type: 'topic' }],
  create(ctx) {
    const node = el('div', { text: '—' });
    const stop = R.subscribe(ctx.cfg.topic, ctx.cfg.type, (m) => { … });
    return { el: node, title: 'My tile', destroy: stop };   // destroy is not optional
  },
});
```

**Always return `destroy`.** Subscriptions are shared and reference-counted in
`ros.js`; a tile that does not release its handler leaks it, and after a while
of rearranging you have fifty handlers on `/imu` at 100 Hz and a page that has
become mysteriously slow.

## Testing without a robot

```bash
source install/setup.bash
python3 tools/test_dashboard_backend.py     # services, safety gate, YAML save
ros2 launch robot_dashboard dashboard.launch.py
```

For live data with no hardware, see `tools/fake_odrive.py` (it answers current
and bus-voltage reads and broadcasts heartbeats, so the Power and Motors tiles
work on `vcan0`).
