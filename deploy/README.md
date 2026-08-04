# deploy/ — the robot starts itself

_Added 2026-08-01. Pieces **A** and **B** of the standalone-operation milestone
in `HANDOFF.md`._

Everything here is **non-ROS production deployment**, in the same spirit that
`tools/` holds non-ROS dev tools. It exists to answer one question: what has to
be true for the robot to come up on its own when you flip the switch, with no
laptop, no `ssh`, and no six terminals.

Two systemd units:

| unit | what it does | runs as |
|---|---|---|
| `biped-can.service` | CANable → `slcand` → `can0`, up at 500 kbps | root |
| `biped-stack.service` | `ros2 launch robot_bringup real.launch.py` | your user |
| `biped-wifi.service` | at boot: join home WiFi, or become an access point | root |
| `biped-wifi.timer` | optional: re-decide periodically if WiFi drops | root |

---

## ⚠️ READ THIS BEFORE YOU `systemctl enable` ANYTHING

**Enabling `biped-stack` means the motor stack runs on power-on, before a human
has looked at the robot.** Here is exactly what that energizes, verified in the
source rather than assumed:

1. **`odrive_bridge` arms both wheels unconditionally.** `arm()` is called from
   `__init__` ([odrive_bridge.py:77](../src/robot_base/robot_base/odrive_bridge.py#L77)),
   with no reference to the mode at all. Both wheel axes enter
   `CLOSED_LOOP_CONTROL` in **torque** mode the moment the node starts.
   This is *acceptable*: `balance_controller` publishes `[0.0, 0.0]` whenever
   the mode is `disabled`
   ([balance_controller.py:99](../src/robot_base/robot_base/balance_controller.py#L99)),
   and a torque-mode axis at 0 Nm is limp — the wheels spin freely by hand.
   Energized, but not moving and not holding.

2. **`leg_controller` DRIVES THE LEGS at startup** — but **it is no longer
   launched.** It arms in POSITION mode and ramps to `home_position`
   ([leg_controller.py:140-145](../src/robot_base/robot_base/leg_controller.py#L140-L145))
   without consulting the mode, so running it means powered leg motion at every
   boot. Since 2026-08-02 `real.launch.py` declares `legs` with a default of
   **false**, and the node is not started at all.

   Not launching the node is a stronger guarantee than any check inside it:
   there is no process on the bus to arm the hips. An earlier draft of this
   file instead claimed the hazard was mitigated because the left hip was off
   the CAN bus — that was wrong (the hip was never broken, just unplugged for
   one test), and "a cable happens to be out" was never a safety mechanism.

**Therefore, as the stack ships today:**

- `biped-can` at boot: **safe.** It creates a network interface. Nothing moves.
- `biped-stack` at boot: **safe to enable.** With `legs:=false` the only thing
  that energizes is the two wheel axes at 0 Nm, which is limp — and
  `mode_manager` boots into DISABLED. Nothing moves on power-on.
- **`legs:=true` is the dangerous flag now**, and it carries the whole hazard.
  Do not set it in the service, and do not use it for leg bring-up — bring the
  legs up ALONE on a stand (HARDWARE_BRINGUP.md), and gate `arm()` on mode
  (backlog 7) before the legs ever join the balance stack.

The rest of the safety doctrine is unchanged and still applies:
`mode_manager` boots into `DISABLED`, and **DISABLED with weight on the legs
means the legs COLLAPSE** (position mode, see the `leg-position-hold` notes).

---

## Layout

```
deploy/
├── install.sh                  run ON THE PI with sudo; idempotent
├── etc/default/biped           config template -> /etc/default/biped
├── etc/sudoers.d/biped-dashboard   -> /etc/sudoers.d/  (templated)
├── systemd/
│   ├── biped-can.service       -> /etc/systemd/system/
│   ├── biped-stack.service     -> /etc/systemd/system/  (templated)
│   ├── biped-wifi.service      -> /etc/systemd/system/
│   └── biped-wifi.timer        -> /etc/systemd/system/
└── bin/                        -> /usr/local/lib/biped/
    ├── biped-can-cleanup.sh    zombie killer (ExecStartPre + ExecStopPost)
    ├── biped-can-start.sh      resolve device, exec slcand
    ├── biped-can-linkup.sh     wait for netdev, ip link set up
    ├── biped-stack.sh          source ROS, exec ros2 launch
    ├── biped-wifi-setup.sh     ONE-TIME: create the AP profile (asks for a
    │                           passphrase; never stored in this repo)
    └── biped-wifi-mode.sh      status | auto | home | ap
```

**All tuning lives in `/etc/default/biped`**, edited in place on the Pi. The
units read it via `EnvironmentFile=`; no rebuild, no reinstall, just
`systemctl restart`. The one exception is `User=` / `WorkingDirectory=`, which
systemd resolves *before* it reads the environment file and which therefore
have to be baked into the unit — that is why `biped-stack.service` is a
template with `@BIPED_USER@` in it and `install.sh` runs `sed` over it.

## Install

```bash
# on the Pi, after a git pull and a colcon build
sudo ~/BipedV1/deploy/install.sh
```

Install-only by default — it does not enable anything, so running it changes
nothing about your next boot. Re-run it after every pull; it will not clobber
an edited `/etc/default/biped`.

### The sudoers drop-in (added 2026-08-03)

`install.sh` also writes `/etc/sudoers.d/biped-dashboard`, which lets the stack
user run `systemctl restart biped-stack.service` without a password. That one
line is what makes the dashboard's **Restart** button work; nothing else in the
stack needs it.

It is validated with `visudo -cf` into a temp file *before* being installed. A
syntax error anywhere under `/etc/sudoers.d` makes `sudo` refuse to run at all —
including the `sudo` you would need to delete the broken file — so on a headless
Pi an unchecked write here is a reinstall. If validation fails, install.sh warns
and skips it: the Restart button then fails with a sudo error and everything
else keeps working.

**It grants five literal command lines and no wildcards.** That is deliberate
and worth understanding: rosbridge binds `0.0.0.0` with **no authentication**,
because that is what lets a phone connect. So the dashboard's WebSocket is
effectively open to whatever network the Pi is on, and this file decides what
"open" is allowed to do. `NOPASSWD: ALL` would turn a dashboard into
unauthenticated root. A wildcard like `systemctl restart *` would look
equivalent and would permit restarting *any* unit on the system.

Treat the Pi's network accordingly: the AP has a passphrase, and the dashboard
is not something to expose to an untrusted LAN.

---

## TEST 7 — CAN comes up from systemd

**Proves:** `biped-can.service` brings `can0` up at 500 kbps from a cold boot,
with no human running `slcand`, and leaves no zombie interface behind when
stopped or restarted.

**Rig:** Robot **on a stand**, or motors powered but the stack NOT running.
Nothing in this test commands torque, but the ODrives are on the bus and
`candump` proves it. Legs collapsed / resting on the stop. No hands needed.

**Abort:** `sudo systemctl stop biped-can`. The interface goes away; the motors
were never commanded, so nothing moves. If the CANable wedges, unplug its USB —
that is also the thing this test is designed to recover from cleanly.

**Prerequisites:** `can-utils` installed. Motors powered and previously
confirmed on the bus (HARDWARE_BRINGUP.md). `deploy/install.sh` has run.

### Commands

```bash
# terminal A — install and start by hand, watching it happen
sudo ~/BipedV1/deploy/install.sh
sudo systemctl start biped-can
systemctl status biped-can --no-pager

# what the device actually resolved to. This is the whole point of not
# hardcoding /dev/ttyACM0 — read what it picked, don't assume.
journalctl -u biped-can -n 30 --no-pager | grep slcand

# terminal A — the interface itself
ip -brief link show can0            # expect: can0  UP
ip -details link show can0          # expect: slcan, and NO <NOARP> without carrier

# terminal B — the bus is real, not just the netdev
candump can0                        # expect all four: 001 021 041 061

# terminal A — the zombie test. THIS is the part that has bitten before.
sudo systemctl restart biped-can
sudo systemctl restart biped-can
sudo systemctl restart biped-can
ip -brief link show can0            # still exactly ONE can0, still UP
ip -brief link show | grep -c '^can'   # expect: 1  (not 2, not 3)

# terminal A — stop leaves nothing behind
sudo systemctl stop biped-can
ip link show can0                   # expect: "Device does not exist"

# terminal A — the reboot, which is the actual claim
sudo systemctl enable biped-can
sudo reboot
# ... after it comes back:
systemctl status biped-can --no-pager
ip -brief link show can0            # expect: can0  UP, with nobody having typed anything
candump can0
```

### Expected

| Action | Expected |
|---|---|
| `systemctl start biped-can` | active (running) within ~2 s |
| `journalctl … grep slcand` | one line naming a `/dev/serial/by-id/...` path, **not** `/dev/ttyACM0` |
| `ip -brief link show can0` | `can0  UP` |
| `candump can0` | **all four** heartbeat frames: `001`, `021`, `041`, `061`. A missing `061` is a left-hip cable, not a fault — check the connector |
| three `restart`s in a row | still exactly one `can*` interface, still UP |
| `systemctl stop` | `can0` gone entirely — "Device does not exist" |
| reboot | `can0` UP with no human intervention |

### Failure modes

| Symptom | Cause | Fix |
|---|---|---|
| unit restart-loops, journal shows slcand starting then exiting instantly | this `slcand` build lacks `-F`, so it daemonized and systemd saw the main process exit | `slcand -h` and check; if truly absent, switch the unit to `Type=forking` + `PIDFile=` |
| `ERROR: no CDC-ACM serial device found` | CANable unplugged, or wedged from a previous teardown | `lsusb`; power-cycle the CANable's USB — it genuinely wedges after many teardowns |
| `ERROR: N CDC-ACM devices present` | something else (a Pico, an Arduino) is also on USB | pin `CAN_DEVICE=` in `/etc/default/biped` to the right by-id path |
| `can0` exists but `candump` is silent, no errors | **not a systemd problem.** Swapped CAN-H/L, or over-termination. slcan does not surface bus errors to the kernel, so silence and "no errors" look identical | check H/L orientation first; multimeter H↔L with power OFF must read ~60 Ω |
| `ip link set up` times out; `dmesg` says `failed to send bitrate command` | zombie netdev — a carrier-less `can0` from a previous run | this is what `biped-can-cleanup.sh` prevents; if it still happens the cleanup did not run, check `journalctl -u biped-can` for the ExecStartPre line |
| two `can*` interfaces after restarts | cleanup ran in the wrong order, or `pkill` did not match | `pkill -x slcand` matches the process *name*; confirm nothing else is spawning slcand |
| works by hand, fails on reboot | USB had not enumerated yet | raise `CAN_DEVICE_WAIT` in `/etc/default/biped` |

### Why this works

`slcand` does three things in order: it opens the CANable's tty and attaches
the `N_SLCAN` line discipline to that file descriptor; the kernel then
registers a netdev against **that line discipline**; and slcand renames it from
`slcan0` to `can0` with the `SIOCSIFNAME` ioctl.

The netdev's lifetime is bound to the line discipline on that fd, **not** to
the daemon's process. That single fact generates every failure this unit has to
defend against. Kill slcand with anything other than a clean shutdown — SIGKILL,
a crash, a yanked USB cable — and the tty disappears while the netdev stays
registered: a `can0` with no carrier and nothing behind it. Bringing that up
makes the slcan driver write `C\rS6\r` to a serial port that is gone, so netlink
times out. Starting a fresh slcand fails at the rename, because the corpse still
owns the name. And interface names are a kernel-global namespace, which is why
"just use `can1`" walks around the body instead of removing it — that is how
this project's bench notes drifted from `can0` to `can1` to `can2`.

Hence the order in `biped-can-cleanup.sh`: **daemon first, netdev second.**
Reverse it and a surviving slcand simply recreates the interface, which is why a
bare `ip link delete can0` appears to do nothing. And hence `ExecStopPost=`
running the same cleanup on the way out: slcand's `-c` flag sends the adapter's
close command only on a *clean* exit, so every unclean path needs the netdev
swept up by something else.

Device naming is the second half. `/dev/ttyACM0` is assigned in enumeration
order, so a replug or a second USB serial device silently moves the CANable and
slcand attaches a CAN line discipline to whatever else landed there — producing
a `can0` that exists, comes up, and carries nothing. `/dev/serial/by-id/` is
built by systemd's stock udev rules from the USB descriptor and serial number,
so it is stable across replug and reboot. Using it means **no custom udev rule
of our own**, and therefore no rule that can quietly stop matching after a
firmware update changes a VID/PID.

The `ExecStartPost=` placement is what makes the *next* test work. systemd holds
a unit in `activating` until every `ExecStartPost` has returned, and units
ordered `After=` wait for activation to finish. So putting `ip link set up` in
`ExecStartPost` means `biped-stack.service`'s `After=biped-can.service` promises
"after `can0` is **up**", not merely "after slcand was launched".

### Pass criteria

- `can0` reports `UP` and `candump` shows heartbeats `001`, `021`, `041`.
- Three consecutive `systemctl restart`s leave **exactly one** `can*`
  interface, still UP. (`ip -brief link show | grep -c '^can'` returns `1`.)
- `systemctl stop` leaves **no** `can*` interface.
- After `enable` + `reboot`, `can0` is UP and carrying heartbeats with nothing
  typed by a human. Hold it 60 s to be sure it is not about to restart-loop.

### On pass

Record the resolved `/dev/serial/by-id/...` path in this file (below), tick
backlog item 3 in `HANDOFF.md`, and go to TEST 8.

**Resolved CANable path on this robot:** _(fill in after the first run)_

---

## TEST 8 — the ROS stack comes up from systemd

**Proves:** `biped-stack.service` launches the full `real.launch.py` on boot,
into `DISABLED`, with the dashboard reachable — replacing the six-terminal
workflow.

**Rig:** Robot **on a stand**, wheels clear of everything, motors powered.
Nothing should move during this test: `legs` defaults to false so
`leg_controller` never starts, and the only thing that energizes is the two
wheel axes at 0 Nm, which is limp. **If a leg moves, stop and find out why** —
something is passing `legs:=true`.

**Abort:** `sudo systemctl stop biped-stack`. `ros2 launch` gets SIGINT and
shuts its nodes down in order; `odrive_bridge`'s shutdown sets both wheels to
0 Nm and `IDLE`. Know where the physical power cut is anyway.

**Prerequisites:** TEST 7 green. Workspace built **on the Pi** as your own user
(`colcon build --symlink-install`, not under sudo, or root ends up owning
`build/`). `BIPED_LAUNCH_ARGS` in `/etc/default/biped` must NOT contain
`legs:=true` — check it.

### Commands

```bash
# terminal A — start it by hand and watch every node report in
sudo systemctl start biped-stack
journalctl -fu biped-stack

# expect, in order: odrive_bridge "armed left/right in torque mode",
# imu_node publishing, mode_manager "mode -> disabled", rosbridge on 9090,
# http.server on 8000. NO leg_controller and NO leg_joy — legs defaults false.

# terminal B — the stack is real, from a NORMAL shell (this is the check that
# the systemd environment matches your interactive one)
source /opt/ros/jazzy/setup.bash
source ~/BipedV1/install/setup.bash
ros2 node list
ros2 topic echo /mode --once            # expect: data: 'disabled'

# terminal B — VERIFY AT RUNTIME, never from the yaml. Same rule as every
# other test in this project.
ros2 param get /balance_controller a1           # expect -0.07
ros2 param get /balance_controller a2           # expect -0.15
ros2 param get /balance_controller pitch_trim   # expect 0.08
ros2 param get /imu_node driver                 # expect i2c (until piece C)
ros2 param get /odrive_bridge can_channel       # expect can0

# terminal B — torque really is zero while disabled
ros2 topic echo /wheel_effort_controller/commands --once   # expect [0.0, 0.0]
# and confirm physically: both wheels spin freely by hand

# from the LAPTOP or a phone on the same network
# http://biped.local:8000/   -> dashboard connects, mode shows disabled

# terminal A — restart cleanly, twice; no leaked processes
sudo systemctl restart biped-stack
sudo systemctl restart biped-stack
pgrep -af 'http.server|rosbridge|odrive_bridge' | wc -l    # expect one set, not three

# terminal A — the reboot
sudo systemctl enable biped-stack
sudo reboot
# ... after it comes back, WITHOUT ssh-ing in first:
#     open http://biped.local:8000/ from the laptop or phone
```

### Expected

| Action | Expected |
|---|---|
| `systemctl start biped-stack` | active (running); journal fills immediately (not after a delay — that would be `PYTHONUNBUFFERED` missing) |
| journal | `armed left/right in torque mode`, `mode -> disabled`, and **no mention of `leg_controller` or `leg_joy`** |
| the legs | **do not move at all.** Any leg motion means `legs:=true` leaked in from somewhere — stop and find it |
| `ros2 topic echo /mode --once` | `data: 'disabled'` |
| wheels by hand | spin freely — armed, but 0 Nm in torque mode is limp |
| `ros2 param get …` | the values in `real.yaml`, proving the service loaded the same params your terminal runs did |
| dashboard from a phone | connects, shows `disabled` |
| two restarts | exactly one set of processes; no orphaned `http.server` |
| reboot | dashboard reachable with no `ssh`, robot still `DISABLED` |

### Failure modes

| Symptom | Cause | Fix |
|---|---|---|
| journal empty for a minute then a burst | `PYTHONUNBUFFERED=1` missing — Python block-buffers to a pipe | it is in the unit; check it survived a hand-edit |
| `workspace overlay not found` | not built on the Pi, or built as root | build as your own user |
| unit fails instantly, error names a path in `/opt/ros` | `set -u` vs ROS's setup scripts | `biped-stack.sh` wraps sourcing in `set +u`; check it survived an edit |
| nodes start but `ros2 node list` from your shell is empty | `ROS_DOMAIN_ID` mismatch between the unit's environment and your shell | `systemctl show biped-stack -p Environment`; align `/etc/default/biped` |
| dashboard connects but mode never arrives | rosbridge bound the wrong port — a generic launch arg leaking into an include | known trap; `grep -l rosbridge_websocket /tmp/launch_params_*` then read it |
| `Address already in use` on 8000 or 9090 | an orphaned `http.server` from a previous run | `KillMode=mixed` should reap it; check `systemctl status` for leftover cgroup members |
| joystick ignored, no error | `joy_node` cannot open `/dev/input/js*` under the service account | `SupplementaryGroups=input` is in the unit; confirm with `systemd-run --uid=… ls /dev/input` |
| stack starts before CAN, `odrive_bridge` errors | `After=` not honoured | check `biped-can` reached `active`, not `activating` — the `ExecStartPost` is what gates it |
| unit gives up after 5 rapid failures | `StartLimitBurst` — working as designed | fix the real error in the journal, then `systemctl reset-failed biped-stack` |

### Why this works

The wrapper script exists because `ExecStart=` is not a shell — systemd execs a
binary directly, so there is nowhere to `source` from. Everything a ROS node
needs (`AMENT_PREFIX_PATH`, `PYTHONPATH`, the library path, the overlay) is set
up by shell scripts, so a shell has to run somewhere; it runs in
`biped-stack.sh` and then gets out of the way with `exec`, so the process
systemd supervises is `ros2 launch` itself rather than a shell wrapping it.

`set +u` around the sourcing is required, not sloppy. ROS's `setup.bash` reads
variables that are legitimately unset on a fresh boot (`AMENT_TRACE_SETUP_FILES`,
`COLCON_TRACE`). Under `set -u` the first one aborts the script, and because the
abort happens inside a sourced file the message names a path in `/opt/ros` —
which reads exactly like a broken ROS install and sends you debugging the wrong
thing.

The dependency direction is the safety-relevant design choice. `Wants=` rather
than `Requires=` means a failed CAN bus still lets the dashboard, `imu_node`
and the journal come up — which is *how you find out why CAN failed*.
`Requires=` would take the diagnostics down with the fault. And deliberately
**not** `BindsTo=`: that would stop this unit whenever `biped-can` stopped, so a
momentary USB glitch on the CANable would kill a running balance loop and drop
the robot. Losing CAN must degrade to "the bridge complains", never to "the
controller is gone". This is the same principle as CLAUDE.md's safety
invariant — signal loss zeros the reference, it never cuts the controller.

Shutdown uses `KillSignal=SIGINT` because `ros2 launch` installs a SIGINT
handler that shuts its children down in order and is far less graceful on
SIGTERM. `KillMode=mixed` sends that signal to the main process only, so launch
runs its own teardown, then SIGKILLs whatever is still in the cgroup after
`TimeoutStopSec` — which is what reaps stragglers like the dashboard's
`python3 -m http.server`.

Running the stack as your own user rather than root is not ceremony: nothing in
it needs privilege (opening an `AF_CAN` socket is unprivileged, and the netdev
was already created by the root-owned `biped-can` unit), and running as your
user means the service reads the same built workspace and writes the same
`~/.ros/log` you read interactively. A root-owned service would quietly build a
second, divergent environment — which is the failure mode where "it works in my
terminal" and "it fails in the service" are both true.

### Pass criteria

- Every node in `real.launch.py` reports in, and `/mode` latches `disabled`.
- The five `ros2 param get` checks return the `real.yaml` values.
- Both wheels spin freely by hand while `DISABLED`, and
  `/wheel_effort_controller/commands` is `[0.0, 0.0]`.
- The dashboard loads from a phone and shows `disabled`.
- Two consecutive restarts leave exactly one set of processes.
- After `enable` + reboot, the dashboard is reachable **without ssh**, the robot
  is `DISABLED`, and it holds that for 5 minutes without the unit restarting.

### On pass

Tick backlog items A and B in `HANDOFF.md`. The six-terminal workflow is dead;
the runbook's PRE-FLIGHT section can point here instead.

Next is **not** the hotspot. It is backlog item 7 (gate `leg_controller`'s
`arm()` on mode), which is what makes `systemctl enable biped-stack` safe, and
then piece **E** — a DS4 button that switches to TELEOP. Together those are the
whole remaining gap between here and *turn on the robot, turn on the pad, press
a button, drive*.

---

## TEST 9 — the DS4 switches modes with no laptop

**Proves:** a tap on the pad enters TELEOP and the L1+L2 combo returns to
DISABLED, with nothing but the robot and the controller.

**Rig:** Robot **on a stand**, wheels clear, motors powered, legs not involved
(`legs:=false`). **Entering TELEOP starts the balance loop — torque comes on at
that instant.** That is the point of the test, and it is why this is on a
stand. Hand near the power cut for the first press.

**Abort:** the combo you are testing (L1+L2) → DISABLED → torque cut. If the
combo is the thing that is broken, use the physical power cut. Do not rely on
`Ctrl-C` — you are testing whether the pad works, so assume it does not.

**Prerequisites:** TEST 8 green. DS4 **paired to the Pi over Bluetooth and
reconnecting on its own** — that is a separate piece of work and if it is not
done, nothing below can pass. Button indices measured (step 0).

### Commands

```bash
# ── step 0: MEASURE the button indices. Not a test, a measurement. ──────────
# DS4 numbering differs between the SDL and joydev backends, and a wrong index
# does not error — the button just never fires.
ros2 run joy joy_node                    # terminal A
python3 ~/BipedV1/tools/joy_probe.py     # terminal B, then press things

# press Options, L1, L2. Write down the three numbers.
# If L2 shows up ONLY as an axis, pick a different digital button for the
# combo rather than adding a threshold.

# put them in real.yaml under mode_manager:, then restart the stack.
# NO colcon build — real.yaml is loaded from SOURCE via --params-file.
sudo systemctl restart biped-stack

# ── step 1: verify at RUNTIME, before trusting anything ─────────────────────
ros2 param get /mode_manager teleop_button      # expect your measured number
ros2 param get /mode_manager disable_buttons    # expect your measured pair

# ── step 2: watch the mode while you press ──────────────────────────────────
ros2 topic echo /mode                    # terminal C, leave it running

# tap Options            -> 'teleop'   AND THE ROBOT STARTS BALANCING
# hold Options           -> nothing more (edge, not level)
# press L1 alone         -> nothing
# press L2 alone         -> nothing
# press L1+L2 together   -> 'disabled'
# press L2 then L1       -> 'disabled'  (order must not matter)

# ── step 3: the interlock ───────────────────────────────────────────────────
# turn the PAD OFF, wait 2 s, then from a shell:
ros2 service call /set_mode robot_interfaces/srv/SetMode "{mode: teleop}"
# expect: success: false, message: 'no controller present'
```

### Expected

| Action | Expected |
|---|---|
| tap teleop button | `/mode` → `teleop`, **wheels start actively balancing** |
| keep holding it | nothing further — one transition per press, not 20/s |
| release, tap again | fires again |
| L1 alone / L2 alone | nothing |
| L1+L2, either order | `/mode` → `disabled`, torque cut |
| pad off, then `/set_mode teleop` | refused, `no controller present` |
| pad off **while in TELEOP** | stays in TELEOP, **keeps balancing**, velocity zeroed in 0.5 s |

### Failure modes

| Symptom | Cause | Fix |
|---|---|---|
| nothing happens on any press | wrong indices, or `/joy` not arriving at all | `ros2 topic hz /joy` first — no messages is a Bluetooth/`joy_node` problem, not a mode problem |
| warn: "`/joy` has N buttons but indices need M" | pad reporting a different layout | re-measure with `joy_probe.py` |
| mode flickers rapidly | edge detection defeated — should be impossible, tested in `tools/test_mode_buttons.py` | re-run that test; if it passes, suspect two `mode_manager` instances |
| tap enters TELEOP but robot does not balance | not a mode problem — IMU or `odrive_bridge` | check `/imu` and `/joint_states` |
| pad turns off and the robot DISABLES itself | **serious** — violates the CLAUDE.md invariant | signal loss must never auto-disable; check nothing added that path |

### Why this works

`mode_manager` already subscribed to `/joy` before this feature existed — it
needed the arrival of a message as evidence a controller is alive, for the
interlock that stops the dashboard arming a robot with no pad. This feature
reads the *content* of a message it was already receiving.

Both entry points, the buttons and the `SetMode` service, go through one
`request_mode()`. That matters: the interlock used to live in the service
callback, so a button handler calling `set_mode()` directly would have skipped
it, and the robot would have had two different rule sets depending on who
asked. One policy point means the pad and the dashboard are equally governed.

Edge detection is not a nicety. `joy_node` autorepeats at 20 Hz, so a held
button is a continuous stream of identical messages; acting on the level would
fire the transition twenty times a second. The combo is edge-detected as a
unit — it fires when *all* its buttons become held — which is why the order you
press them in cannot matter.

The asymmetry between the two gestures is deliberate. Entering TELEOP is one
tap because it is the thing you do constantly. Leaving it cuts torque and drops
the robot, so it takes two fingers: `DISABLED` is not a safe state, and the one
thing CLAUDE.md is absolute about is that it must only ever arrive from an
explicit command.

### Pass criteria

- Every row of the Expected table observed, including the two negative ones
  (single combo button does nothing; held button does not re-fire).
- The interlock refuses TELEOP with the pad off, and says why.
- Pad switched off mid-TELEOP: robot **keeps balancing** and stays in TELEOP.
- Ten consecutive tap/combo cycles with no missed or doubled transition.

### On pass

Write the measured indices into `real.yaml` **and** `sim.yaml`, and record them
here. Piece E is done, and the goal — *turn on, pad on, press a button, drive* —
is reachable end to end.

**Measured button indices on this robot:** _(fill in)_ teleop `___`, disable `___`+`___`

---

## TEST 10 — the hotspot

**Proves:** with no home WiFi in range the Pi brings up its own WPA2 access
point at boot, and the dashboard is reachable from a phone with no other
network involved.

**Rig:** Robot powered, on a stand or on the bench. Nothing moves in this test.
Motors may be off entirely — this is a networking test.

**Abort:** nothing to abort; no motion. To recover a lost connection, use
Ethernet or a console: `sudo /usr/local/lib/biped/biped-wifi-mode.sh home`.

**Prerequisites:** `deploy/install.sh` has run. NetworkManager in use.
`iw` installed (for the capability check). A phone or laptop to join with.

### Commands

```bash
# ⚠️ RUN THIS OVER ETHERNET OR A CONSOLE, NOT OVER WIFI.
# Activating the AP reconfigures the very interface an ssh-over-wifi session is
# riding on. Same shape of mistake as pkill'ing your own shell.

# ── one-time setup. Asks for a WPA2 passphrase; it is stored by
#    NetworkManager (root-only, 0600) and never written into the repo.
sudo /usr/local/lib/biped/biped-wifi-setup.sh

# ── manual activation first, before anything is automatic
sudo /usr/local/lib/biped/biped-wifi-mode.sh ap
/usr/local/lib/biped/biped-wifi-mode.sh status   # expect role: ACCESS POINT

# from a PHONE: join SSID 'BipedV1', then open
#   http://10.42.0.1:8000/
# the dashboard should connect and show the latched mode.

# ── back to home WiFi, deliberately
sudo /usr/local/lib/biped/biped-wifi-mode.sh home
/usr/local/lib/biped/biped-wifi-mode.sh status   # expect role: client

# ── now the automatic decision
sudo systemctl enable --now biped-wifi.service
journalctl -u biped-wifi -n 20 --no-pager        # expect "already on '<home>'"

# ── the real test: boot with home WiFi UNREACHABLE.
# Take the robot out of range, or power your home AP down, then reboot.
sudo reboot
# after it comes back, from a phone: join 'BipedV1', open the dashboard.
```

### Expected

| Action | Expected |
|---|---|
| `biped-wifi-setup.sh` | reports AP-mode support and a set regulatory domain; creates `biped-ap` |
| `... mode.sh ap` | `role: ACCESS POINT`, address `10.42.0.1` on wlan0 |
| phone joins `BipedV1` | gets a `10.42.0.x` DHCP lease |
| `http://10.42.0.1:8000/` | dashboard loads and shows the latched mode |
| `... mode.sh home` | rejoins home WiFi; AP gone |
| boot **in** home range | journal: "already on '<home>'" — AP never starts |
| boot **out of** home range | journal: "no home WiFi — falling back"; AP up |

### Failure modes

| Symptom | Cause | Fix |
|---|---|---|
| you lose your ssh session at `mode.sh ap` | you ran it over WiFi | expected; rejoin via the AP. Use Ethernet next time |
| `nmcli con up` fails with a generic activation error | radio cannot do AP mode, or regulatory domain unset | `iw list` for AP support; `sudo iw reg set <CC>` |
| AP appears but nobody can get an address | `ipv4.method` not `shared` | `nmcli con show biped-ap \| grep ipv4.method` |
| AP up at boot even at home | scan ran before NM settled | `nm-online` in the unit covers this; raise its timeout |
| interface flaps between AP and home | two deciders | the AP profile must be `autoconnect no` — verify it |
| dashboard loads, mode never arrives | rosbridge, not WiFi | it binds `0.0.0.0`; check port 9090 is reachable from the phone |

### Why this works

The Pi 5 has one radio, and it cannot usefully be an access point and a
home-WiFi client at once. So this is an either/or, and something has to choose.
That something is `biped-wifi-mode.sh`, and it is the **only** chooser — the AP
profile is created `autoconnect no` precisely so NetworkManager does not hold a
competing opinion. Two deciders would produce a flapping interface with no
single cause to point at.

The policy is asymmetric on purpose. Falling *to* the AP is automatic, because
that is the case where you have no other way to reach the robot. Returning to
home WiFi is **not** automatic by default, for a reason that is more than
caution: most drivers cannot scan while operating as an AP on the same radio,
so noticing that home WiFi came back is not passive observation — it requires
tearing the AP down to look. That outage would land exactly when you are in the
field with a phone on the AP. Staying on the AP too long costs you a `git pull`
you will notice; dropping it costs you telemetry while the robot is live.

WPA2 is a safety control here, not hygiene. `rosbridge` binds `0.0.0.0:9090`
and exposes `/set_mode`. On an open AP, anyone in range could put the robot
into TELEOP and energize the motors. The passphrase is what stops a stranger
arming your robot.

Nothing in the ROS stack is ordered against this unit. Both `rosbridge` and the
dashboard's `http.server` bind `0.0.0.0`, so they neither know nor care which
interface appears when — which is what makes the hotspot independently
testable, and keeps a WiFi scan out of the boot path for balancing.

### Pass criteria

- Booting **out of** home-WiFi range brings the AP up unattended, and a phone
  can load the dashboard at `http://10.42.0.1:8000/` and see the live mode.
- Booting **in** home-WiFi range joins home and never starts the AP.
- `biped-wifi-mode.sh status` reports the truth in both cases.
- The AP requires the passphrase (confirm a wrong one is rejected).

### On pass

Record the SSID here. Piece D is done. Note that while on the AP the Pi has no
internet, so code reaches it by `scp`/`rsync` from a laptop joined to the AP
rather than by `git pull`.

**AP SSID on this robot:** _(fill in)_

---

## What this does NOT do

- **Anything to do with the legs.** Deferred entirely (2026-08-02) —
  `legs:=false`. Gating `arm()` on mode (backlog 7) is still required, but it
  is now a prerequisite for turning the legs ON, not a blocker on shipping the
  wheeled robot.
- **Piece E, the DS4 mode-switch button** (backlog 9). `mode_manager` boots into
  DISABLED and the only way out today is the `SetMode` service — a laptop or a
  phone. Until a pad button can do it, the robot is not standalone however well
  systemd works.
- **Piece C, IMU I²C → UART.** Soldering iron. Still required before the first
  genuinely untethered drive.
- **Piece D, the hotspot.** NetworkManager AP mode; not written yet, and no
  longer on the critical path — the dashboard is optional and nothing about
  driving needs the network.

**Not planned, deliberately: web teleop.** The dashboard changes modes and shows
readouts. Driving is the physical DS4 only. An earlier plan specified an
on-screen deadman stick; that was an assumption, never a requirement, and it is
withdrawn.
