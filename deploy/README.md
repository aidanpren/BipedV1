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

2. **`leg_controller` DRIVES THE LEGS at startup.** It arms in **POSITION**
   mode and ramps to `home_position`
   ([leg_controller.py:140-145](../src/robot_base/robot_base/leg_controller.py#L140-L145)),
   and it does not consult the mode either. `home_position` is `0.0` — the
   retracted hard stop. So a boot with the legs extended is **powered motion at
   power-on, with nobody's hand near the cutoff.**

   Today that cannot happen, because the left hip (node 3) is off the CAN bus,
   so `establish_zero` times out and the node **refuses to arm**. That is safety
   *by accident*. The moment backlog item 1 is fixed, enabling this unit at boot
   becomes a leg that retracts as soon as the Pi finishes booting.

**Therefore:**

- Enabling `biped-can` at boot is safe now and stays safe. It creates a network
  interface. Nothing moves.
- Enabling `biped-stack` at boot is safe **only while the left hip is off the
  bus**. Before you put node 3 back, `leg_controller` needs to gate `arm()` on
  the mode the same way `balance_controller` gates torque — see backlog item 7
  in `HANDOFF.md`.
- Until then: enable `biped-can`, and *start `biped-stack` by hand* until you
  have watched it come up a few times.

The rest of the safety doctrine is unchanged and still applies:
`mode_manager` boots into `DISABLED`, and **DISABLED with weight on the legs
means the legs COLLAPSE** (position mode, see the `leg-position-hold` notes).

---

## Layout

```
deploy/
├── install.sh                  run ON THE PI with sudo; idempotent
├── etc/default/biped           config template -> /etc/default/biped
├── systemd/
│   ├── biped-can.service       -> /etc/systemd/system/
│   └── biped-stack.service     -> /etc/systemd/system/  (templated)
└── bin/                        -> /usr/local/lib/biped/
    ├── biped-can-cleanup.sh    zombie killer (ExecStartPre + ExecStopPost)
    ├── biped-can-start.sh      resolve device, exec slcand
    ├── biped-can-linkup.sh     wait for netdev, ip link set up
    └── biped-stack.sh          source ROS, exec ros2 launch
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
candump can0                        # expect heartbeats 001 021 041 (061 absent)

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
| `candump can0` | heartbeat frames `001`, `021`, `041` — **`061` will be absent** (left hip is off the bus, backlog 1) |
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

**Rig:** Robot **on a stand**, legs collapsed onto the retracted stop. Motors
powered. Read the ⚠️ section at the top of this file first — this test arms the
wheel axes at 0 Nm, and would drive the legs if the left hip were on the bus.

**Abort:** `sudo systemctl stop biped-stack`. `ros2 launch` gets SIGINT and
shuts its nodes down in order; `odrive_bridge`'s own shutdown sets both wheels
to 0 Nm and `IDLE`. If that is not fast enough, the physical power cut — know
where it is before you start.

**Prerequisites:** TEST 7 green. Workspace built **on the Pi** as your own user
(`colcon build --symlink-install`, not under sudo, or root ends up owning
`build/`). Left hip still off the CAN bus, or `leg_controller` gated on mode.

### Commands

```bash
# terminal A — start it by hand and watch every node report in
sudo systemctl start biped-stack
journalctl -fu biped-stack

# expect, in order: odrive_bridge "armed left/right in torque mode",
# imu_node publishing, leg_controller REFUSING TO ARM (left hip absent),
# mode_manager "mode -> disabled", rosbridge on 9090, http.server on 8000.

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
| journal | `armed left/right in torque mode`, `mode -> disabled`, and `leg_controller` refusing to arm |
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
the runbook's PRE-FLIGHT section can point here instead. Next: piece **D**, the
Pi hotspot — and note it takes the radio, so the Pi cannot be on your home WiFi
and be an access point on the same interface at the same time.

---

## What this does NOT do

- **Piece C, IMU I²C → UART.** Soldering iron. Still required before the first
  genuinely untethered drive.
- **Piece D, the hotspot.** NetworkManager AP mode; not written yet.
- **Piece E, web teleop.** The deadman-button control on the dashboard.
- **Gate `leg_controller` on mode.** See the ⚠️ section — this is now on the
  critical path *before* the left hip goes back on the bus.
