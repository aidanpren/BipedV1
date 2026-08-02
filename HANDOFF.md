# Session handoff — 2026-08-01 (end of day)

Read `CLAUDE.md`, `BALANCE_RUNBOOK.md`, `TEST_FORMAT.md` first. This file is
the "where we are and what's next"; the runbook is how to test it.

---

## STATE: the robot balances, drives, and takes joystick input

All on the rewritten control law, verified on hardware today.

- TEST 3 (station keeping) — **passed** on the new law and the retuned gains
- TEST 4 (driving) — **passed**
- Joystick teleop — **working**
- Original complaint (oscillation/jerk when accelerating under joystick) —
  **resolved**

`git push` is still OUTSTANDING — commit `a50d256` is committed locally but
was never pushed (no credentials in the agent environment). **Push before
doing anything else, or the Pi pulls nothing.**

---

## What changed today

### The control law was restructured, not just retuned

The old law had two branches — driving (`pitch_target = a2*(v_f - v_ref)`) and
station keeping (`pitch_target = a1*(x - x_home) + a2*v_f`) — selected by
`abs(v_ref) > 0.05`. Three faults:

1. `v_ref` appeared ONLY in the branch test, never in the station-keeping law.
   At `scale_linear` 0.15 the threshold sat at 33% of stick travel, so the
   bottom third of the stick was silently discarded, then the law changed
   underneath you in one cycle.
2. `a2` did two contradictory jobs — damping (D term) for station keeping,
   where big is good, and velocity-loop P gain for driving, where big is
   unstable. No single value satisfied both. This was the real bug.
3. Driving threw position feedback away entirely (`x_home = x` every cycle),
   so releasing the stick snapped it back on with a step.

Replaced with ONE law, no branch, no threshold — the reference POSITION ramps
at the reference SPEED:

```
x_home += v_cmd*dt                                   (clamped to +-max_pos_error)
pitch_target = a1*(x - x_home) + a2*(v_f - v_cmd) + accel_to_lean*a_ref
```

At `v_cmd = 0` this is numerically identical to the old station-keeping law
(verified, max diff 0.00e+00), so nothing that already worked was lost.

### Gains retuned as a COUPLED pair

`a1` and `a2` are not independent knobs:

```
omega_n = sqrt(g*|a1|)          zeta = (|a2|/2) * sqrt(g/|a1|)
```

`g*|a2|` is also the driving-loop bandwidth, and a balancer is non-minimum
phase (it drives its wheels backward to lean forward), capping usable
bandwidth at roughly `sqrt(g/l_com)/4` ~ 1.6 rad/s. **The old `a2 = -0.3`
demanded 2.9 rad/s — about twice the physical ceiling.** That, not a bad gain
elsewhere, is what made acceleration surge.

`-0.09/-0.3` -> **`-0.07/-0.15`**. Halves the driving bandwidth to 1.5 rad/s
AND leaves station keeping better damped and twice as fast to return.

### Reference shaping, two stages

Stage 1 bounds acceleration (`accel_limit`). Stage 2 bounds jerk
(`jerk_tau`) and is **not optional** — rate-limiting velocity alone turns
acceleration into a square wave, and `accel_to_lean` feeds acceleration
straight into torque, so stage 1 alone just moves the discontinuity from `v`
to `a`. Offline sim measured 0.663 Nm/tick peak torque slew with stage 1
only, 0.074 Nm/tick with both.

The rate limiter lives INSIDE `balance_controller`, not in a smoother node
upstream, deliberately: a smoother would keep publishing fresh messages after
its own input died, `balance_controller`'s watchdog would never fire, and
staleness detection would have silently migrated out of the node CLAUDE.md
requires to own it.

### Other

- Velocity filter is now a TIME CONSTANT (`v_filter_tau`), not a hardcoded
  0.85/0.15. The old coefficient was 123 ms at the 50 Hz `odrive_bridge`
  actually publishes, not the 65 ms its comment claimed.
- `wheel_radius`, `cutoff_pitch` promoted from hardcoded to params.
- Teleop speeds +50%: linear 0.225, turbo 0.45, yaw 0.75; `accel_limit` 0.45
  so full-stick ramp time stays 0.5 s.
- Runbook: added TEST 6 (stand-side qualification), revised TEST 3.

---

## READY TO TUNE 2026-08-02 — friction compensation (stiction)

**Symptom it targets:** the robot balances in place but cannot hold still —
leans, wheels do not respond, leans further, breaks free, over-corrects.
Bounded hunting, RANDOM direction, always recovers. That is a symmetric
stiction deadband. (Consistent direction would mean BIAS instead — check
`pitch_trim` and the IMU, not this.)

Two terms in `odrive_bridge`, both **defaulting to 0.0 = OFF**. Installing this
changes nothing until tuned on the robot. All four params are read live, so
tune with `ros2 param set /odrive_bridge <name> <value>` while it balances — no
restart, no rebuild.

- `dither_torque` — square-wave dither that keeps the wheel micro-moving so
  there is no STATIC friction left to break. **This is the one that fixes
  standstill.** Applied in ANTIPHASE (left +, right −) so the translational
  components cancel exactly and only a tiny yaw wiggle remains, which the
  balance loop is blind to. In phase it would shake the robot fore-and-aft,
  straight into the pitch loop we are trying to help.
- `friction_ff` — Coulomb feedforward on wheel velocity. **Does nothing at
  standstill** (friction opposes MOTION and there is none yet); it is what
  makes DRIVING smooth. It is positive velocity feedback, so deliberately
  under-compensate — ~70% of measured breakaway.

**Calibration:** Kt = 0.43 Nm/A × gear 8 = 3.44 Nm/A at the wheel, k3 = 20, so
`0.35 Nm breakaway ≈ 0.1 A of Iq ≈ 1° of pitch deadband`. Hunting over ±2°
implies breakaway near 0.7 Nm.

**Order:** `dither_torque` first, in 0.05 Nm steps until hunting stops. Then
`friction_ff`. Then `friction_v_eps` only if low-speed driving feels notchy.

**Do not raise `dither_hz` toward 50.** At exactly half the ~100 Hz command
rate every sample lands on the square wave's transition and the sign is decided
by floating-point rounding — measured as a −0.04 Nm **DC torque bias**, the
very failure mode this is meant to avoid. Default 25 Hz; the bridge warns above
30 Hz.

**Safety contract, do not remove:** exactly `[0.0, 0.0]` from
`balance_controller` passes through uncompensated. `balance_controller` keeps
publishing at 100 Hz while DISABLED, so without that branch dither would
energise motors the operator deliberately shut off. Covered by
`tools/test_friction_comp.py` (26 checks, no hardware).

---

## CLOSED 2026-08-02 — yaw rate limiting (was the KNOWN GAP)

Two defects, both fixed in `balance_controller`, both covered by
`tools/test_yaw_shaping.py` (11 checks, no hardware).

**1. yaw arrived as a step.** `linear.x` got two stages of shaping and yaw got
none, so a stick flick delivered `k_yaw * 0.75 = 3.0 Nm` of differential torque
in one tick, out of an 8.0 Nm budget shared with balancing. Now ramped by a new
`yaw_accel_limit` (2.0 rad/s², in real.yaml) — 0.08 Nm/tick, the same slew the
linear path achieves with both of its stages. Full-stick ramp 0.375 s.

Only ONE stage for yaw, deliberately: stage 2 on the linear path exists because
`accel_to_lean` feeds acceleration into the lean target, and yaw has no such
feedforward. A second lag would only add steering delay.

**2. saturation quietly stole from balance.** `left` and `right` were clamped
INDEPENDENTLY. At torque 7.0 with t_yaw 3.0 that gives left 4.0 / right 10.0 →
8.0, so the common mode holding the robot up dropped from 7.0 to 6.0 — a 1.0 Nm
loss, paid out of BALANCING, invisible in the logs. The budget is now spent in
priority order: balance takes what it needs, yaw gets the headroom.

**Still worth testing on hardware** — the sign convention is unchanged and has
never had current through it (BALANCE_RUNBOOK TEST 4). What has changed is that
a hard flick can no longer arrive as a step, so the spin-while-nudging test is
now a tuning check rather than a hazard. Turn `yaw_accel_limit` DOWN if turning
upsets the pitch loop; that is the knob to reach for before `scale_angular.yaw`.

---

## NEXT MILESTONE: standalone operation

Goal, in the dev's words (**CORRECTED 2026-08-02**): *turn on the robot, turn on
the PS4 controller, press a button to switch to teleop mode, drive.* This is
CLAUDE.md goals 1 and 3.

**The earlier "get on the hotspot, go to the website, teleop, drive" framing was
a misunderstanding and is withdrawn.** The dev never wanted to drive from a web
page. The physical DS4 is the ONLY control device, and the dashboard is
**optional** — it exists to change modes conveniently and to visualise
readouts, which is exactly what CLAUDE.md goal 4 already said. No driving
control belongs on it.

Five pieces:

| | piece | notes |
|---|---|---|
| **A** | `slcand` under systemd | **WRITTEN 2026-08-01, NOT YET RUN ON THE PI.** `deploy/biped-can.service`. Handles the zombie-interface trap (`pkill -x slcand` BEFORE `ip link delete`, on start AND on stop). Resolves the CANable through `/dev/serial/by-id/` rather than a bare `/dev/ttyACM0` — stable across re-enumeration and needs no custom udev rule of ours that could silently stop matching. Test: `deploy/README.md` TEST 7. |
| **B** | ROS stack under systemd | **WRITTEN 2026-08-01, NOT YET RUN ON THE PI.** `deploy/biped-stack.service`. This is what kills the six-terminal workflow. Boots into DISABLED (mode_manager already does). Test: `deploy/README.md` TEST 8. **But read the leg-arming finding below before enabling it at boot.** |
| **C** | IMU I2C -> UART | backlog 2. CLAUDE.md says before untethered running, and driving off a stand IS untethered. Clock stretching drops samples. Wiring + PS1/PS0 strapping are in real.yaml's imu_node comment. |
| **D** | Pi hotspot | **WRITTEN 2026-08-02, NOT YET RUN ON THE PI.** `biped-wifi.service` + `biped-wifi-mode.sh`. Home WiFi preferred, AP fallback, and the AP is *sticky* once up — see the design note below. Test: `deploy/README.md` TEST 10. |
| **E** | **Mode switch from the DS4** | **WRITTEN 2026-08-02, LOGIC FULLY TESTED, indices unmeasured.** Replaces the withdrawn "web teleop". Tap → TELEOP, L1+L2 → DISABLED. `tools/test_mode_buttons.py` passes 21/21 with no hardware. Test: `deploy/README.md` TEST 9. |

**A, B, D and E are all written.** What is left is running them on the Pi:
TESTs 7-10 in `deploy/README.md`, plus C (the IMU rewire) before the first
genuinely untethered drive.

### ⚠️ THE UNBUILT PREREQUISITE FOR E — Bluetooth pairing

The button logic is done and tested. **But "turn on the PS4 controller" assumes
the DS4 pairs to the PI, and it never has** — every joystick session so far ran
through the laptop as driver station. So E's real remaining work is:

1. **Pair + `trust` the DS4 to the Pi in `bluetoothctl`**, so the PS button
   reconnects on its own after a power cycle. Untested, unknown effort.
2. **`joy_node` must grab the right device.** The same class of trap as the
   laptop, where `js0` turned out to be the accelerometer.
3. Button logic — done.

If 1 does not work, 2 and 3 are decoration. Do 1 first.

### Piece E is much smaller than the old piece E was

`mode_manager` **already subscribes to `/joy`** — but only to timestamp
controller presence; the message content is explicitly discarded
([mode_manager.py:50-53](src/robot_teleop/robot_teleop/mode_manager.py#L50-L53)).
So the change is: read a button out of that same callback and call the
already-factored-out `set_mode()`. No new node, no new topic, no new service
round-trip — the subscription, the interlock and the setter all exist.

Three things to settle before writing it:

1. **Which button, and edge-vs-level.** It must be a deliberate press, so latch
   on the rising edge — a level test would re-fire at 20 Hz.
2. **Does the same button toggle back to DISABLED, or does a different one?**
   Worth remembering that DISABLED is not "safe" here: with weight on the legs
   it means COLLAPSE. Consider a separate, harder-to-hit button for it.
3. **The interlock is already correct and should stay.** Entering a motion mode
   requires a live controller
   ([mode_manager.py:80](src/robot_teleop/robot_teleop/mode_manager.py#L80)) —
   which a button press on that very controller trivially satisfies, so the
   interlock costs nothing and still blocks the dashboard from arming a robot
   with no pad connected.

Note the deadman already exists and is unaffected: `teleop_twist_joy` has
`enable_button: 7`, so even in TELEOP mode `cmd_vel` only flows while that
button is held.

### A+B are written. What is left is running them on the Pi.

Everything is in `deploy/` — two units, four helper scripts, an idempotent
`install.sh`, and TEST 7 / TEST 8 in the standard format. Validated as far as a
laptop allows: `systemd-analyze verify` clean on both units, `bash -n` clean on
all five scripts, `slcand -F` confirmed present, the device-resolution failure
path exercised (fails loudly with an actionable message, exit 1), and
`biped-stack.sh` run end-to-end against this workspace — it sourced Jazzy plus
the overlay, exec'd `ros2 launch`, brought up 12 processes including rosbridge
on 9090 and the dashboard on 8000, and passed `can_channel:=vcan0
imu_driver:=fake` through to the nodes. **None of that proves anything about
the Pi**: no CANable, no reboot, no `systemctl`. TEST 7 and TEST 8 are the real
verification.

`install.sh` is deliberately **install-only**. Running it changes nothing about
the next boot; `systemctl enable` is a separate, explicit act.

### ⚠️ NEW SAFETY FINDING — `leg_controller` arms and MOVES at startup

Found while writing piece B, verified in source, not previously written down.

`odrive_bridge` arms both wheels unconditionally in `__init__`
(`arm()` at odrive_bridge.py:77) with no reference to the mode. That is fine:
`balance_controller` publishes `[0.0, 0.0]` while DISABLED, and a torque-mode
axis at 0 Nm is limp.

`leg_controller` is different. It arms in **POSITION** mode and ramps to
`home_position` (leg_controller.py:140-145), also without consulting the mode.
`home_position` is `0.0` — the retracted stop. So **"power on" would mean "the
legs drive themselves to retracted", with nobody's hand near the cutoff.**

An earlier draft claimed this was mitigated because the left hip was off the
CAN bus. **That was wrong on both counts:** the hip was never broken — it was
unplugged for a single test on 2026-07-29 and the absence got recorded as a
fault — and "a cable happens to be out" is not a safety mechanism.

**RESOLVED 2026-08-02 by not launching the node.** `real.launch.py` now
declares `legs`, default **false**, and `leg_controller` runs only with
`legs:=true`. No process, nothing to arm. That is a stronger guarantee than a
check inside the node, and it needed no change to `leg_controller` itself.

Consequences:
- **`systemctl enable biped-stack` is now safe.** On power-on the only thing
  that energizes is the two wheel axes at 0 Nm (limp), with mode DISABLED.
- **Backlog 7 is no longer a blocker on shipping the wheeled robot.** It is now
  a prerequisite for turning the legs ON.
- **`legs:=true` carries the whole hazard.** Do not put it in the service, and
  do not use it for leg bring-up — see below.

### Design decisions taken for E and D (2026-08-02)

**E — one policy point, not two.** The interlock ("entering a motion mode needs
a live controller") used to live in the `SetMode` service callback. A button
handler calling `set_mode()` directly would have skipped it, giving the robot
two rule sets depending on who asked. Both paths now go through
`request_mode()`. The buttons live in `mode_manager` rather than a new node
because `mode_manager` already had to subscribe to `/joy` for the presence
watchdog — a separate node would mean two subscribers to one topic for two
halves of one concern.

**E — the gestures are deliberately asymmetric.** Tap → TELEOP (one press, you
do it constantly). L1+L2 together → DISABLED (two fingers, because it cuts
torque and drops the robot). Edge-detected, not level: `joy_node` autorepeats
at 20 Hz. Mode gestures are on the LEFT hand; `teleop_twist_joy`'s enable(R2)
and turbo(R1) are on the right. Worth knowing: driving is the left *stick*, so
the disable combo sits under the steering hand's index/middle finger — a
two-button combo should still be safe, but that is where to look if it ever
misfires.

**E — indices are params in `real.yaml`, not constants.** Loaded from SOURCE,
so finding the right number is edit-and-restart with no `colcon build` in the
loop — which matters because it is a guess-and-check loop, and a rebuild in the
middle of one is how you end up testing the old value.

**D — exactly one thing decides which network is up.** The AP profile is
created `autoconnect no` so NetworkManager does not hold a competing opinion;
`biped-wifi-mode.sh` is the sole decider. Two deciders make a flapping
interface with no single cause to point at.

**D — the AP is STICKY once up, by default.** Falling *to* the AP is automatic
(that is the case where you have no other way in). Returning to home WiFi is
not, and the reason is technical before it is cautious: most drivers cannot
scan while operating as an AP on the same radio, so noticing home WiFi returned
requires tearing the AP down to look. That outage would land while you are in
the field with a phone on it. Switch back deliberately:
`biped-wifi-mode.sh home`.

**D — WPA2 is a safety control, not hygiene.** `rosbridge` binds `0.0.0.0:9090`
and exposes `/set_mode`; on an open AP anyone in range could put the robot in
TELEOP and energize it. The passphrase lives in NetworkManager's root-only
store and is never written into this repo.

### WITHDRAWN 2026-08-02 — the web-teleop design

This section used to specify a held-pointer deadman on the dashboard, wired as
a third twist_mux input below the joystick, and argued at length that it
reversed an earlier decision not to allow on-screen driving.

**All of that is deleted. The dev never asked for web teleop** — it was an
assumption, restated confidently enough across two documents and a memory file
that it started to look like a requirement. The 2026-07-18 decision it claimed
to reverse (*no on-screen driving*) was right, and stands.

The dashboard changes modes and shows readouts. It does not drive the robot.
`twist_mux` keeps its two inputs (`joy_vel`, `nav_vel`); no third input is
needed.

Kept because it is still true and still useful: `balance_controller`'s watchdog
degrades ANY loss of `cmd_vel` to "zeros the velocity reference and keeps
balancing" in 0.5 s, and that is already tested (TEST 5). It is what makes a
dropped DS4 connection safe too.

---

## GOTCHA LEARNED TODAY — the two yaml files load differently

- `src/robot_bringup/config/real.yaml` is passed as `--params-file` from the
  SOURCE path. Edit it, restart the node, done. **No build.**
- `src/robot_teleop/config/teleop_twist_joy.yaml` is loaded from the
  **share** directory via `get_package_share_directory`. Edit it and nothing
  happens until `colcon build --packages-select robot_teleop`.

Same file extension, same-looking edit, completely different deploy step. A
speed change that "didn't take" is almost always this.

---

---

## SCOPE DECISION 2026-08-02 — legs are deferred

**Finish the wheeled robot first; legs are a separate project afterwards.**
`balance_controller` contains no leg references at all, so this costs the
current goal nothing — the legs were only ever sharing the CAN bus and the
launch file.

Mechanism: `real.launch.py` declares `legs` (default **false**), which gates
both `leg_controller` and `leg_joy`. Verified all three ways — default gives
zero leg processes, `legs:=true` restores both, and `teleop.launch.py` on its
own still defaults legs on so the sim path is unchanged. (`leg_joy` returning
under `legs:=true` is the check that the argument really reached the include
rather than silently defaulting.)

`real.yaml`'s `leg_controller:` block is KEPT, not deleted — it holds the
accumulated measurements and reasoning, and it is what the eventual leg test
will load. It now carries a header saying editing it does nothing on a normal
run, because "my tuning change didn't take" and "the node was never started"
look identical from a terminal.

### When the legs DO come back, the rig has to be built for it

The dev's requirement, in his words: *easily and most importantly SAFELY test
and tune the legs.* **`legs:=true` on the full stack is not that**, and must
not be mistaken for it — it runs the legs inside a live balance loop on a robot
that has never moved them, changing two things at once.

What to build instead, when the time comes:

- **A legs-alone launch.** `leg_controller` only — no balance, no wheels armed.
  One variable under test.
- **Deliberately low `current_limit`** (real.yaml ships 5.0). Under-current on a
  leg means SAG, which is the safe direction to be wrong in; over-current
  breaks printed linkage parts, which has already happened once on this robot.
- **The power-cycle test FIRST** (HARDWARE_BRINGUP.md step 0). It decides
  whether `pos_max` can be 0.10 or 0.19, and everything downstream assumes an
  answer nobody has measured.
- **The one-time zero measurement**, then `use_measured_zero: true`.
- **Backlog 7 (gate `arm()` on mode) before the legs join the balance stack** —
  not necessarily before the legs-alone bring-up, where you want the node to
  arm on purpose.
- **A TEST in TEST_FORMAT.md shape**, with the abort being the physical power
  cut. A position-mode leg does not give up, and `Ctrl-C` is not fast enough.

Remember DISABLED means the legs COLLAPSE (position mode) — so "just disable
it" is not an abort once there is weight on them.

---

## BACKLOG (unchanged except where noted)

0. **LEGS ARE DEFERRED (2026-08-02).** Items 1 and 7 below are leg work and are
   parked until the wheeled robot is finished. See the scope decision above.
1. ~~Left hip (node 3) off the CAN bus~~ — **NOT A FAULT. CLOSED 2026-08-02.**
   All four motors are on the bus and the left hip works. It was unplugged for
   a single test on 2026-07-29; the missing `061` heartbeat was recorded as a
   hardware fault and then carried forward through four documents as if it were
   one. **Lesson: a one-off observation is not a diagnosis.** Legs are not
   blocked — but see item 7, which this makes urgent rather than theoretical.
2. IMU I2C -> UART. Now on the critical path for standalone (piece C).
3. `slcand` under systemd. **WRITTEN** (`deploy/`), awaiting TEST 7 on the Pi.
4. Friction feedforward for standstill stiction. **May have shrunk** — sim
   attributed much of the standstill rocking to outer-loop ringing from
   `a2 = -0.3` rather than stiction. Check what TEST 3 actually showed before
   building this.
5. `robot_description` has undeclared sim-only deps (controller_manager,
   ros_gz_bridge, ros_gz_sim). Deliberately not added — declaring them means
   rosdep pulls Gazebo onto the Pi.
6. Yaw rate limiting (new, see KNOWN GAP above).
7. **Gate `leg_controller`'s `arm()` on mode.** **DEFERRED with the legs** — no
   longer blocks `systemctl enable biped-stack`, because the node is no longer
   launched. Required before the legs join the balance stack. Same pattern
   `balance_controller` already uses for torque.
8. Run TEST 7 + TEST 8 on the Pi (`deploy/README.md`). A and B are written but
   entirely unverified on real hardware.
9. ~~Piece E — DS4 button to enter TELEOP~~ **CODE DONE 2026-08-02**, 21/21 in
   `tools/test_mode_buttons.py`. Two things remain, both on the Pi:
   **(a) pair the DS4 to the Pi over Bluetooth** — never done, the real unknown;
   **(b) measure the button indices** with `tools/joy_probe.py` and write them
   into `real.yaml` + `sim.yaml`. Then TEST 9.
10. **Piece D — hotspot. CODE DONE 2026-08-02**, unverified: this laptop has no
    WiFi radio, so AP capability could not be checked here. Run
    `biped-wifi-setup.sh` on the Pi (it checks `iw list` and the regulatory
    domain for you), then TEST 10.

---

## HARD-WON RULES (carried forward, all still true)

- Verify params at RUNTIME (`ros2 param get /node param`), never by reading
  the yaml. File vs launch vs CLI overrides disagree and fail silently.
- A param is live-tunable only if the code fetches it inside a callback.
  Captured-at-startup params (`publish_rate`) need a restart.
- `imu_calibrate.py --watch` prints DEGREES; `pitch_trim` is RADIANS.
- Encoder estimates read zero unless the axis is in CLOSED_LOOP.
- launch nodes need `output='screen'` or errors go to a log file you are not
  reading.
- `pkill slcand` BEFORE `ip link delete`, or a zombie netdev squats the name.
- The dev is a beginner in ROS 2 — explain the why, don't just hand over code.
