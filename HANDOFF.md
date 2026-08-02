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

## KNOWN GAP — yaw is not rate-limited

`balance_controller` shapes `linear.x` only. `yaw_ref` reaches the law as a
step: `k_yaw * 0.75 = 3.0 Nm` of differential torque, instantly, out of an
8.0 Nm budget. Since `left = torque - t_yaw` and `right = torque + t_yaw` are
each clamped, a hard yaw flick WHILE the robot is catching itself can saturate
one wheel and clip the BALANCE torque.

Fine at the old 0.5. At 0.75 it is untested. **Test it deliberately: spin hard
while nudging it fore/aft.** If it loses authority mid-spin, that is this.
Fix is to shape yaw the same way as linear — small change, same pattern.

---

## NEXT MILESTONE: standalone operation

Goal, in the dev's words: *turn on, get on the hotspot, go to the website,
teleop, drive.* This is CLAUDE.md goals 1 and 4. Five pieces:

| | piece | notes |
|---|---|---|
| **A** | `slcand` under systemd | **WRITTEN 2026-08-01, NOT YET RUN ON THE PI.** `deploy/biped-can.service`. Handles the zombie-interface trap (`pkill -x slcand` BEFORE `ip link delete`, on start AND on stop). Resolves the CANable through `/dev/serial/by-id/` rather than a bare `/dev/ttyACM0` — stable across re-enumeration and needs no custom udev rule of ours that could silently stop matching. Test: `deploy/README.md` TEST 7. |
| **B** | ROS stack under systemd | **WRITTEN 2026-08-01, NOT YET RUN ON THE PI.** `deploy/biped-stack.service`. This is what kills the six-terminal workflow. Boots into DISABLED (mode_manager already does). Test: `deploy/README.md` TEST 8. **But read the leg-arming finding below before enabling it at boot.** |
| **C** | IMU I2C -> UART | backlog 2. CLAUDE.md says before untethered running, and hotspot teleop IS untethered. Clock stretching drops samples. Wiring + PS1/PS0 strapping are in real.yaml's imu_node comment. |
| **D** | Pi hotspot | NetworkManager AP mode. Independent of ROS, testable alone. Note it takes the radio — the Pi cannot be on your home WiFi and be an AP on the same interface. |
| **E** | Web teleop | The dashboard, rosbridge, and phone access ALREADY WORK. This is adding a control to a working page, not building one. |

**Recommended order: A+B first.** Biggest quality-of-life win, needs no
hardware, testable in one reboot, and it addresses the actual pain (six
terminals). Then D, then E. C before the first genuinely untethered drive.

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

It cannot happen today only because the left hip (node 3) is off the CAN bus,
so `establish_zero` times out and the node refuses to arm. That is safety by
accident. **Backlog 1 (left hip) and enabling `biped-stack` at boot must not
both be true until `leg_controller` gates `arm()` on mode** — new backlog
item 7. Until then: enable `biped-can` at boot, start `biped-stack` by hand.

E is the most fun and the lowest marginal value right now — the DS4 already
works, and E is the only piece that cannot be fully judged until everything
beneath it does.

### Design decision for E, settle before building

**A web page needs a deadman.** Reuse the invariant that already exists rather
than inventing a new safety path: the page publishes only while a touch or
pointer is held down, at a fixed rate. Release, close the tab, lock the phone,
walk out of range — publishing stops, and `balance_controller`'s watchdog
zeros velocity in 0.5 s and KEEPS BALANCING. Identical behaviour for every
failure mode, and it is already tested (TEST 5).

Wire it as a THIRD twist_mux input with its own timeout, at LOWER priority
than the joystick, so plugging in the DS4 always overrides the browser.

**This reverses a decision from 2026-07-18** which deferred on-screen driving
because "a laggy WiFi touch stick shouldn't keep it upright." That reasoning
was correct when written and is now obsolete: the balance loop runs entirely
on the Pi at 100 Hz, `cmd_vel` is only a velocity REFERENCE, and the watchdog
degrades a dropout to "stops and stands there" rather than "falls." WiFi lag
never touches the loop that keeps it upright.

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

## BACKLOG (unchanged except where noted)

1. Left hip (node 3) off the CAN bus — no `061` heartbeat. Legs blocked.
   **Now coupled to item 7 — do not fix this and enable `biped-stack` at boot
   without also doing 7.**
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
7. **Gate `leg_controller`'s `arm()` on mode** (new, see the safety finding
   above). Blocks "enable `biped-stack` at boot" from coexisting with item 1.
   Same pattern `balance_controller` already uses for torque.
8. Run TEST 7 + TEST 8 on the Pi (`deploy/README.md`). A and B are written but
   entirely unverified on real hardware.

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
