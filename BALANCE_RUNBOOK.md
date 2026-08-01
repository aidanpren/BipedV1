# Balance Bring-up Runbook

_Written 2026-07-31, after the wheel-direction and IMU-mount gates passed._

Everything below assumes those gates are green. If you are unsure, re-run the
PRE-FLIGHT section — it is cheap and it is the whole safety argument.

**Stop command, know it before you start:** `Ctrl-C` in the balance_controller
terminal. Torque stops publishing, `cmd_timeout: 0.5` fires in `odrive_bridge`
and the wheels COAST. Also know where the physical power cut is. Do not rely on
DISABLED mode — it is not wired to a button yet.

---

## PRE-FLIGHT (every session — the Pi loses CAN on reboot)

```bash
cd ~/BipedV1
source install/setup.bash

ls /dev/ttyACM*                                    # find the CANable
sudo pkill -f slcand; sudo ip link delete can0 2>/dev/null; sleep 1
sudo slcand -o -c -f -s6 /dev/ttyACM0 can0
sudo ip link set up can0
ip -br link show can0                              # expect: can0  UP
timeout 3 candump can0                             # expect heartbeats 001 021 041
```
`pkill slcand` MUST come before `ip link delete`, or a carrier-less zombie
squats the name and the next attempt fails with `SIOCSIFNAME: File exists`.

`061` missing = left hip (node 3) still off the bus. Does not block balance.

### Verify the two safety gates

```bash
# gate 1: IMU mount
ros2 run robot_base imu_node --ros-args \
  --params-file src/robot_bringup/config/real.yaml          # terminal B
ros2 param get /imu_node driver                             # i2c
ros2 param get /imu_node mount_rpy                          # [-0.051677, 0.016108, 3.107462]
ros2 topic hz /imu                                          # ~100
python3 tools/imu_calibrate.py --watch                      # nose-down -> pitch POSITIVE
```
| Result | Meaning |
|---|---|
| pitch POSITIVE on nose-down | correct — the loop will drive into the fall |
| pitch NEGATIVE | **STOP.** Re-run `--solve`. The loop would accelerate the fall |
| roll moves a lot when you pitch | in-plane skew; works but re-solve tipping on the WHEELS |
| no data at all | wrong `driver` — see the liveness error the node prints |

```bash
# gate 2: wheel direction
ros2 run robot_base odrive_bridge --ros-args \
  --params-file src/robot_bringup/config/real.yaml -p can_channel:=can0
ros2 topic pub -r 10 /wheel_effort_controller/commands \
  std_msgs/Float64MultiArray "{data: [0.5, 0.5]}"
```
Both wheels turn the SAME way and that way is FORWARD. `/joint_states`
velocities both positive. Expect acceleration, not steady speed — torque mode
on an unloaded wheel has no speed target.

---

## TEST 1 — stand, inner loop only

Robot on a stand, **wheels off the ground**.

```bash
# A
ros2 run robot_base odrive_bridge --ros-args \
  --params-file src/robot_bringup/config/real.yaml -p can_channel:=can0
# B
ros2 run robot_base imu_node --ros-args \
  --params-file src/robot_bringup/config/real.yaml
# C
ros2 run robot_base balance_controller --ros-args \
  --params-file src/robot_bringup/config/real.yaml -p a1:=0.0 -p a2:=0.0
# D
ros2 topic pub -r 2 /mode std_msgs/String "{data: 'teleop'}" \
  --qos-durability transient_local
```

**Why `a1:=0.0 a2:=0.0`:** those drive the outer position/velocity loop off
wheel odometry. With the wheels free-spinning, `x` runs away and the loop
chases a position it can never reach. Zeroing them leaves
`torque = k3*pitch + k4*pitch_rate` — one thing to judge.

| Action | Expected |
|---|---|
| tilt nose-down | wheels spin FORWARD, harder with more tilt |
| tilt nose-up | wheels spin BACKWARD |
| hold level | wheels nearly still, maybe a slow creep |
| tilt past ~40 deg | wheels STOP (`cutoff_pitch = 0.7` rad, hardcoded) |

This is a sanity check on polarity and magnitude, not a real test. If anything
here is wrong, do not go to the ground.

---

## TEST 2 — ground, inner loop only

Robot **tethered**, legs collapsed, hand ready. Same four terminals.

Hold it upright, enable mode, then ease your hands off gradually.

**It WILL wander.** There is no position hold with `a1`/`a2` at zero — the
robot holds the angle, not the spot. Correct behaviour; do not fix it yet.

Watch the balance_controller log line: `pitch  x  v  L  R`.

| What you see | Cause | Fix |
|---|---|---|
| fast buzz / vibration | `k3` too high, or IMU lag | `k3` 20 -> 12, then 15 |
| slow growing oscillation | `k4` too low | `k4` 2.0 -> 3.0 -> 4.0 |
| catches, overshoots, catches worse | `k4` too low relative to `k3` | raise `k4` first |
| sags and falls, wheels barely move | `k3` too low | `k3` up in steps of 5 |
| holds ~5 s then diverges | usually `k4` | raise `k4` |
| drifts steadily one way, `pitch` log pinned near 0 | fore/aft imbalance | go to TEST 2A |
| drifts one way AND won't sit level | residual mount tilt | re-run `--solve` |
| wheels saturate instantly | `max_torque` too low for the gains | see note below |

Change ONE gain at a time, by no more than ~50%, and restart terminal C:
```bash
ros2 run robot_base balance_controller --ros-args \
  --params-file src/robot_bringup/config/real.yaml \
  -p a1:=0.0 -p a2:=0.0 -p k3:=15.0 -p k4:=3.0
```

**On saturation.** `torque = k3*pitch`, so the controller saturates at
`pitch = max_torque / k3`. At `max_torque 8.0`, `k3 20` that is 0.4 rad
(23 deg). Measured ceiling is `0.43 Nm/A * 12 A * 8 = 41.3 Nm`, so there is
plenty of headroom — but do not raise `max_torque` to fix bad gains. A robot
that oscillates at 8 Nm oscillates harder at 20.

Your pendulum is SHORTER than sim's because the legs are collapsed, so it falls
faster and generally wants more damping (`k4`) than Gazebo did.

---

## TEST 3 — outer loop (station keeping)

_Standard shape — see TEST_FORMAT.md._

**Proves:** that the outer loop closes the free velocity integrator TEST 2A
left open, so the robot holds a SPOT rather than just an ANGLE — and, as a
by-product, independently re-measures `pitch_trim`.

**Rig:** ground, tethered, legs collapsed, hands hovering. Clear **2 m in
front and behind** — it may lurch once before it settles.

**Abort:** `Ctrl-C` in terminal C -> `cmd_timeout` fires -> wheels COAST.

**Prerequisites:** TEST 2A passed — trim measured, and the robot holds 30 s
without accelerating in one direction.

### Commands

PRE-FLIGHT as usual (the Pi loses CAN on reboot), then:

```bash
# terminal A
ros2 run robot_base odrive_bridge --ros-args \
  --params-file src/robot_bringup/config/real.yaml -p can_channel:=can0
# terminal B
ros2 run robot_base imu_node --ros-args \
  --params-file src/robot_bringup/config/real.yaml
# terminal C — NOTE: no a1/a2 override this time, they come from the file
ros2 run robot_base balance_controller --ros-args \
  --params-file src/robot_bringup/config/real.yaml
# terminal D
ros2 topic pub -r 2 /mode std_msgs/String "{data: 'teleop'}" \
  --qos-durability transient_local
```

```bash
# VERIFY AT RUNTIME that the overrides are actually gone. This is the whole
# point of the test and it is one habit-slip away from silently not running.
ros2 param get /balance_controller a1          # expect -0.05, NOT 0.0
ros2 param get /balance_controller a2          # expect -0.15, NOT 0.0
ros2 param get /balance_controller pitch_trim  # expect 0.085
```

Live-tune without restarting. **`a1` and `a2` are NEGATIVE — make them more
negative to strengthen them. Never make them positive:** positive a1/a2 is the
saturated-lean runaway documented at balance_controller.py:19.
```bash
ros2 param set /balance_controller a1 -0.075   # 50% stronger position pull
ros2 param set /balance_controller a2 -0.20    # 33% stronger velocity damping
```

### Expected

| Action | Expected |
|---|---|
| mode goes teleop, hands off | balances; outer loop is OFF for the first 1 s by design |
| **at ~1 s** | a small step or lean as the outer loop engages — expected, not a fault |
| after a few seconds | settles; `v` decays toward 0 instead of coasting |
| logged `x` | returns toward `x_home` and stays within ~0.2 m |
| logged `pitch` | settles at **~+0.085**, i.e. your trim |
| push it gently fore/aft | leans against you, recovers, returns to about the same spot |
| leave it 60 s | still there, still upright |

### Failure modes

| Symptom | Cause | Fix |
|---|---|---|
| drifts away slowly, never returns | `a1` too weak | `a1` -0.05 -> -0.075 |
| lurching, wind-up, overshoots home | `a1` too strong | back `a1` off toward -0.03 |
| endless slow rocking about home | `a2` too weak | `a2` -0.15 -> -0.20 |
| parks consistently off home by a fixed distance | residual trim error | use the formula below |
| small stop-start hunting around home | stiction, not tuning | expected at low speed; nudge `a2` up, or accept |
| snaps over hard when the outer loop engages | `a1` badly wrong, or sign flipped | check `a1` is NEGATIVE |
| behaves exactly like TEST 2A | overrides still applied | re-check `ros2 param get a1` |

### The trim cross-check (the second measurement)

At steady state `a1*(x - x_home) + trim = θ_true`, so a persistent parking
offset `d = x - x_home` is a direct readout of how wrong the trim is:

```
trim_correction = a1 * d          (a1 = -0.05)
new_trim = trim + a1 * d
```

Parks 0.2 m BEHIND home -> `d = -0.2` -> correction `+0.01` -> raise trim to
0.095. Same direction as the TEST 2A rule (drifts backward -> more trim), but
now quantitative, and measured by the robot rather than by your hands. If this
disagrees with the hand measurement by more than ~0.02 rad, one of the two is
wrong — do not just average them.

### Why this works

TEST 2A left wheel velocity as an unforced free integrator: the law regulated
pitch and nothing read `x` or `v`. `a2` closes the loop on `v` directly, which
is what stops the coast. `a1` closes it on position, which is what brings it
home. Both work by commanding a small LEAN — the robot has no way to push
itself sideways except by falling in the direction it wants to go and catching
itself further along.

The 1 s dead time is `upright_since` in balance_controller.py: the outer loop
is deliberately suppressed until the robot has been upright for a second, so
position feedback cannot fight a recovery mid-tumble. `pitch_trim` is applied
throughout, including during that second — which is why the trim had to be
right before this test could mean anything.

`max_lean: 0.3` bounds the outer loop's lean command around the trim (clamp
applied BEFORE the trim is added, so the authority is symmetric), which makes
a badly tuned outer loop tip slowly instead of snapping over.

### Pass criteria

- Returns to within **0.2 m** of where you released it, and stays.
- Survives a deliberate fore/aft nudge and comes back.
- **60 s** unaided.
- Steady `pitch` equals `pitch_trim` within ~0.02 rad.

### On pass

Write the tuned `a1`/`a2` into real.yaml **with a comment saying why**, apply
any trim correction from the cross-check, then TEST 4.

---

## TEST 4 — driving (translation, then yaw)

_Standard shape — see TEST_FORMAT.md._

**Proves:** that the robot tracks a commanded velocity and a commanded yaw
rate while staying balanced — and it is the FIRST test of the differential
(yaw) sign, which gate 2 never checked.

**Rig:** ground, tethered with SLACK, legs collapsed. Clear **3 m of runway**
plus room to spin. Hands off but following it.

**Abort:** `Ctrl-C` terminal C (wheels coast). To stop it driving without
stopping balance, `Ctrl-C` the `cmd_vel` publisher only — the watchdog zeros
the reference in 0.5 s and it keeps balancing. That is TEST 5.

**Prerequisites:** TEST 3 passed — holds station 60 s, returns after a nudge.

### Commands

Same four terminals as TEST 3, plus a fifth for commands.

```bash
# terminal E — forward, gently
ros2 topic pub -r 10 /cmd_vel geometry_msgs/Twist \
  "{linear: {x: 0.2}, angular: {z: 0.0}}"
```
```bash
# then backward
ros2 topic pub -r 10 /cmd_vel geometry_msgs/Twist \
  "{linear: {x: -0.2}, angular: {z: 0.0}}"
```
```bash
# yaw — START LOW, this sign has never been tested on hardware
ros2 topic pub -r 10 /cmd_vel geometry_msgs/Twist \
  "{linear: {x: 0.0}, angular: {z: 0.3}}"
```
```bash
# finally the joystick
ros2 launch robot_teleop teleop.launch.py
```

### Expected

| Action | Expected |
|---|---|
| command +0.2 m/s | **wheels first roll BACKWARD ~0.2 s**, robot pitches nose-down, THEN drives forward |
| once up to speed | `pitch` returns to ~`pitch_trim`; `v` settles at **0.2**, not 0.13 or 0.26 |
| command −0.2 m/s | mirror image: brief forward twitch, leans back, drives backward |
| stop publishing | leans back to decelerate, stops, resumes holding station |
| `angular.z: 0.3` | spins in place, **counter-clockwise seen from above**, roughly constant rate |
| `angular.z: -0.3` | clockwise |
| joystick | same behaviours, proportional to stick |

### Failure modes

| Symptom | Cause | Fix |
|---|---|---|
| rolls backward and KEEPS going on a +0.2 command | wrong `invert_*` — but see below, this is NOT the brief backstep | STOP; re-run gate 2 |
| steady speed consistently offset from commanded | residual `pitch_trim` error | `offset = trim_error/a2`; re-check TEST 3 cross-check |
| yaw rate runs away to full speed on a small command | differential sign inverted — positive feedback on yaw | STOP; swap the `t_yaw` sign convention |
| turns the wrong way but at a controlled rate | yaw sign convention, harmless | negate `k_yaw` or fix the sign |
| tips over on the velocity step | `a2` too strong; the lean command is too aggressive | reduce `a2` magnitude |
| drives fine, wanders when stopped | expected — `x_home` follows while driving |  |

### Why this works

**The backstep is real and correct.** A wheeled inverted pendulum is
non-minimum phase: to accelerate forward it must first put its body ahead of
its wheels, and the only way to do that is to drive the wheels BACKWARD for a
moment. Watch the math — on a +0.2 command,
`pitch_target = a2*(v_f - v_ref) + trim` = `-0.15*(0 - 0.2) + 0.085` = `0.115`,
which is MORE nose-down than the current 0.085, so
`torque = k3*(0.085 - 0.115)` is NEGATIVE. It backs up on purpose.

That is also how you tell it from a wrong `invert_*`: the backstep is brief
(~0.2 s) and REVERSES. A sign error diverges and never comes back.

**Steady-state speed is exact because the trim is right.** At constant
velocity `pitch` must equal `θ_eq`, so `a2*(v_f - v_ref)` must be zero, so
`v_f = v_ref`. Untrimmed it would have settled `trim/a2` = `0.085/-0.15` =
**0.57 m/s** slow — a +0.2 command would have driven it BACKWARD at 0.37 m/s,
which is why this test was worthless before TEST 2A.

**Yaw is genuinely unverified.** Gate 2 drove both wheels with `[0.5, 0.5]`,
which only ever tested COMMON mode. `left = torque - t_yaw` /
`right = torque + t_yaw` is DIFFERENTIAL and has never had current through it
in that configuration. It is also a RATE loop, `k_yaw*(yaw_ref - ω_z)`, so a
flipped sign is positive feedback and spins up rather than merely turning the
wrong way. Start at 0.3, not 0.5.

### Pass criteria

- Forward and backward both track within ~20% of commanded steady speed.
- Stays upright through every start and stop.
- Yaw turns the commanded direction at a controlled, bounded rate.
- Joystick reproduces all of it.

### On pass

Record anything retuned. Then TEST 5 — the safety invariant.

---

## TEST 5 — the cmd_vel watchdog (safety invariant)

**Proves:** that losing `cmd_vel` zeros the velocity reference and **KEEPS
BALANCING** — it does not cut torque. This is the non-negotiable invariant
from CLAUDE.md and it has never been verified on hardware.

**Rig:** ground, tethered, hands close. It should NOT fall — that is the
point — but be ready for the case where it does.

**Abort:** `Ctrl-C` terminal C.

**Prerequisites:** TEST 4 passed.

### Commands

```bash
# drive it, then kill ONLY the publisher — leave A/B/C/D running
ros2 topic pub -r 10 /cmd_vel geometry_msgs/Twist "{linear: {x: 0.2}}"
# ... let it reach speed, then Ctrl-C THIS terminal only
```
```bash
# harsher version: yank the network / kill the joystick node mid-drive
ros2 launch robot_teleop teleop.launch.py     # then Ctrl-C it while moving
```

### Expected

| Action | Expected |
|---|---|
| publisher stops | within 0.5 s the robot decelerates smoothly to a stop |
| after stopping | **still upright, still actively balancing**, holding station |
| terminal C log | torque values keep updating — they do NOT go to 0.0 and stay |
| push it after the timeout | it still resists and recovers |

### Failure modes

| Symptom | Cause | Severity |
|---|---|---|
| torque goes to zero and it DROPS | watchdog wired to cut torque instead of zeroing refs | **CRITICAL** — this is the invariant |
| keeps driving at 0.2 forever | watchdog not firing; check `cmd_timeout` reaches the node | serious — runaway |
| stops but then drifts off | expected only if `a1`/`a2` regressed | check params |

### Why this works

Three behaviours must never be merged, and this test isolates the first:
1. **Stale `cmd_vel`** -> zero the velocity references, KEEP BALANCING.
2. **DISABLED mode** -> deliberately cut torque, only ever from an explicit
   command, because it drops the robot.
3. **Stale wheel torque** (`odrive_bridge`, `cmd_timeout: 0.5`) -> COAST,
   because the controller itself has died and there is nothing left to
   balance with.

The watchdog under test is #1, in `balance_controller`: `age > 0.5` sets
`v_ref`/`yaw_ref` to zero and then falls through into the normal balance law.
Note that #3 has the same 0.5 s timeout but a completely different meaning —
do not let them blur together.

### Pass criteria

- Robot remains upright and balancing indefinitely after cmd_vel stops.
- Torque continues to be published and to respond to pushes.

### On pass

Balance bring-up is complete. Move to the backlog: left hip on CAN, IMU to
UART, slcand under systemd, and friction feedforward for the standstill
stiction hunting.

---

## TEST 2A — measure and apply `pitch_trim`

_Written in the standard shape — see TEST_FORMAT.md._

**Proves:** that the robot's backward drift is a fore/aft imbalance, not a
gain problem, by measuring the chassis angle at which the CoM sits over the
contact patch and handing it to the inner loop.

**Rig:** Part 1 motors UNPOWERED, robot in your hands. Part 2 on the ground,
tethered, legs collapsed, hands hovering. Same leg posture in both parts —
the trim is a property of a posture, not of the robot.

**Abort:** `Ctrl-C` in terminal C. Torque stops publishing, `cmd_timeout: 0.5`
fires in `odrive_bridge`, wheels COAST. Know where the physical power cut is.

**Prerequisites:** PRE-FLIGHT green, both safety gates green, TEST 1 (stand)
passed. `balance_controller` must be the build that has `pitch_trim` — verify
at runtime, step 0.

### Commands

```bash
# Get the code onto the Pi and rebuild — pitch_trim's clamp order changed.
cd ~/BipedV1 && git pull && colcon build --packages-select robot_base robot_bringup
source install/setup.bash
```

```bash
# PRE-FLIGHT — the Pi loses CAN on every reboot
ls /dev/ttyACM*
sudo pkill -f slcand; sudo ip link delete can0 2>/dev/null; sleep 1
sudo slcand -o -c -f -s6 /dev/ttyACM0 can0
sudo ip link set up can0
ip -br link show can0            # expect: can0  UP
timeout 3 candump can0           # expect 001 021 041  (061 absent = left hip, fine)
```

**Part 1 — measure. Motors unpowered.**

```bash
# terminal B — IMU only. Do NOT start odrive_bridge: with no bridge the
# ODrives stay in IDLE and the wheels spin freely, which is what you need.
ros2 run robot_base imu_node --ros-args \
  --params-file src/robot_bringup/config/real.yaml
```
```bash
# terminal E — live attitude, in DEGREES
python3 tools/imu_calibrate.py --watch
```

Balance the robot by hand on its wheels. Rock it gently and feel for the null
— the angle where it does not want to tip either way. Read `pitch` there.
Take three readings and average; you are looking for a stable number, not a
precise one.

**Convert: `--watch` prints DEGREES, `pitch_trim` is RADIANS. Divide by 57.3.**

Write it into `src/robot_bringup/config/real.yaml` as `pitch_trim`.

**Part 2 — apply. Ground, tethered.**

```bash
# 0. verify the running node actually has the parameter, before trusting it
ros2 param get /balance_controller pitch_trim
```
```bash
# terminal A
ros2 run robot_base odrive_bridge --ros-args \
  --params-file src/robot_bringup/config/real.yaml -p can_channel:=can0
# terminal B
ros2 run robot_base imu_node --ros-args \
  --params-file src/robot_bringup/config/real.yaml
# terminal C — a1/a2 zeroed so ONLY the inner loop + trim is under test
ros2 run robot_base balance_controller --ros-args \
  --params-file src/robot_bringup/config/real.yaml -p a1:=0.0 -p a2:=0.0
# terminal D
ros2 topic pub -r 2 /mode std_msgs/String "{data: 'teleop'}" \
  --qos-durability transient_local
```

Live-tune without restarting — `pitch_trim` is re-read inside the IMU callback:
```bash
ros2 param set /balance_controller pitch_trim 0.05   # NOT 0 — bare int is rejected
```

### Expected

| Action | Expected |
|---|---|
| hand-balanced, `--watch` | `pitch` steady at **+1 to +6 deg** |
| trim applied, hands off | `pitch` in the log sits at your trim value, not 0.000 |
| watch `x` in the log | wanders and coasts; does NOT pick a direction and build speed |
| nudge it fore/aft | returns to the same pitch, drifts from wherever it ended up |
| trim set too low | still drifts backward, slower |
| trim set too high | drift reverses — it now runs forward |

Tuning rule: **drifts backward -> increase `pitch_trim`.** 0.01 steps.

### Failure modes

| Symptom | Cause | Fix |
|---|---|---|
| lurches backward hard the instant mode goes teleop | you pasted DEGREES into a radians field | divide by 57.3 |
| `pitch_trim` not a parameter | Pi running the old build | `git pull && colcon build`, re-source |
| constant slow creep that never accelerates | NOT the trim — dragging wheel or torque offset | check both wheels spin equally free by hand |
| drift direction flips run to run | you are reading a null that isn't there | re-check legs are collapsed and equally so |
| `pitch` log won't settle near trim at all | `k3`/`k4` wrong, not trim | back to TEST 2 gain tuning first |
| measured trim > 0.15 rad (8.6 deg) | robot is genuinely badly balanced | move the battery, don't trim around it |

### Why this works

The equilibrium is where the CoM sits over the CONTACT PATCH, which equals
"chassis level" only if the robot is balanced fore and aft. Back-heavy means
it must stand slightly NOSE-DOWN to put the mass over the wheels. Hold
pitch = 0 instead and gravity applies a constant torque about the contact
point; the only way to sustain that angle is to accelerate backwards forever,
at roughly `g * trim` ≈ 0.5 m/s². No `k3`/`k4` fixes it, because it is a bias,
not a gain.

The measurement is valid because both programs compute pitch identically —
`asin(2*(w*y - z*x))` on the same `/imu` quaternion, in
`balance_controller.py` and `imu_calibrate.py` alike. `--watch` is not a
similar angle; it is bit-for-bit the number the loop subtracts the trim from.
If either formula ever changes, this procedure silently stops being valid.

With `a1`/`a2` at zero the trim removes the ACCELERATION, not the motion:
nothing is regulating position or velocity yet, so it will still wander. The
signature of success is that it stops *running away*, not that it stops moving.

`pitch_trim` is FEEDFORWARD of a known constant; the outer loop is FEEDBACK
against unknown ones. They are not alternatives. `a1` is proportional on
position, and a P term converts a constant disturbance into a constant ERROR
rather than removing it — leaving the trim at 0.0 makes the robot station-keep
at `pitch_trim/a1` = 0.05/-0.05 = **one metre** from home, permanently, and
drive `pitch_trim/a2` = **0.33 m/s** slower than commanded, forever. Ask for
0.2 m/s forward and it rolls backward. The trim also covers the one second
after recovery when the outer loop is deliberately switched off
(`if not settled: pitch_target = 0.0`) — the most fragile second there is.

### Pass criteria

- Hand-measured trim lands in **0.02–0.10 rad** and repeats within ~1 deg.
- With trim applied and `a1`/`a2` at zero, the robot holds upright unaided for
  **30 s** without accelerating in one direction.
- Logged `pitch` sits at the trim value, not at 0.000.

### On pass

Record the measured value in `real.yaml` with the date and how it was
measured. Then TEST 3 — and cross-check there: the steady pitch the outer
loop parks at is this same number, arrived at by the robot instead of by hand.
If they disagree by much, one of the two measurements is wrong.

---

## AFTER IT WORKS — the backlog

1. **Left hip (node 3)** back on the CAN bus, then legs via `leg_controller`
   (measure `zero_raw_*`, set `use_measured_zero` — see HARDWARE_BRINGUP.md).
2. **IMU to UART.** I2C drops samples to clock stretching; that is not good
   enough to hold the robot up long-term. Wiring and strapping are in
   real.yaml's imu_node comment.
3. **`slcand` in systemd** so CAN survives a reboot — required for the
   "powers on and balances" goal.
4. **Write the tuned gains into real.yaml** and record why.
5. **Friction feedforward** for the standstill hunting. Observed at TEST 3
   (2026-08-01): station keeping holds, but the robot rocks slightly back and
   forth about home. That is a stiction limit cycle, not a tuning fault —
   through the 8:1 gearbox, breakaway torque is large enough that small
   corrections do nothing until they suddenly do, and the robot cannot sit
   inside the deadband because a locked-wheel inverted pendulum is unstable.
   More `a2` will not fix it and may amplify it. The proper fix is the
   friction-feedforward term from the LQR reference paper: add a torque of
   `sign(v) * f_c` (plus a viscous term) so the loop starts from the edge of
   the deadband rather than the middle of it. Standstill-only — it does not
   appear once the wheels are turning.
