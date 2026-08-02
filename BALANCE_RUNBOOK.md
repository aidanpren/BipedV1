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

> **Superseded once TEST 7 passes.** `deploy/biped-can.service` does all of
> this at boot, including the zombie cleanup, and resolves the CANable through
> `/dev/serial/by-id/` instead of assuming `/dev/ttyACM0`. After that the
> pre-flight is just `systemctl status biped-can`. Keep the manual sequence
> below — it is still how you bring CAN up on the bench, and how you check
> whether a systemd failure is systemd's fault or the bus's.

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
_REVISED 2026-08-01 for the unified control law and the `a1`/`a2` retune.
The old expected values (-0.05/-0.15) and the old "rocking -> raise `a2`"
advice are superseded — see the failure-mode table._

**Proves:** that the outer loop closes the free velocity integrator TEST 2A
left open, so the robot holds a SPOT rather than just an ANGLE — and, as a
by-product, independently re-measures `pitch_trim`.

**What this test actually exercises now.** With no `cmd_vel` publisher running,
`v_cmd` stays at zero, and at `v_cmd = 0` the new unified law collapses to
*exactly* the old station-keeping law — verified numerically, max difference
0.00e+00 rad. So this is no longer a test of the rewrite. It is a test of the
**retune**, `a1/a2` from -0.09/-0.3 to -0.07/-0.15, plus one genuinely new
behaviour: the `max_pos_error` clamp.

**Rig:** ground, tethered, legs collapsed, hands hovering. Clear **2 m in
front and behind** — it may lurch once before it settles.

**Abort:** `Ctrl-C` in terminal C -> `cmd_timeout` fires -> wheels COAST.

**Prerequisites:** TEST 2A passed — trim measured, and the robot holds 30 s
without accelerating in one direction. **TEST 6 passed** — the Pi is on the new
build and the reference shaping behaves. If you skipped TEST 6, the runtime
parameter check below is the part of it you cannot skip.

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
ros2 param get /balance_controller a1             # expect -0.07, NOT 0.0, NOT -0.09
ros2 param get /balance_controller a2             # expect -0.15, NOT 0.0, NOT -0.3
ros2 param get /balance_controller pitch_trim     # expect 0.08
ros2 param get /balance_controller max_pos_error  # expect 0.25
ros2 param get /balance_controller jerk_tau       # errors => OLD BUILD, stop
```

Live-tune without restarting. **`a1` and `a2` are NEGATIVE — more negative is
stronger. Never make them positive:** positive a1/a2 is the saturated-lean
runaway documented at balance_controller.py:19.

**They are NOT independent knobs.** The outer loop is second order:

```
omega_n = sqrt(g*|a1|)          zeta = (|a2|/2) * sqrt(g/|a1|)
```

so changing `a1` alone changes the damping ratio too. If you strengthen the
position pull, strengthen the damping with it:

```bash
# a matched pair — omega_n up ~20%, zeta held near 0.9
ros2 param set /balance_controller a1 -0.10
ros2 param set /balance_controller a2 -0.18
```

**Do not raise `|a2|` past about -0.20 to cure rocking.** That was the old
advice and it is backwards. `g*|a2|` is the driving-loop bandwidth, and the
non-minimum-phase ceiling for this robot is roughly 1.6 rad/s, i.e. `|a2|`
around 0.16. -0.3 sits at nearly twice that, and offline sim of this exact
code showed it ringing at standstill — 48 direction reversals, never settling
inside 20 s — even though the lag-free `zeta` formula calls it overdamped
(1.57). The formula is optimistic because it ignores inner-loop and filter lag.

### Expected

| Action | Expected |
|---|---|
| mode goes teleop, hands off | balances; outer loop is OFF for the first 1 s by design |
| **at ~1 s** | a small step or lean as the outer loop engages — expected, not a fault |
| after a few seconds | settles; `v` decays toward 0 instead of coasting |
| logged `x` | returns toward `x_home` and stays within ~0.2 m |
| logged `pitch` | settles at **~+0.08**, i.e. your trim |
| push it gently fore/aft | leans against you, recovers, returns to about the same spot |
| leave it 60 s | still there, still upright |
| **standstill rocking vs last session** | **noticeably less**, or gone — this is the headline prediction of the retune |
| logged `err` | stays small; **never exceeds ±0.25** (the new clamp) |
| shove it more than 0.25 m | still returns, but the pull no longer grows with distance — constant authority beyond the clamp, so the last part of the trip is slower |

### Failure modes

| Symptom | Cause | Fix |
|---|---|---|
| drifts away slowly, never returns | `a1` too weak | `a1` -0.07 -> -0.10, and `a2` -0.15 -> -0.18 with it |
| lurching, wind-up, overshoots home | `a1` too strong | back BOTH off: `a1` -0.05, `a2` -0.13 |
| **endless slow rocking about home** | **ambiguous — two opposite causes.** Either `a2` too weak (underdamped) or `a2` too STRONG (lag-limited ringing, which is what -0.3 was doing) | **Test which: drop `a2` to -0.10.** Worse => it was too weak, go to -0.18. Better => it was too strong, stay low. Do NOT reflexively raise it |
| parks consistently off home by a fixed distance | residual trim error | use the formula below |
| small stop-start hunting around home, wheels visibly sticking | stiction, not tuning | expected; backlog item 5 (friction feedforward). More `a2` will not fix it and may amplify it |
| returns from a big shove but crawls the last stretch | `max_pos_error` clamp — working as designed | none; raise `max_pos_error` only if you have a reason |
| `err` grows past 0.25 | clamp not working — **stop, do not drive** | this is the runaway guard; report it |
| snaps over hard when the outer loop engages | `a1` badly wrong, or sign flipped | check `a1` is NEGATIVE |
| behaves exactly like TEST 2A | overrides still applied | re-check `ros2 param get a1` |
| behaves exactly like LAST session (same rocking) | Pi on the old build | `ros2 param get jerk_tau` — if it errors, rebuild |

### The trim cross-check (the second measurement)

At steady state `a1*(x - x_home) + trim = θ_true`, so a persistent parking
offset `d = x - x_home` is a direct readout of how wrong the trim is:

```
trim_correction = a1 * d          (a1 = -0.07)
new_trim = trim + a1 * d
```

Parks 0.2 m BEHIND home -> `d = -0.2` -> correction `+0.014` -> raise trim to
0.094. Same direction as the TEST 2A rule (drifts backward -> more trim), but
now quantitative, and measured by the robot rather than by your hands. If this
disagrees with the hand measurement by more than ~0.02 rad, one of the two is
wrong — do not just average them.

**The formula is only valid for `|d| < max_pos_error` (0.25 m).** Past the
clamp, `x_home` is dragged along behind the robot, `a1*(x - x_home)` stops
growing, and the offset no longer reads out the trim error — it reads out the
clamp. A parking offset pinned at almost exactly 0.25 m is that, not a trim
measurement.

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

**`max_pos_error` is the one new mechanism here.** `x_home` is now an
integrator — it ramps at `v_cmd` — and an unbounded integrator against a robot
that cannot keep up is a wind-up trap: hold the robot still while it wants to
move and the reference walks away, storing distance the robot will sprint to
recover the moment you let go. Clamping `x_home` to within 0.25 m of `x` bounds
that. The side effect you will feel is that beyond 0.25 m the restoring pull
stops growing with distance — `a1 * 0.25` = 0.0175 rad of lean, about
0.17 m/s² — so a big shove comes home under constant rather than proportional
authority. That is the intended trade: slower recovery from a large
displacement, in exchange for no runaway.

### Pass criteria

- Returns to within **0.2 m** of where you released it, and stays.
- Survives a deliberate fore/aft nudge and comes back.
- **60 s** unaided.
- Steady `pitch` equals `pitch_trim` within ~0.02 rad.
- Logged `err` never exceeds ±0.25.
- **Standstill rocking is no worse than last session.** Better is the
  prediction; equal is acceptable; worse means the retune hurt and you should
  say so rather than tune around it.

### On pass

Write the tuned `a1`/`a2` into real.yaml **with a comment saying why**, apply
any trim correction from the cross-check, then TEST 4.

Record the rocking result explicitly, either way. The sim claims the standstill
rocking was largely outer-loop ringing from `a2 = -0.3` rather than stiction.
If it improved, backlog item 5 (friction feedforward) shrinks. If it did not,
the stiction diagnosis stands and item 5 is still the real fix — that is a
useful measurement, not a failed test.

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

## TEST 6 — stand, the rewritten law and reference shaping

_Standard shape — see TEST_FORMAT.md. Written 2026-08-01, after the control
law was unified and `a1`/`a2` retuned._

**Proves:** that the rewritten `balance_controller` is safe to put back on the
ground — the inner loop still behaves exactly as it did, the new reference
shaping ramps `v_cmd` smoothly instead of stepping, and the anti-windup clamp
actually bounds the reference position.

**Rig:** stand, **wheels OFF the ground**, robot sitting roughly UPRIGHT on the
stand (see below — this matters). Legs collapsed. No tether needed; it cannot
go anywhere.

**Abort:** `Ctrl-C` in terminal C. Torque stops publishing, `cmd_timeout: 0.5`
fires in `odrive_bridge`, wheels COAST. In Parts 2 and 3 the controller cannot
command torque at all, so there is nothing to abort from.

**Prerequisites:** TEST 3, 4 and 5 previously passed on the OLD code. This test
re-qualifies the new code before it goes back to the ground; it does not
replace them.

**Why upright matters:** `|pitch| > cutoff_pitch` (0.7 rad) takes the fallen
branch, which resets `v_cmd` and `v_ramp` to zero every cycle. Lie the robot
over on the stand and Part 2 reads as a dead ramp for a reason that has
nothing to do with the ramp.

### Commands

```bash
# Get the new code onto the Pi. The .py changed, so this MUST be rebuilt —
# unlike the yaml, which is read from source by --params-file.
cd ~/BipedV1 && git pull
colcon build --packages-select robot_base robot_bringup
source install/setup.bash
```

PRE-FLIGHT as usual (the Pi loses CAN on reboot), then:

**Part 0 — verify the build at RUNTIME. This is the gate.**

```bash
# terminal A
ros2 run robot_base odrive_bridge --ros-args \
  --params-file src/robot_bringup/config/real.yaml -p can_channel:=can0
# terminal B
ros2 run robot_base imu_node --ros-args \
  --params-file src/robot_bringup/config/real.yaml
# terminal C
ros2 run robot_base balance_controller --ros-args \
  --params-file src/robot_bringup/config/real.yaml
# terminal D
ros2 topic pub -r 2 /mode std_msgs/String "{data: 'teleop'}" \
  --qos-durability transient_local
```

```bash
# terminal E — every one of these must return a value, not an error.
# An error on ANY of them means the Pi is running the OLD build. Stop here.
ros2 param get /balance_controller jerk_tau        # 0.15   <- new, check first
ros2 param get /balance_controller accel_limit     # 0.3
ros2 param get /balance_controller accel_to_lean   # 0.102
ros2 param get /balance_controller max_pos_error   # 0.25
ros2 param get /balance_controller v_filter_tau    # 0.06
ros2 param get /balance_controller wheel_radius    # 0.105
ros2 param get /balance_controller cutoff_pitch    # 0.7
ros2 param get /balance_controller a1              # -0.07  <- RETUNED
ros2 param get /balance_controller a2              # -0.15  <- RETUNED
ros2 param get /balance_controller pitch_trim      # 0.08
```

**Part 1 — inner loop regression. Did the rewrite change anything it shouldn't?**

Restart terminal C with the outer loop off, exactly as TEST 1:

```bash
# terminal C
ros2 run robot_base balance_controller --ros-args \
  --params-file src/robot_bringup/config/real.yaml -p a1:=0.0 -p a2:=0.0
```

Tilt the chassis by hand. This must behave **identically to TEST 1** — it is
the same code path, and any difference is a bug introduced by the refactor.

**Part 2 — reference shaping, with torque physically impossible.**

```bash
# terminal C — k3/k4/k_yaw ZERO: torque = 0 always, the wheels cannot move.
# accel_limit and jerk_tau are slowed 15x ON PURPOSE: the log line is
# throttled to 1 Hz, so at the real 0.3 m/s^2 the whole ramp is over inside
# one log line and you would see nothing.
ros2 run robot_base balance_controller --ros-args \
  --params-file src/robot_bringup/config/real.yaml \
  -p k3:=0.0 -p k4:=0.0 -p k_yaw:=0.0 \
  -p accel_limit:=0.02 -p jerk_tau:=0.5
```

```bash
# terminal E — step the command. Watch the "v <v_f>-><v_cmd>" field in C.
ros2 topic pub -r 10 /cmd_vel geometry_msgs/Twist "{linear: {x: 0.15}}"
# let it climb for ~10 s, then Ctrl-C THIS terminal only and watch it ramp back
```

```bash
# terminal E — then the sub-threshold check. 0.03 is BELOW the old 0.05
# driving threshold, i.e. the bottom third of the stick that used to be
# silently discarded.
ros2 topic pub -r 10 /cmd_vel geometry_msgs/Twist "{linear: {x: 0.03}}"
```

**Part 3 — anti-windup clamp, by hand.**

Leave terminal C as it is from Part 2 (still zero torque, so the wheels turn
freely in your hand). Watch the `err` field.

```bash
# nothing to run — spin BOTH wheels forward by hand, steadily, through more
# than 0.5 m of travel (about 0.8 turns each). Keep going well past it.
```

### Expected

| Action | Expected |
|---|---|
| Part 0, every `param get` | returns a value; none error |
| Part 1, tilt nose-down | wheels spin FORWARD, harder with more tilt — same as TEST 1 |
| Part 1, tilt nose-up | wheels spin BACKWARD |
| Part 1, past ~40 deg | wheels STOP (`cutoff_pitch`) |
| Part 2, command 0.15 | `v_cmd` climbs **smoothly from 0.00 toward 0.15** over ~8 s |
| Part 2, first 1–2 log lines | climb starts **gently**, not at full slope — that is `jerk_tau` |
| Part 2, `v_cmd` at rest | reaches 0.15 and stays; does not overshoot or hunt |
| Part 2, wheels throughout | **do not move at all** — `k3 = k4 = 0` |
| Part 2, Ctrl-C the publisher | `v_cmd` ramps back DOWN to 0.00; it does **not** jump |
| Part 2, command 0.03 | `v_cmd` climbs to 0.03 and holds — the old code ignored this entirely |
| Part 3, spin wheels forward | `err` grows, then **stops at +0.25** and stays no matter how far you spin |
| Part 3, spin backward | `err` goes to **−0.25** and stops |

### Failure modes

| Symptom | Cause | Fix |
|---|---|---|
| any `param get` errors "Parameter not set" | Pi running the OLD build — the whole point of Part 0 | `git pull && colcon build --packages-select robot_base`, re-source, restart C |
| params return the OLD `a1 -0.09 / a2 -0.3` | pulled the code but not the yaml, or an override is set | check for stray `-p a1:=` on the command line |
| `v_cmd` stays 0.00 forever | robot lying past `cutoff_pitch` on the stand — fallen branch resets it | sit it upright |
| `v_cmd` stays 0.00, robot IS upright | mode never went teleop; disabled branch returns early | check terminal D is still publishing |
| `v_cmd` jumps straight to 0.15 in one line | ramp not applied — check `accel_limit` came through at runtime | `ros2 param get` it |
| `v_cmd` climbs but wheels also spin in Part 2 | `k3`/`k4` overrides did not take | they are CLI overrides; verify at runtime |
| `err` keeps growing past 0.25 in Part 3 | anti-windup clamp not working — **do not go to the ground** | report it; this is the clamp that stops a runaway |
| Part 1 differs from TEST 1 in any way | refactor changed the inner loop | stop; the inner loop was supposed to be untouched |

### Why this works

The rewrite replaced a two-branch law (drive vs station-keep) with one
continuous law whose reference POSITION ramps at the reference SPEED. Nothing
in that touches the inner loop, so Part 1 is a pure regression check: if
`torque = k3*(pitch - pitch_target) + k4*pitch_rate` still responds to a hand
tilt exactly as it did in TEST 1, the sign chain and the IMU path survived.

Part 2 is the only way to see the new code honestly. On a stand `x` and `v` are
meaningless — the wheels spin free, so anything driven by odometry is chasing
a number that runs away. But `v_cmd` is derived **purely from `cmd_vel` and the
clock**, not from odometry, so it is exactly as valid on a stand as on the
ground. Zeroing `k3`/`k4` removes the only path from the controller to the
motors, which makes this a software test that happens to be running on the
robot. The 15x slowdown is not a different test — `accel_limit` and `jerk_tau`
are linear time-scalings of the same ramp, so the SHAPE you are checking is
unchanged; you are only making it slower than the 1 Hz log throttle.

The gentle start is the whole point of `jerk_tau`. A bare rate limiter bounds
acceleration, which makes acceleration a SQUARE WAVE — and since
`accel_to_lean` feeds acceleration straight into the lean target, that square
wave lands in torque and the jerk survives. Offline sim of this exact code
measured peak torque slew at 0.663 Nm/tick with rate limiting alone and
0.074 Nm/tick with both stages. What you are looking for in those first log
lines is that the climb *eases in* rather than starting at full slope.

Part 3 checks the one new failure mode that is genuinely dangerous. `x_home`
integrates `v_cmd` without limit, so if the robot cannot keep up — blocked,
held, torque-saturated — the reference runs away, and every centimetre of that
is distance the robot will sprint to recover the moment it comes free. The
clamp bounds it at `max_pos_error`. Spinning the wheels by hand is a direct
simulation of exactly that: you are moving `x` while `v_cmd` is zero, which is
the same divergence with the sign reversed.

### Pass criteria

- Every parameter in Part 0 returns a value, with `a1 = -0.07`, `a2 = -0.15`.
- Part 1 is indistinguishable from TEST 1.
- `v_cmd` reaches the commanded value **monotonically**, with a visibly gentle
  start, and ramps back down on publisher loss rather than stepping.
- A 0.03 m/s command produces a `v_cmd` of 0.03 — the discarded bottom third
  of the stick is gone.
- `err` saturates at **±0.25** and does not exceed it under sustained hand
  spinning.

### On pass

Nothing to record — this test measures no new constants. Restore the real
`accel_limit`/`jerk_tau` by restarting terminal C without the overrides, then
re-run **TEST 3** on the ground: station keeping is what the retune changed
most, and the sim predicts a large improvement (peak excursion 0.229 -> 0.132 m,
settling 20+ s -> 4.1 s, 48 direction reversals -> 1). Then TEST 4, then TEST 5.

If TEST 3 does NOT improve, the most likely explanation is that the standstill
rocking really is stiction (backlog item 5) rather than the outer-loop ringing
the sim attributes it to — that is a useful result, not a failure.

---

## AFTER IT WORKS — the backlog

1. **Left hip (node 3)** back on the CAN bus, then legs via `leg_controller`
   (measure `zero_raw_*`, set `use_measured_zero` — see HARDWARE_BRINGUP.md).
2. **IMU to UART.** I2C drops samples to clock stretching; that is not good
   enough to hold the robot up long-term. Wiring and strapping are in
   real.yaml's imu_node comment.
3. **`slcand` in systemd** so CAN survives a reboot — required for the
   "powers on and balances" goal. **WRITTEN**: `deploy/`, with TEST 7 (CAN
   under systemd) and TEST 8 (the ROS stack under systemd) in
   `deploy/README.md`. Not yet run on the Pi. Read the leg-arming safety
   finding at the top of that file before enabling anything at boot.
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
