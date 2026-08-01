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

Once it holds upright unaided for ~30 s, restore the outer loop:

```bash
ros2 run robot_base balance_controller --ros-args \
  --params-file src/robot_bringup/config/real.yaml     # a1/a2 from the file
```

`a1: -0.05` pulls it back toward `x_home`, `a2: -0.15` damps velocity. Together
they stop the wandering by commanding a small lean.

| Expected | Wrong |
|---|---|
| returns to roughly one spot | slow drift away = `a1` too small |
| gentle lean when nudged, then recovers | lurching / wind-up = `a1` too big |
| settles within a couple of seconds | endless slow rocking = `a2` too small |

`max_lean: 0.3` clamps the commanded lean, so a bad outer loop tips slowly
rather than snapping over.

---

## TEST 4 — driving

```bash
ros2 topic pub -r 10 /cmd_vel geometry_msgs/Twist \
  "{linear: {x: 0.2}, angular: {z: 0.0}}"
```
Forward should mean the robot leans slightly forward, then rolls. Stop
publishing and the watchdog zeros the reference after 0.5 s — it keeps
BALANCING, it does not cut torque. That is the safety invariant; verify it
deliberately once.

Then yaw: `angular: {z: 0.5}`. `k_yaw: 4.0` splits torque between the wheels.

Finally the joystick: `ros2 launch robot_teleop teleop.launch.py`.

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
