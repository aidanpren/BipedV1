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
| drifts steadily one way | residual mount tilt | re-run `--solve` |
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

## THE BACK-HEAVY DRIFT — `pitch_trim`

If it drifts steadily one way and no `k3`/`k4` combination stops it, it is
almost certainly not a tuning problem.

The equilibrium is where the CoM sits over the CONTACT PATCH, which is only
"chassis level" if the robot is perfectly balanced fore and aft. Back-heavy
means the chassis has to stand slightly NOSE-DOWN to put the mass over the
wheels. Holding pitch = 0 instead leaves a constant gravity torque, and the
only way to hold that angle is to accelerate backwards forever.

**Measure the trim:**
1. Power the motors OFF.
2. Balance the robot by hand on its wheels — find the angle where it does not
   want to tip either way. Rock it gently to feel the null.
3. Read pitch there:
   ```bash
   python3 tools/imu_calibrate.py --watch
   ```
4. Put that number in real.yaml as `pitch_trim` (POSITIVE = nose-down).

**Then re-run TEST 2.** The drift should stop with `a1`/`a2` still at zero,
because the inner loop is now holding the actual equilibrium angle.

Sanity: for a robot this size expect a few degrees, i.e. 0.02-0.10 rad. If the
number you measure is bigger than ~0.15 rad, the robot is very unbalanced and
you would be better off moving the battery than trimming around it.

Note the outer loop CAN hide this on its own — `a1` will command the needed
lean — but only by carrying a permanent position error of `pitch_trim/a1`. It
parks off-home with `a1` fighting gravity full time, which eats the authority
you wanted for rejecting real disturbances.

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
