# BipedV1 — Hardware Bring-up Handoff

_Last updated 2026-07-29. Living doc — update it as bring-up progresses._

This is the resume point for hardware work. Sim is complete; the motor/CAN
layer is up and the ROS read-loop is proven on real hardware. What remains is
the **staged, on-a-stand bring-up of everything that moves under power.**

---

## STATUS AT A GLANCE

| Layer | State |
|---|---|
| Sim (LQR balance, prismatic legs, sim_leg_bridge, leg_joy) | ✅ done + tested |
| Pi deployed, builds, no-hardware tests pass | ✅ |
| All 4 motors on CAN at correct IDs | ✅ |
| All 4 motors calibrated | ✅ |
| `odrive_bridge` ROS↔CAN↔motor **read** loop on real HW | ✅ proven |
| `leg_controller` safe-startup (read→zero→seed→arm) | ✅ written + tested no-HW |
| Legs moving under ROS | ⬜ next |
| Power-cycle test: what `pos_estimate` does across a reboot | ⬜ **do first, decides `pos_max`** |
| Leg zero measured (`zero_raw_*`, `use_measured_zero`) | ⬜ one-time |
| Direction (`invert_*`) verified open-loop | ✅ 2026-07-30 — both flipped |
| IMU up (I2C) + `mount_rpy` solved + pitch sign verified | ✅ 2026-07-30 |
| Wheels closed-loop + balance | ⬜ next |
| Left hip (node 3) back on the CAN bus | ⬜ dropped off, not blocking balance |
| IMU rewired I2C -> UART | ⬜ before untethered running |

---

## THE HARDWARE

- **Pi:** Ubuntu 24.04, hostname **`biped`**, user **`aidanpren`**.
  `ssh aidanpren@biped.local` (laptop needs `libnss-mdns avahi-daemon`, installed).
  Repo at `~/BipedV1`.
- **Motors:** Steadywin **GIM8108-8 + GDS68** driver. They run **ODrive-fork
  firmware** and speak genuine **ODrive CANSimple** — configure with
  **`odrivetool`** (native on Linux; do NOT fight MotorWizard/Zadig/Wine).
- **Encoder:** MA732, 14-bit single-turn **absolute**.
- **Node IDs** (confirmed via candump): **right wheel=0, right hip=1,
  left wheel=2, left hip=3.** `real.yaml` matches this.
- **CAN adapter:** CANable 2.0, **slcan** firmware → `/dev/ttyACM0`.
- **Bus:** 500 kbps. Gear ratio 8.

---

## CHEAT SHEET (commands that work)

**Bring up CAN on the Pi:**
```bash
ls /dev/ttyACM*                                   # confirm the CANable path
sudo pkill -f slcand; sudo ip link delete can0 2>/dev/null; sleep 1
sudo slcand -o -c -f -s6 /dev/ttyACM0 can0        # -s6 = 500k
sudo ip link set up can0
candump can0                                      # heartbeats: 001 021 041 061
```

**Config a motor over USB (odrivetool):**
```python
odrv0.config.enable_can_a  = True
odrv0.can.config.protocol  = 1            # CANSimple
odrv0.can.config.baud_rate = 500000
odrv0.axis0.config.can.node_id = <N>
odrv0.can.config.enable_r120 = <True/False>   # ends ON, middle OFF (see termination)
odrv0.save_configuration()                    # reboots; VERIFY by reading back after
```

**Calibrate (only if `pre_calibrated` is False, or the encoder/rotor was disturbed):**
```python
odrv0.axis0.motor.config.pre_calibrated       # check first — may skip entirely
odrv0.axis0.encoder.config.pre_calibrated
# if needed (DECOUPLE from linkage or ensure clearance; 2A cal current):
odrv0.axis0.requested_state = AXIS_STATE_MOTOR_CALIBRATION
dump_errors(odrv0)
odrv0.axis0.requested_state = AXIS_STATE_ENCODER_OFFSET_CALIBRATION
dump_errors(odrv0)
odrv0.axis0.encoder.config.pre_calibrated = 1
odrv0.axis0.motor.config.pre_calibrated = 1
odrv0.save_configuration()
```

**Leg startup logic, with NO hardware at all** (no vcan, no root — runs the
real node against two fake ODrives on python-can's in-process virtual bus):
```bash
python3 tools/test_leg_startup.py
```

**Wheels-only READ test (safe — torque mode, 0 torque, wheels stay free):**
```bash
# on a stand. Terminal 1:
ros2 run robot_base odrive_bridge --ros-args \
  --params-file ~/BipedV1/src/robot_bringup/config/real.yaml -p current_limit:=3.0
# Terminal 2:
ros2 topic echo /joint_states     # spin a wheel by hand -> values change
```

---

## FIXES / GOTCHAS THAT COST TIME (read before debugging CAN again)

- **Encoder estimates are ZERO unless the axis is in CLOSED_LOOP.** This fork
  answers the RTR/cyclic `Get_Encoder_Estimates` with all-zero data in IDLE.
  It looks exactly like "every joint is at the origin", which is a convincing
  and completely wrong answer to seed a position ramp with. `leg_controller`
  therefore arms in TORQUE mode at ZERO torque (limp, same as IDLE) purely to
  make the encoder talk, reads, and only then enters POSITION mode.
- **`mount_rpy` must be applied on the BODY side.** `imu_node` used to do
  `quat_mul(q_mount, quat)` — a WORLD-frame rotation. That can still be tuned
  to make a level robot read zero, so it passes calibration, but it does not
  correct the SIGN of pitch as the robot tilts, and no value in the file can.
  Correct form is `quat_mul(quat, quat_conj(q_mount))`.
- **One calibration pose cannot determine the mount.** Level constrains it to
  a family of rotations, half of which read pitch BACKWARDS. Use
  `imu_calibrate.py --solve`, which takes a level pose AND a nose-down pose.
- **A wrong `driver:` fails SILENTLY.** The BNO085 is wired for **I2C** (SDA
  pin 3, SCL pin 5). With `driver:'uart'` the port opens, nothing ever
  arrives, the node logs nothing and `/imu` stays empty — indistinguishable
  from "still starting". `imu_node` now errors after 3s with no data.
  Related trap: every working run used `-p driver:=i2c` on the command line,
  which silently disappeared the moment we switched to `--params-file`.
- **`/dev/serial0` does not exist on Ubuntu** (it is a Raspberry Pi OS udev
  convention). On a Pi 5 use `/dev/ttyAMA0`; `ttyAMA10` is the DEBUG uart and
  opening it succeeds while returning nothing forever.
- **Verify params at RUNTIME, not in the file**: `ros2 param get /imu_node
  driver`. The file, the launch override and the CLI override disagree often
  enough that reading the file proves nothing.

- **Swapped CAN-H/CAN-L → totally silent bus** (zero RX, zero errors). On slcan,
  a bitrate mismatch AND an H/L swap BOTH look like silence — slcan doesn't
  surface bus errors to the kernel. So "no errors" does NOT rule out wiring.
  **Check H/L orientation first when silent.**
- **Over-termination → silent bus.** More than two 120 Ω terminators drops
  impedance below what the transceivers can drive → zero frames. Multimeter
  across CAN-H↔CAN-L (power OFF) must read **~60 Ω**. The GDS68 has a *software*
  terminator (`odrv0.can.config.enable_r120`): **ON only for the two bus ENDS
  (the wheels), OFF for the middle (the hips).**
- **The CANable can wedge** after many `slcand` teardowns → **power-cycle its
  USB** to clear. Always `pkill slcand` BEFORE `ip link delete can0`.
- **Config over USB, not CAN** — `enable_r120`/node_id etc. are reliable via
  `odrivetool` USB; over CAN they need fiddly SDO on this fw.
- **`save_configuration()` silently fails if the axis has a live error** — after
  setting a value, `dump_errors`, `clear_errors()`, set again, save, then
  **reconnect and read it back** to confirm it stuck.
- **The `/dev/ttyACMx` number can move** on replug — always `ls /dev/ttyACM*`.

---

## WHAT'S NEXT — staged bring-up, ON A STAND, in this order

**Do NOT just launch the whole stack.** Each step is powered motion; verify one
before the next. Robot on a stand, low currents, hand near the power cut.

1. **Legs.** `leg_controller` now **reads before it arms**: it waits for
   encoder feedback, establishes the zero, seeds its ramp at the MEASURED leg
   position, and only then closes the loop. It still moves the legs (it ramps
   to `home`), but it can no longer step them. It **refuses to arm** if there
   is no feedback within `arm_timeout` or if the two legs disagree by more
   than `max_leg_mismatch`.

   **Travel is capped at 0.10 output turns, not 0.19.** Three ceilings, in
   increasing order of how much they hurt:
   - `0.1937` turns is the linkage toggle point (`sin` peaks at 90°) past
     which more turns means LESS extension and the map inverts.
   - `1/gear_ratio = 0.125` turns is ONE MOTOR TURN. The MA732 is single-turn
     absolute, so travel must fit inside one turn or two leg heights alias to
     the same boot reading and a homing move becomes mandatory.
   - the **dead band** left over after travel is the entire error budget for
     backlash, wind-up and noise when unwrapping the boot reading — and it is
     measured at the MOTOR, before the gearbox. At `pos_max` 0.12 that is 7°
     of motor rotation. At 0.10 it is 36°.

   So usable travel is `ext = 0.462·sin(20.28° + 360°·turns)` m = **0.160 m
   retracted → 0.384 m extended**, 0.224 m of lift. Step 0 below can buy some
   of that back.

0. **[DO THIS FIRST — it decides the travel limit] The power-cycle test.**
   Everything above assumes the fork firmware re-derives `pos_estimate` from
   the single-turn absolute reading at every boot. **Verify it, don't assume
   it.** With one leg on the bench:
   ```bash
   # park the leg somewhere mid-stroke, note pos_estimate, power cycle, re-read
   candump can0 &
   # ... or in odrivetool:  odrv0.axis0.encoder.pos_estimate
   ```
   Repeat at three or four leg positions spread across the stroke, including
   past one full motor turn from the stop.
   - If `pos_estimate` comes back **wrapped into a one-turn window** →
     everything above holds; keep `pos_max: 0.10`.
   - If it comes back as a **persisted multi-turn count** → none of the
     single-turn reasoning binds. Raise `pos_max` toward 0.19 (still under the
     toggle point) and you get the full 0.302 m of lift back.
   - If it comes back as **0.0 every time** → there is no absolute restore at
     all, and you need a real homing routine. Say so and we'll write one.

   **The one-time zero measurement** (do this first, then never again):
   ```bash
   # legs physically against the RETRACTED hard stop, use_measured_zero false
   ros2 run robot_base leg_controller --ros-args \
     --params-file ~/BipedV1/src/robot_bringup/config/real.yaml \
     -p can_channel:=can0 -p current_limit:=2.0
   # the node logs a WARN containing the raw values it took as zero.
   # paste those into zero_raw_left / zero_raw_right in real.yaml,
   # set use_measured_zero: true, done.
   ```
   Until that flag is set the node treats **boot posture as zero**. That is
   right on the ground (weight collapses the legs onto the stop) but **WRONG
   on a stand**, where the legs hang extended — so push them onto the stop
   before starting, or the soft limits guard the wrong range.
2. **Verify `invert_left`/`invert_right` OPEN-LOOP** at the lowest torque,
   before any loop closes. A wrong wheel sign makes the balance loop *add* energy
   to a fall.
3. **IMU.** Bring up `imu_node`, calibrate **`mount_rpy`** (tilt the robot,
   confirm pitch reads the right sign and zeros level).
4. **Wheels closed-loop + `balance_controller` LAST.** Low `max_torque`,
   tethered. Cross-check `max_torque ≤ current_limit·(8.27/motor_kv)·gear_ratio`.

---

## SAFETY INVARIANTS (non-negotiable)

- **On a stand / tethered** until balance is proven.
- **Know the physical power cut** before energizing.
- **DISABLED = legs COLLAPSE** (position mode) — never disable with weight on legs.
- **Wrong `invert_*` on wheels = balance drives INTO the fall.** Verify open-loop.
- **Calibration MOVES the motor** — decouple or ensure clearance, low current.
- **Never jog the assembled leg from MotorWizard / raw position commands** — it
  bypasses `leg_controller`'s clamp/ramp/travel-limits (this is what broke the
  3DP linkage parts once).

---

## REFERENCE

- **Vendor manual + MotorWizard:** `GDS68 Driver.zip` from
  <https://steadywin-motor.com/products/document-download> (English manual:
  _SteadyWin GIM6010-8 Motor Manual rev2.2_ — it's an ODrive-derived doc).
- **Config file:** `src/robot_bringup/config/real.yaml` (node IDs, can_channel,
  limits, `mount_rpy` placeholder, etc.). `[SATURDAY]`-tagged values are the
  ones still to confirm on hardware (`motor_kv`, `pos_min/max`, `mount_rpy`).
- **Memory files** (auto-loaded for Claude): `hardware-bringup-checkpoint`,
  `odrive-can-protocol`, `leg-forward-kinematics`, `slcan-zombie-interface`,
  `leg-position-hold`, `mode-manager-design`.
