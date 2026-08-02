# BipedV1 — Project Brief for Claude Code

## What this project is
A wheeled biped self-balancing robot (Ascento-inspired): two parallel-linkage
legs ending in wheels, driven by four GIM8108 hub motors over CAN. It balances
like an inverted pendulum — it is NOT a static rover. Every design and code
decision must be evaluated through dynamic balance first.

## The developer
Beginner in ROS 2 and programming. The #1 goal is his INDEPENDENT CAPABILITY —
understanding ROS 2 well enough to design future robots on his own. Shipping a
working robot is secondary to him learning why things work.

## Hardware / compute split
**CHANGED 2026-07-22 — the balance loop moved OFF the Pico onto the Pi.**
The old plan (Pico runs balance + IMU + CAN via micro-ROS) was dropped so the
real robot could be coded and validated in simulation first; the Pico path
can't meaningfully be tested in Gazebo and would be a rewrite in C rather than
a port. Accepted trade-off: Linux is not hard real-time, so loop jitter is a
real risk. Fallback if it bites: port the by-then-proven loop to Pico firmware.

- **Raspberry Pi 5** (the brain, and now the controller): the balance loop
  (`robot_base/balance_controller`), `odrive_bridge` (CAN to the motors),
  `imu_node`, twist_mux, joystick node, mode_manager, rosbridge + dashboard,
  RPLidar, later Nav2/SLAM. Auto-starts on boot via systemd.
- **Motors**: driven by **ODrive** controllers over CAN at **500 kbps** — NOT
  the MIT/Mini-Cheetah protocol. Torque (effort) mode; gear ratio 8. Frame
  IDs, enums and the motor-vs-output unit traps are documented in the memory
  file `odrive-can-protocol`.
- **RP2040 Pico**: currently UNUSED. Retained as an option for a future
  real-time port of the balance loop, or as a dedicated sensor board.
- **BNO085 (BNO08x) IMU**: onboard fusion, gives a quaternion directly.
  Prefer **UART or SPI** — the BNO08x uses I2C clock stretching, which the
  Pi's hardware I2C handles badly. Mounting rotation MUST be calibrated
  (`imu_node`'s `mount_rpy`) before torque is ever enabled.
- **Ubuntu laptop** (driver station): joystick + dashboard. Fully OPTIONAL —
  nothing on the robot depends on it. The dashboard is a web page, so a phone
  works too.
- **CANable 2.0** (USB-CAN adapter): was a bench-only tool, but with the loop
  on the Pi it is now a candidate for the robot's production CAN path. NOTE
  the fragility: it runs through `slcand`, and a killed daemon or a yanked USB
  cable leaks a zombie netdev that squats the interface name (see the memory
  file `slcan-zombie-interface`). A permanent MCP2515-class CAN HAT on the Pi's
  SPI bus is the more robust production choice — decide before final assembly.
- **CAN bus wiring note**: exactly two 120Ω termination resistors, one at
  each PHYSICAL END of the bus — not one per device. A single-motor bench
  test (CANable + motor) is a two-node bus and both ends get terminated.
  On the real robot only the two devices at the physical ends are terminated,
  not every motor.

## Software goals (what "done" looks like)
1. **Standalone**: power on and the robot balances and works with zero network
   dependency. The Pi auto-launches the whole stack on boot.
2. **Optional driver station**: robot always publishes telemetry and accepts
   cmd_vel. When the laptop appears on the network, DDS discovery connects it
   automatically. When it's gone, the robot doesn't notice.
3. **Control arbitration**: teleop or autonomous. The PHYSICAL PS4 controller
   is the ONLY thing that drives the robot. The intended sequence is: turn on
   the robot, turn on the pad, press a button to switch to TELEOP, drive.
   twist_mux arbitrates teleop vs autonomous by priority + timeout.
4. **Dashboard**: see robot state, battery voltage, current draw; change modes.
   OPTIONAL and read-mostly — a convenience for switching modes and watching
   readouts. **It does not drive the robot.** No on-screen sticks.
5. **Maintainable**: easy to add to, change, and navigate. Features are added
   by adding NODES, not editing a monolith. Every tunable is a YAML param,
   never hardcoded.

## Safety invariant (non-negotiable)
The command-timeout watchdog lives in `balance_controller` (on the Pi, since
2026-07-22). On loss of cmd_vel it zeros translational/rotational velocity BUT
KEEPS BALANCING. Never cut motors on signal loss — that drops the robot.

Three distinct behaviours that must never be merged into one code path:
1. **Stale cmd_vel** → zero the velocity references, keep balancing.
2. **DISABLED mode** → deliberately cut torque. Only ever from an EXPLICIT
   command, never automatically, because it drops the robot.
3. **Stale wheel-torque commands** (`odrive_bridge`) → coast. This means the
   controller itself died, so there is nothing left to balance with. Safe on a
   stand; revisit before the robot runs untethered.

## Workspace layout
BipedV1/ is both the git repo and the ROS 2 workspace root. Packages live in
src/. Build from the root with colcon; never commit build/, install/, or log/.
- robot_bringup     — launch files + YAML config; single entry point
                      (sim.launch.py / real.launch.py — identical SHARED block)
- robot_description — URDF/xacro + Gazebo worlds
- robot_teleop      — joy, twist_mux, mode_manager
- robot_base        — balance_controller, odrive_bridge, imu_node, odrive_can
- robot_interfaces  — custom msgs/srvs (SetMode)
- robot_dashboard   — rosbridge + web dashboard (browser/phone)
- robot_navigation  — Nav2/SLAM (later)
- tools/            — non-ROS dev tools: fake_odrive + no-hardware tests
- deploy/           — non-ROS production deployment: the systemd units that
                      make the robot start itself (CAN + the ROS stack),
                      their config, and TEST 7/8. See deploy/README.md.

## Environment
ROS 2 Jazzy on Ubuntu 24.04 (WSL2 on the dev laptop). Staying on Jazzy —
Lyrical Luth ecosystem (slam_toolbox, micro-ROS) isn't ready yet.

## How to help me (collaboration rules)
- DEFAULT to explaining, not writing. Don't write implementation code unless
  I explicitly ask.
- Explain the concept and the "why" first, then let me write it.
- When I'm stuck, give a HINT or a leading question — not the answer.
- Boilerplate is fair game to write directly (package.xml, setup.py,
  CMakeLists, colcon config, scaffolding).
- After any code, ask 1–2 questions to check I understand it.
- For errors, explain what they MEAN and how to reason about them before fixing.