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
- **RP2040 Pico** (real-time): balance loop, IMU, CAN to the 4 motors via a
  Waveshare CAN Bus module (SPI, MCP2515-class) — this is the robot's
  ONBOARD production CAN path. Runs micro-ROS. Subscribes to a velocity
  setpoint; publishes telemetry (IMU, motor current/voltage, battery). Never
  depends on the network.
- **Raspberry Pi 5** (the brain): micro-ROS agent, twist_mux, joystick node,
  mode_manager, RPLidar, later Nav2/SLAM. Auto-starts on boot via systemd.
- **Ubuntu laptop** (driver station): joystick + dashboard. Fully OPTIONAL —
  nothing on the robot depends on it.
- **CANable 2.0** (USB-CAN adapter): BENCH/DEV TOOL ONLY — plugs into a
  laptop or the Pi over USB (shows up as SocketCAN `can0`) for talking
  directly to the GIM8108 motors via `candump`/`cansend`/python-can. Used
  for testing and tuning motors independently of Pico firmware. NOT part
  of the robot's onboard production CAN path.
- **CAN bus wiring note**: exactly two 120Ω termination resistors, one at
  each PHYSICAL END of the bus — not one per device. A single-motor bench
  test (CANable + motor) is a two-node bus and both ends get terminated.
  On the real robot (Pico + 4 motors daisy-chained), only the two devices
  at the physical ends are terminated, not all four motors.

## Software goals (what "done" looks like)
1. **Standalone**: power on and the robot balances and works with zero network
   dependency. Pico runs on power; Pi auto-launches the stack.
2. **Optional driver station**: robot always publishes telemetry and accepts
   cmd_vel. When the laptop appears on the network, DDS discovery connects it
   automatically. When it's gone, the robot doesn't notice.
3. **Control arbitration**: teleop or autonomous. Driver-station teleop is
   PREFERRED but not mandatory — a controller plugged directly into the Pi is
   a fallback. twist_mux handles this by priority + timeout.
4. **Dashboard**: see robot state, battery voltage, current draw; change modes.
5. **Maintainable**: easy to add to, change, and navigate. Features are added
   by adding NODES, not editing a monolith. Every tunable is a YAML param,
   never hardcoded.

## Safety invariant (non-negotiable)
The command-timeout watchdog lives on the Pico. On loss of cmd_vel, it zeros
translational/rotational velocity BUT KEEPS BALANCING. Never cut motors — that
drops the robot.

## Workspace layout
BipedV1/ is both the git repo and the ROS 2 workspace root. Packages live in
src/. Build from the root with colcon; never commit build/, install/, or log/.
- robot_bringup     — launch files + YAML config; single entry point
- robot_description — URDF/xacro
- robot_teleop      — joy, twist_mux, mode_manager
- robot_base        — micro-ROS bridge to the Pico
- robot_navigation  — Nav2/SLAM (later)

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