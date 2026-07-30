"""A fake ODrive that speaks the real CAN protocol, so driver code can be
developed and tested with NO motor, NO CANable, and NO robot.

Run it against a virtual CAN bus in one terminal, point motor_test.py at the
same bus in another, and the whole command/feedback path is exercised for real
— framing, byte packing, gear-ratio conversions, mode transitions.

    sudo modprobe vcan
    sudo ip link add dev vcan0 type vcan && sudo ip link set up vcan0
    python3 tools/fake_odrive.py --channel vcan0

It deliberately models the three behaviours that bite in torque mode:
  * zero torque COASTS (friction only) — it does not brake
  * torque mode has no inherent speed limit unless the vel-limit guard is on
    (run with --no-vel-limit to watch it run away, safely)
  * feedback only arrives if you ask for it (RTR) or enable cyclic broadcast

Everything on the wire is MOTOR-shaft, exactly like the real thing.
"""
import argparse
import math
import os
import struct
import sys
import threading
import time

import can

# import the protocol module from the source tree, so this dev tool works
# whether or not the colcon workspace has been sourced
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                '..', 'src', 'robot_base'))

from robot_base.odrive_can import (
    CMD_GET_BUS_VI, CMD_GET_ENCODER_ESTIMATES, CMD_GET_IQ, CMD_GET_TORQUES,
    CMD_SET_AXIS_STATE, CMD_SET_CONTROLLER_MODES, CMD_SET_INPUT_POS,
    CMD_SET_INPUT_TORQUE, CMD_SET_INPUT_VEL, CMD_SET_LIMITS,
    AxisState, ControlMode, arb_id, torque_constant,
)

# --- plausible bench motor. SET MOTOR_KV TO THE REAL VALUE before trusting
#     any current/torque number this produces. ---
MOTOR_KV = 100.0
INERTIA = 0.0015        # kg*m^2, motor shaft
VISCOUS = 0.0008        # Nm per rad/s
COULOMB = 0.010         # Nm, constant friction
BUS_VOLTAGE = 24.0


class FakeODrive:
    def __init__(self, bus, node_id=1, vel_limit_guard=True):
        self.bus = bus
        self.node_id = node_id
        self.vel_limit_guard = vel_limit_guard

        self.axis_state = AxisState.IDLE
        self.control_mode = ControlMode.VELOCITY
        self.input_mode = 1
        self.vel_setpoint = 0.0      # motor turns/s
        self.torque_setpoint = 0.0   # Nm, motor shaft
        self.pos_setpoint = 0.0      # motor turns
        self.pos = 0.0               # motor turns
        self.vel = 0.0               # motor turns/s
        self.vel_limit = 160.0       # motor turns/s
        self.current_limit = 12.0
        self.applied_torque = 0.0
        self.kt = torque_constant(MOTOR_KV)

        # Position-loop gains. On a REAL ODrive these live in odrivetool
        # (axis0.controller.config.pos_gain / vel_gain / vel_integrator_gain),
        # not in the basic CAN command set — set them there, not over CAN.
        self.pos_gain = 20.0             # (turns/s) per turn of error
        self.vel_gain = 0.05             # Nm per (turn/s) of error  <- DAMPING
        self.vel_integrator_gain = 0.0   # 0 = pure P: a constant load WILL sag
        self._vel_integrator = 0.0

        # Constant opposing torque, i.e. the robot's weight on a leg. This is
        # what makes "can it hold its weight?" answerable without hardware.
        self.load_torque = 0.0           # Nm at the motor shaft

        # see feedback_frame(): real fw only reports encoder data in CLOSED_LOOP
        self.closed_loop_feedback_only = True

        self.running = True
        self._lock = threading.Lock()

    # ── physics ───────────────────────────────────────────────────────────────
    def step(self, dt):
        with self._lock:
            if self.axis_state != AxisState.CLOSED_LOOP_CONTROL:
                tau_cmd = 0.0                      # IDLE coasts
            elif self.control_mode == ControlMode.TORQUE:
                tau_cmd = self.torque_setpoint
                # ODrive's enable_torque_mode_vel_limit. Without it, constant
                # torque on an unloaded motor accelerates without bound.
                if self.vel_limit_guard and abs(self.vel) > self.vel_limit:
                    if math.copysign(1, tau_cmd) == math.copysign(1, self.vel):
                        tau_cmd = 0.0
            elif self.control_mode == ControlMode.VELOCITY:
                # a real closed-loop velocity controller producing torque
                tau_cmd = 0.02 * (self.vel_setpoint - self.vel)
            elif self.control_mode == ControlMode.POSITION:
                # ODrive's cascade: position P -> velocity PI -> torque.
                err = self.pos_setpoint - self.pos
                vel_sp = self.pos_gain * err
                vel_sp = max(-self.vel_limit, min(self.vel_limit, vel_sp))
                vel_err = vel_sp - self.vel
                self._vel_integrator += self.vel_integrator_gain * vel_err * dt
                tau_cmd = self.vel_gain * vel_err + self._vel_integrator
            else:
                tau_cmd = 0.0

            tau_max = self.current_limit * self.kt
            tau_cmd = max(-tau_max, min(tau_max, tau_cmd))

            omega = self.vel * 2 * math.pi
            friction = VISCOUS * omega + (COULOMB * math.copysign(1, omega) if abs(omega) > 1e-6 else 0.0)
            # load_torque is the robot's weight pulling the leg down
            alpha = (tau_cmd - friction - self.load_torque) / INERTIA
            omega += alpha * dt
            # friction must not reverse the shaft, only stop it
            if abs(tau_cmd) < COULOMB and abs(omega) < 0.05:
                omega = 0.0
            self.vel = omega / (2 * math.pi)
            self.pos += self.vel * dt
            self.applied_torque = tau_cmd

    # ── wire ──────────────────────────────────────────────────────────────────
    def _reply(self, cmd, data):
        self.bus.send(can.Message(arbitration_id=arb_id(self.node_id, cmd),
                                  data=data, is_extended_id=False))

    def feedback_frame(self, cmd):
        with self._lock:
            if cmd == CMD_GET_ENCODER_ESTIMATES:
                # CONFIRMED on hardware 2026-07-30: this fork reports encoder
                # estimates as ZEROS unless the axis is in CLOSED_LOOP. Driver
                # code that reads position before arming gets a convincing
                # "everything is at the origin", so model it here.
                if (self.closed_loop_feedback_only
                        and self.axis_state != AxisState.CLOSED_LOOP_CONTROL):
                    return struct.pack('<ff', 0.0, 0.0)
                return struct.pack('<ff', self.pos, self.vel)
            if cmd == CMD_GET_IQ:
                iq = self.applied_torque / self.kt
                return struct.pack('<ff', iq, iq)
            if cmd == CMD_GET_BUS_VI:
                mech_w = abs(self.applied_torque * self.vel * 2 * math.pi)
                return struct.pack('<ff', BUS_VOLTAGE, mech_w / BUS_VOLTAGE)
            if cmd == CMD_GET_TORQUES:
                return struct.pack('<ff', self.torque_setpoint, self.applied_torque)
        return None

    def handle(self, msg):
        if msg.arbitration_id >> 5 != self.node_id:
            return
        cmd = msg.arbitration_id & 0x1F
        if msg.is_remote_frame:                      # RTR read (fw 0.5.x)
            frame = self.feedback_frame(cmd)
            if frame:
                self._reply(cmd, frame)
            return
        d = bytes(msg.data)
        with self._lock:
            if cmd == CMD_SET_AXIS_STATE and len(d) >= 4:
                self.axis_state = struct.unpack('<I', d[:4])[0]
                if self.axis_state != AxisState.CLOSED_LOOP_CONTROL:
                    self.vel_setpoint = self.torque_setpoint = 0.0
            elif cmd == CMD_SET_CONTROLLER_MODES and len(d) >= 8:
                self.control_mode, self.input_mode = struct.unpack('<II', d[:8])
                self.vel_setpoint = self.torque_setpoint = 0.0
            elif cmd == CMD_SET_INPUT_VEL and len(d) >= 8:
                self.vel_setpoint, _ = struct.unpack('<ff', d[:8])
            elif cmd == CMD_SET_INPUT_TORQUE and len(d) >= 4:
                # 4 bytes, not 8
                self.torque_setpoint = struct.unpack('<f', d[:4])[0]
            elif cmd == CMD_SET_LIMITS and len(d) >= 8:
                self.vel_limit, self.current_limit = struct.unpack('<ff', d[:8])
            elif cmd == CMD_SET_INPUT_POS and len(d) >= 8:
                pos, vel_ff, tq_ff = struct.unpack('<fhh', d[:8])
                self.pos_setpoint = pos          # motor turns
                # the two int16 feedforwards are scaled 0.001 per count
                self._vel_ff = vel_ff * 0.001
                self._torque_ff = tq_ff * 0.001

    # ── loops ─────────────────────────────────────────────────────────────────
    def run(self, cyclic_ms=0):
        """Blocking. cyclic_ms > 0 also broadcasts feedback (fw 0.6.x style)."""
        last = time.time()
        next_cyclic = last
        while self.running:
            msg = self.bus.recv(timeout=0.005)
            if msg:
                self.handle(msg)
            now = time.time()
            dt = now - last
            if dt >= 0.002:
                self.step(dt)
                last = now
            if cyclic_ms and now >= next_cyclic:
                for c in (CMD_GET_ENCODER_ESTIMATES, CMD_GET_IQ):
                    self._reply(c, self.feedback_frame(c))
                next_cyclic = now + cyclic_ms / 1000.0


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--interface', default='socketcan')
    ap.add_argument('--channel', default='vcan0')
    ap.add_argument('--node-id', type=int, default=1)
    ap.add_argument('--cyclic-ms', type=int, default=0,
                    help='>0 broadcasts feedback instead of waiting for RTR')
    ap.add_argument('--no-vel-limit', action='store_true',
                    help='disable the torque-mode velocity guard (watch it run away)')
    ap.add_argument('--load-torque', type=float, default=0.0,
                    help='constant opposing torque at the motor (Nm) — the '
                         'robot weight on a leg. Use to test position hold.')
    ap.add_argument('--vel-gain', type=float, default=None,
                    help='position-loop damping (Nm per turn/s of error)')
    ap.add_argument('--vel-integrator-gain', type=float, default=None,
                    help='>0 removes steady-state sag under load')
    a = ap.parse_args()

    bus = can.interface.Bus(interface=a.interface, channel=a.channel, bitrate=500000)
    drv = FakeODrive(bus, node_id=a.node_id, vel_limit_guard=not a.no_vel_limit)
    drv.load_torque = a.load_torque
    if a.vel_gain is not None:
        drv.vel_gain = a.vel_gain
    if a.vel_integrator_gain is not None:
        drv.vel_integrator_gain = a.vel_integrator_gain
    print(f"fake ODrive: node {a.node_id} on {a.interface}:{a.channel}  "
          f"kt={drv.kt:.4f} Nm/A  vel-guard={'off' if a.no_vel_limit else 'on'}  "
          f"load={drv.load_torque:.3f} Nm  vel_gain={drv.vel_gain}  "
          f"vel_i={drv.vel_integrator_gain}")
    try:
        drv.run(cyclic_ms=a.cyclic_ms)
    except KeyboardInterrupt:
        pass
    finally:
        drv.running = False
        bus.shutdown()


if __name__ == '__main__':
    main()
