"""ODrive CAN protocol for BipedV1 — shared by the bench script and the simulator.

Bus is 500 kbps, standard 11-bit IDs: arbitration_id = (NODE_ID << 5) | CMD_ID.

EVERY ODrive quantity on the wire is MOTOR-shaft. This module is the single
place the gear-ratio conversions live, because they run in OPPOSITE directions:

    velocity/position:  motor = output * GEAR_RATIO
    torque:             motor = output / GEAR_RATIO   <- a gearbox MULTIPLIES torque

Copy-pasting the velocity pattern into a torque function is wrong by
GEAR_RATIO**2 (64x at ratio 8) and in the wrong direction. Keep the conversions
here so there is exactly one place to get them right.
"""
import struct

# ── command frames (host -> ODrive) ───────────────────────────────────────────
CMD_SET_AXIS_STATE        = 0x007
CMD_GET_ENCODER_ESTIMATES = 0x009
CMD_SET_CONTROLLER_MODES  = 0x00B
CMD_SET_INPUT_POS         = 0x00C
CMD_SET_INPUT_VEL         = 0x00D
CMD_SET_INPUT_TORQUE      = 0x00E
CMD_SET_LIMITS            = 0x00F
CMD_GET_IQ                = 0x014
CMD_GET_BUS_VI            = 0x017
CMD_GET_TORQUES           = 0x01C   # firmware 0.6.x ONLY


class AxisState:
    IDLE = 1
    CLOSED_LOOP_CONTROL = 8


class ControlMode:
    VOLTAGE = 0
    TORQUE = 1
    VELOCITY = 2
    POSITION = 3


class InputMode:
    INACTIVE = 0
    PASSTHROUGH = 1
    VEL_RAMP = 2
    POS_FILTER = 3
    MIX_CHANNELS = 4
    TRAP_TRAJ = 5
    TORQUE_RAMP = 6
    MIRROR = 7
    TUNING = 8


# the two int16 feedforward fields in Set_Input_Pos are scaled 0.001 per count
POS_FF_SCALE = 0.001


def arb_id(node_id, cmd):
    return (node_id << 5) | cmd


def torque_constant(motor_kv):
    """Nm per amp of Iq. torque = Iq * torque_constant."""
    return 8.27 / motor_kv


class ODriveClient:
    """Thin command/decode layer. Public API is in OUTPUT-shaft units."""

    def __init__(self, bus, node_id=1, gear_ratio=8):
        self.bus = bus
        self.node_id = node_id
        self.gear = gear_ratio

    # ── raw send ──────────────────────────────────────────────────────────────
    def _send(self, cmd, data):
        import can
        self.bus.send(can.Message(arbitration_id=arb_id(self.node_id, cmd),
                                  data=data, is_extended_id=False))

    def request(self, cmd):
        """RTR read (firmware 0.5.x). On 0.6.x prefer cyclic broadcast."""
        import can
        self.bus.send(can.Message(arbitration_id=arb_id(self.node_id, cmd),
                                  is_remote_frame=True, is_extended_id=False, dlc=8))

    # ── commands ──────────────────────────────────────────────────────────────
    def set_axis_state(self, state):
        self._send(CMD_SET_AXIS_STATE, struct.pack('<I', state))

    def set_controller_modes(self, control_mode, input_mode):
        self._send(CMD_SET_CONTROLLER_MODES, struct.pack('<II', control_mode, input_mode))

    def set_limits(self, vel_limit_output, current_limit_a):
        # vel limit is a VELOCITY -> multiply by the gear ratio
        self._send(CMD_SET_LIMITS,
                   struct.pack('<ff', vel_limit_output * self.gear, current_limit_a))

    def set_velocity(self, vel_output, torque_ff_output=0.0):
        # velocity: motor = output * gear    torque_ff: motor = output / gear
        self._send(CMD_SET_INPUT_VEL,
                   struct.pack('<ff', vel_output * self.gear, torque_ff_output / self.gear))

    def set_torque(self, torque_output):
        """torque_output is Nm at the OUTPUT shaft. Note the DIVIDE."""
        self._send(CMD_SET_INPUT_TORQUE, struct.pack('<f', torque_output / self.gear))

    def set_input_pos(self, pos_output, vel_ff_output=0.0, torque_ff_output=0.0):
        self._send(CMD_SET_INPUT_POS, struct.pack(
            '<fhh',
            pos_output * self.gear,
            int(round(vel_ff_output * self.gear / POS_FF_SCALE)),
            int(round(torque_ff_output / self.gear / POS_FF_SCALE)),
        ))

    # ── decode ────────────────────────────────────────────────────────────────
    def decode(self, msg):
        """-> (name, values-in-OUTPUT-units) or None if not ours."""
        if msg.arbitration_id >> 5 != self.node_id or msg.is_remote_frame:
            return None
        cmd = msg.arbitration_id & 0x1F
        d = bytes(msg.data)
        if cmd == CMD_GET_ENCODER_ESTIMATES and len(d) >= 8:
            pos, vel = struct.unpack('<ff', d[:8])
            return ('encoder', (pos / self.gear, vel / self.gear))
        if cmd == CMD_GET_IQ and len(d) >= 8:
            return ('iq', struct.unpack('<ff', d[:8]))
        if cmd == CMD_GET_BUS_VI and len(d) >= 8:
            return ('bus', struct.unpack('<ff', d[:8]))
        if cmd == CMD_GET_TORQUES and len(d) >= 8:
            tgt, est = struct.unpack('<ff', d[:8])
            # torques come back motor-shaft -> multiply up to the output
            return ('torques', (tgt * self.gear, est * self.gear))
        return None
