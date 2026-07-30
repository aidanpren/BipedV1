"""Position control for the two LEG ODrives (the non-wheel motors).

Separate node from odrive_bridge on purpose: different control mode, different
failure behaviour, and SocketCAN happily gives both nodes their own socket on
the same interface.

    subscribes  leg_position_cmd   Float64  target leg position, OUTPUT turns
    publishes   leg_states         JointState (left_leg_joint, right_leg_joint)

SAFETY, because these carry the robot's weight:
  * commands are clamped to [pos_min, pos_max] — soft travel limits
  * the setpoint is RAMPED here at max_speed rather than stepped, so a loaded
    leg is never slammed. This does not depend on the ODrive's input_mode
    config, which we cannot verify from here.
  * we do NOT arm until the encoders have told us where the legs actually are,
    and the ramp is then SEEDED at that measured position. Seeding it at the
    home target instead makes the first frame on the bus a full-magnitude
    position step — the ramp only limits how fast the setpoint CHANGES, so a
    setpoint that already sits on its target ramps nothing at all.
  * torque cut == COLLAPSE for a leg, unlike a wheel where it just coasts.
    Read the note on shutdown() before wiring this into DISABLED.

ZEROING (why there is no homing routine). The MA732 is 14-bit single-turn
ABSOLUTE at the MOTOR, so a boot-time reading pins the rotor to within one
motor turn — but only one. Keep usable travel under one motor turn
(1/gear_ratio = 0.125 output turns) and that reading is UNAMBIGUOUS: measure
the raw estimate at the retracted hard stop once, put it in real.yaml as
zero_raw_*, and the robot never has to home again, in any posture. Exceed one
motor turn of travel and two different leg heights alias to the same encoder
value, at which point a boot-time homing move becomes mandatory.

Dry-run with no hardware (note --load-torque: that is the robot's weight):
    python3 tools/fake_odrive.py --channel vcan0 --node-id 3 --load-torque 0.15 &
    python3 tools/fake_odrive.py --channel vcan0 --node-id 4 --load-torque 0.15 &
    ros2 run robot_base leg_controller --ros-args -p can_channel:=vcan0
"""
import math
import threading
import time

import can
import rclpy
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64

from robot_base.odrive_can import (
    CMD_GET_ENCODER_ESTIMATES, AxisState, ControlMode, InputMode, ODriveClient,
)


class LegController:
    def __init__(self):
        self.node = rclpy.create_node('leg_controller')

        self.node.declare_parameter('can_interface', 'socketcan')
        self.node.declare_parameter('can_channel', 'vcan0')
        self.node.declare_parameter('bitrate', 500000)
        self.node.declare_parameter('left_node_id', 3)
        self.node.declare_parameter('right_node_id', 4)
        self.node.declare_parameter('gear_ratio', 8.0)
        self.node.declare_parameter('invert_left', False)
        self.node.declare_parameter('invert_right', True)
        self.node.declare_parameter('current_limit', 12.0)
        self.node.declare_parameter('vel_limit', 5.0)        # output turns/s
        self.node.declare_parameter('publish_rate', 50.0)
        # Soft travel limits, OUTPUT turns, measured from the retracted hard
        # stop at 0.0. MEASURE THESE ON THE REAL LINKAGE. pos_max must stay
        # under 1/gear_ratio (0.125) to keep the boot reading unambiguous, and
        # under the 0.1937-turn linkage toggle point past which extension
        # DECREASES again with increasing turns.
        self.node.declare_parameter('pos_min', 0.0)
        self.node.declare_parameter('pos_max', 0.12)
        self.node.declare_parameter('home_position', 0.0)
        self.node.declare_parameter('max_speed', 0.15)        # output turns/s
        # 1 PASSTHROUGH, 3 POS_FILTER, 5 TRAP_TRAJ. 3/5 need extra config in
        # odrivetool; the node-side ramp below protects us either way.
        self.node.declare_parameter('input_mode', InputMode.POS_FILTER)
        self.node.declare_parameter('idle_on_shutdown', True)
        # how long to wait for the first encoder frame before refusing to arm
        self.node.declare_parameter('arm_timeout', 3.0)
        # ZEROING, see the module docstring. zero_raw_* is the RAW ODrive
        # pos_estimate (output turns) observed with that leg against its
        # RETRACTED hard stop. Measure once on hardware, then set
        # use_measured_zero -- until you do, the node falls back to treating
        # boot posture as zero, which is only correct if the legs really are
        # sitting on the stop.
        self.node.declare_parameter('use_measured_zero', False)
        self.node.declare_parameter('zero_raw_left', 0.0)
        self.node.declare_parameter('zero_raw_right', 0.0)
        # if the two legs disagree by more than this at boot, something is
        # wrong (bad zero constant, slipped linkage) -- refuse to arm
        self.node.declare_parameter('max_leg_mismatch', 0.03)   # output turns
        # smallest dead band (output turns, either side of travel) we consider
        # a trustworthy error budget for unwrapping the boot reading
        self.node.declare_parameter('min_unwrap_margin', 0.01)

        def p(name):
            return self.node.get_parameter(name).value

        self.pos_min, self.pos_max = p('pos_min'), p('pos_max')
        self.max_speed = p('max_speed')
        self.min_unwrap_margin = p('min_unwrap_margin')
        self.idle_on_shutdown = p('idle_on_shutdown')
        self.invert = {'left': -1.0 if p('invert_left') else 1.0,
                       'right': -1.0 if p('invert_right') else 1.0}

        self.bus = can.interface.Bus(interface=p('can_interface'),
                                     channel=p('can_channel'),
                                     bitrate=p('bitrate'))
        gear = self.gear = p('gear_ratio')
        self.clients = {
            'left':  ODriveClient(self.bus, node_id=p('left_node_id'),  gear_ratio=gear),
            'right': ODriveClient(self.bus, node_id=p('right_node_id'), gear_ratio=gear),
        }

        self.fb = {'left': [0.0, 0.0], 'right': [0.0, 0.0]}   # output turns, turns/s
        self.fb_seen = {'left': False, 'right': False}        # 0.0 is a legit value
        self._lock = threading.Lock()

        self.state_pub = self.node.create_publisher(JointState, 'leg_states', 10)
        self.cmd_sub = self.node.create_subscription(
            Float64, 'leg_position_cmd', self.command_callback, 10)

        # ORDER MATTERS for the rest of this constructor, and it is the whole
        # safety story of startup: read, THEN zero, THEN seed, THEN arm. Nothing
        # can move until the last step, and by then we know where we are.
        self.running = True
        threading.Thread(target=self.can_reader, daemon=True).start()

        self.raw_zero = self.establish_zero(
            p('arm_timeout'), p('use_measured_zero'),
            {'left': p('zero_raw_left'), 'right': p('zero_raw_right')},
            p('max_leg_mismatch'))

        # Seed the ramp where the legs ACTUALLY are, then let it walk to home.
        # Deliberately NOT clamped: if a leg booted outside the soft limits we
        # want to ramp back INTO range, not step to the edge of it.
        start = self.measured_position()
        self.setpoint = 0.5 * (start['left'] + start['right'])
        self.target = self.clamp(p('home_position'))
        self.node.get_logger().info(
            f'legs at {start["left"]:+.4f}/{start["right"]:+.4f} turns; '
            f'ramping to home {self.target:+.4f} at {p("max_speed")} turns/s')

        self.arm(p('vel_limit'), p('current_limit'), p('input_mode'))

        self.rate = p('publish_rate')
        self.timer = self.node.create_timer(1.0 / self.rate, self.update)

    def clamp(self, v):
        return max(self.pos_min, min(self.pos_max, float(v)))

    def measured_position(self):
        """Where the legs are, in LOGICAL turns (0.0 = retracted hard stop).

        raw = raw_zero + invert * logical, so logical = invert * (raw - zero).
        """
        with self._lock:
            return {n: self.invert[n] * (v[0] - self.raw_zero[n])
                    for n, v in self.fb.items()}

    def unwrap_boot(self, delta):
        """Resolve a boot encoder delta that is only known modulo one motor turn.

        Travel occupies [pos_min, pos_max] out of a 1/gear_ratio-turn window.
        Whatever is left over is the DEAD BAND — positions the leg physically
        cannot reach. Anything landing there is really a slightly-negative
        position that wrapped, so we split the dead band down the middle and
        fold the upper half back below zero.

        The dead band is also our whole error budget: the closer pos_max gets
        to 1/gear_ratio, the less room there is for backlash, wind-up and
        encoder noise before a reading lands on the wrong side of the split.
        """
        window = 1.0 / self.gear                       # one motor turn, OUTPUT turns
        d = math.fmod(delta, window)
        if d < 0.0:
            d += window                                # now in [0, window)
        split = self.pos_max + 0.5 * (window - self.pos_max - self.pos_min)
        if d > split:
            d -= window
        return d

    # ── lifecycle ────────────────────────────────────────────────────────────
    def establish_zero(self, timeout, use_measured, measured, max_mismatch):
        """Decide which raw encoder reading counts as leg position 0.0.

        Runs BEFORE arm(), so the axes are still IDLE and nothing can move
        while we work it out. Encoder estimates come back in IDLE just fine —
        the RTR read does not need closed-loop control.
        """
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            for c in self.clients.values():
                c.request(CMD_GET_ENCODER_ESTIMATES)
            time.sleep(0.05)
            with self._lock:
                if all(self.fb_seen.values()):
                    break

        with self._lock:
            missing = [n for n, seen in self.fb_seen.items() if not seen]
            raw = {n: v[0] for n, v in self.fb.items()}
        if missing:
            # Hard failure, not a warning. Arming without feedback is exactly
            # the blind position step this whole sequence exists to prevent.
            raise RuntimeError(
                f'no encoder feedback from leg(s) {missing} within {timeout}s — '
                'refusing to arm. Check the CAN interface, bitrate and node IDs.')

        if not use_measured:
            # BRING-UP FALLBACK: whatever posture the legs are in RIGHT NOW
            # becomes 0.0. That is only true if they are physically against the
            # retracted stop — which holds on the ground (weight collapses them)
            # but NOT on a stand, where they hang extended. Push them onto the
            # stop before starting, or the soft limits protect the wrong range.
            self.node.get_logger().warn(
                'use_measured_zero is FALSE — taking boot posture as leg zero '
                f'(raw {raw["left"]:+.4f}/{raw["right"]:+.4f}). Valid ONLY if '
                'the legs are against the retracted hard stop. Measure '
                'zero_raw_left/right on hardware and set use_measured_zero.')
            return dict(raw)

        # Is the unwrap even trustworthy at this travel setting? The dead band
        # left over after travel is the ENTIRE error budget for backlash,
        # wind-up and encoder noise, and it is measured at the MOTOR, before the
        # gearbox. Squeeze pos_max toward 1/gear_ratio and it vanishes.
        window = 1.0 / self.gear
        margin = 0.5 * (window - self.pos_max - self.pos_min)
        if margin <= 0.0:
            raise RuntimeError(
                f'travel {self.pos_min}..{self.pos_max} needs {self.pos_max - self.pos_min:.4f} '
                f'turns but one motor turn is only {window:.4f} — boot position is '
                'AMBIGUOUS and cannot be unwrapped. Reduce pos_max or add homing.')
        if margin < self.min_unwrap_margin:
            self.node.get_logger().warn(
                f'unwrap margin is only {margin:.4f} output turns '
                f'({margin * self.gear:.3f} motor turns, '
                f'{margin * self.gear * 360:.0f} deg of MOTOR rotation) — below '
                f'min_unwrap_margin {self.min_unwrap_margin}. Backlash or wind-up '
                'past this flips the boot reading to the far end of the stroke. '
                'Lower pos_max to widen it.')

        # UNWRAP the boot reading. It came from a single-turn absolute encoder,
        # so it is only meaningful modulo ONE MOTOR TURN, and the window's
        # boundary sits at an arbitrary rotor angle set by the magnet — almost
        # certainly somewhere inside our travel. Plain subtraction would be
        # silently wrong on one side of that boundary. Travel is under one motor
        # turn, so exactly one candidate lands in range and we can recover it.
        #
        # AFTER boot, pos_estimate accumulates multi-turn continuously, so this
        # correction is needed exactly once: fold it into raw_zero and every
        # later read is plain arithmetic again.
        logical = {n: self.unwrap_boot(self.invert[n] * (raw[n] - measured[n]))
                   for n in raw}
        zero = {n: raw[n] - self.invert[n] * logical[n] for n in raw}

        mismatch = abs(logical['left'] - logical['right'])
        if mismatch > max_mismatch:
            raise RuntimeError(
                f'legs disagree by {mismatch:.4f} turns at boot '
                f'(left {logical["left"]:+.4f}, right {logical["right"]:+.4f}, '
                f'limit {max_mismatch}) — refusing to arm. Either a zero_raw_* '
                'constant is stale or the linkage has slipped.')
        window = 1.0 / self.gear
        margin = 0.5 * (window - self.pos_max - self.pos_min)
        split = self.pos_max + margin
        for n, v in logical.items():
            # 1e-6 so a leg resting exactly on a limit does not warn on float noise
            if not (self.pos_min - 1e-6 <= v <= self.pos_max + 1e-6):
                self.node.get_logger().warn(
                    f'{n} leg booted at {v:+.4f} turns, OUTSIDE the soft limits '
                    f'{self.pos_min}..{self.pos_max} — will ramp back into range.')
            # Distance to the unwrap SPLIT, measured around the window. This is
            # the real cliff: a reading that drifts across it resolves to the
            # opposite end of the stroke. Note a leg resting at the retracted
            # stop is only `margin` away from it, going negative — which is why
            # margin has to be big enough to swallow backlash and wind-up.
            to_split = min(v - split + window, split - v)
            if to_split < 0.25 * margin:
                self.node.get_logger().warn(
                    f'{n} leg booted at {v:+.4f}, only {to_split:.4f} turns from '
                    f'the unwrap split at {split:.4f}. This reading may resolve '
                    'to the WRONG end of the stroke — check /leg_states against '
                    'the physical leg before putting weight on it.')
        return zero

    def arm(self, vel_limit, current_limit, input_mode):
        for name, c in self.clients.items():
            c.set_limits(vel_limit, current_limit)
            c.set_controller_modes(ControlMode.POSITION, input_mode)
            c.set_axis_state(AxisState.CLOSED_LOOP_CONTROL)
            self.node.get_logger().info(
                f'armed {name} leg (node {c.node_id}) in POSITION mode')

    def shutdown(self):
        self.running = False
        if self.idle_on_shutdown:
            # WARNING: for a LEG, IDLE means the joint goes limp and the robot
            # sits down. Safe on a stand; think hard before doing this with the
            # robot's weight on the legs.
            self.node.get_logger().warn('idling leg motors — legs will go limp')
            for c in self.clients.values():
                try:
                    c.set_axis_state(AxisState.IDLE)
                except Exception:
                    pass

    # ── CAN ──────────────────────────────────────────────────────────────────
    def can_reader(self):
        while self.running:
            try:
                msg = self.bus.recv(timeout=0.1)
            except Exception:
                break
            if msg is None:
                continue
            for name, c in self.clients.items():
                decoded = c.decode(msg)
                if decoded and decoded[0] == 'encoder':
                    with self._lock:
                        self.fb[name] = list(decoded[1])
                        self.fb_seen[name] = True
                    break

    def command_callback(self, msg):
        requested = float(msg.data)
        self.target = self.clamp(requested)
        if abs(self.target - requested) > 1e-9:
            self.node.get_logger().warn(
                f'leg command {requested:.3f} clamped to {self.target:.3f} '
                f'(limits {self.pos_min}..{self.pos_max})')

    # ── control ──────────────────────────────────────────────────────────────
    def update(self):
        # ramp the setpoint instead of stepping it: a loaded leg given a step
        # sees a huge instantaneous position error and slams.
        step = self.max_speed / self.rate
        delta = self.target - self.setpoint
        if abs(delta) <= step:
            self.setpoint = self.target
        else:
            self.setpoint += step * (1.0 if delta > 0 else -1.0)

        for name, c in self.clients.items():
            # logical -> raw: undo the zero offset the same way we applied it
            c.set_input_pos(self.raw_zero[name] + self.invert[name] * self.setpoint)
            c.request(CMD_GET_ENCODER_ESTIMATES)

        pos = self.measured_position()
        with self._lock:
            vel = {n: v[1] for n, v in self.fb.items()}

        msg = JointState()
        msg.header.stamp = self.node.get_clock().now().to_msg()
        msg.name = ['left_leg_joint', 'right_leg_joint']
        msg.position = [pos['left'], pos['right']]
        # velocity is a rate, so the zero OFFSET drops out — only the sign stays
        msg.velocity = [self.invert['left'] * vel['left'],
                        self.invert['right'] * vel['right']]
        self.state_pub.publish(msg)

    @property
    def sag(self):
        """Commanded minus actual — how far the load is pulling the leg down."""
        pos = self.measured_position()
        return (self.setpoint - pos['left'], self.setpoint - pos['right'])


def main(args=None):
    rclpy.init(args=args)
    try:
        ctrl = LegController()
    except RuntimeError as exc:
        # establish_zero refused to arm. Report it as a one-line reason rather
        # than a traceback — this is an expected operational outcome, not a bug.
        rclpy.logging.get_logger('leg_controller').fatal(str(exc))
        rclpy.shutdown()
        return 1
    try:
        rclpy.spin(ctrl.node)
    except KeyboardInterrupt:
        pass
    finally:
        ctrl.shutdown()
        ctrl.bus.shutdown()
        rclpy.shutdown()
    return 0
