import math

import rclpy
from rclpy.qos import QoSProfile, DurabilityPolicy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu, JointState
from std_msgs.msg import Float64MultiArray, String

class BalanceController:
    def __init__(self):
        self.node = rclpy.create_node('balance_controller')

        self.default_mode = 'disabled'
        self.mode = self.default_mode

        # live-tunable: ros2 param set /balance_controller <name> <value>
        self.node.declare_parameter('k3', 20.0)
        self.node.declare_parameter('k4', 2.0)
        # outer-loop signs verified 2026-07-13: positive a1/a2 = saturated-lean
        # runaway; negative = station hold (with the plus-form law below)
        #
        # a1 and a2 are NOT independent. The outer loop is second order:
        #     omega_n = sqrt(g*|a1|)      zeta = (|a2|/2) * sqrt(g/|a1|)
        # so lowering a2 (which the driving loop requires, see imu_callback)
        # is paid back by lowering a1 too, NOT by raising it. -0.09/-0.3 gave
        # zeta 1.57 -- overdamped, slow pole tau 3 s. -0.07/-0.15 gives
        # zeta 0.89, tau 1.4 s: half the driving bandwidth AND a station hold
        # that recovers twice as fast.
        self.node.declare_parameter('a1', -0.07)
        self.node.declare_parameter('a2', -0.15)
        self.node.declare_parameter('k_yaw', 4.0)
        self.node.declare_parameter('max_lean', 0.3)
        self.node.declare_parameter('max_torque', 40.0)
        # chassis pitch at which the CoM is over the contact patch, rad.
        # POSITIVE = nose-down, which is what a BACK-heavy robot needs.
        # Measure it: power off, balance the robot by hand on its wheels, read
        # pitch from `imu_calibrate.py --watch`. See BALANCE_RUNBOOK.md.
        self.node.declare_parameter('pitch_trim', 0.0)

        # REFERENCE SHAPING (2026-08-01). See the block comment in imu_callback.
        # accel_limit ramps v_cmd; accel_to_lean feeds that ramp forward as a
        # lean so feedback never has to generate it; max_pos_error is the
        # anti-windup clamp on how far x_home may run ahead of the robot.
        self.node.declare_parameter('accel_limit', 0.3)      # m/s^2
        self.node.declare_parameter('jerk_tau', 0.15)        # s
        self.node.declare_parameter('accel_to_lean', 0.0)    # rad per m/s^2
        self.node.declare_parameter('max_pos_error', 0.25)   # m

        # YAW SHAPING (2026-08-02). yaw_ref used to reach the law as a raw 20 Hz
        # staircase while linear.x got two stages of shaping, so a stick flick
        # put k_yaw*0.75 = 3.0 Nm of DIFFERENTIAL torque on the wheels in one
        # tick, out of a max_torque of 8.0 shared with balancing.
        #
        # ONE stage here, not two. Stage 2 on the linear path exists because
        # accel_to_lean feeds a_ref into pitch_target, so a square-wave a would
        # become a square-wave lean target. Yaw has no such feedforward — t_yaw
        # goes straight to differential torque — so bounding yaw ACCELERATION
        # is enough, and a second lag would only add steering delay.
        #
        # 2.0 rad/s^2 puts the torque slew at k_yaw*2.0*dt = 0.08 Nm/tick at
        # 100 Hz, which is the same slew the linear path achieves WITH both its
        # stages (0.074 Nm/tick, measured offline). Full-stick ramp: 0.375 s.
        self.node.declare_parameter('yaw_accel_limit', 2.0)  # rad/s^2

        # promoted from hardcoded constants 2026-08-01
        self.node.declare_parameter('v_filter_tau', 0.06)    # s
        self.node.declare_parameter('wheel_radius', 0.105)   # m
        self.node.declare_parameter('cutoff_pitch', 0.7)     # rad

        self.x_ready = False
        self.x = 0.0
        self.v = 0.0
        self.v_f = 0.0              # low-passed v: slip spikes can't slam the lean target
        self.upright_since = None   # outer loop engages 1 s after recovery, not mid-tumble
        self.x_home = 0.0
        self.v_ref = 0.0            # raw request from cmd_vel
        self.v_ramp = 0.0           # after the accel limit (stage 1)
        self.v_cmd = 0.0            # after the jerk limit (stage 2) — the law uses this
        self.yaw_ref = 0.0          # raw request from cmd_vel
        self.yaw_cmd = 0.0          # after the yaw accel limit — the law uses this
        self.last_cmd_time = self.node.get_clock().now()
        self.last_step = None       # measured dt for imu_callback
        self.last_js = None         # measured dt for joint_state_callback

        self.publisher = self.node.create_publisher(Float64MultiArray, 'wheel_effort_controller/commands', 10)
        self.imu_sub = self.node.create_subscription(
            Imu,
            'imu',
            self.imu_callback,
            10
        )
        self.joint_state_sub = self.node.create_subscription(
            JointState, 'joint_states', self.joint_state_callback, 10)
        self.cmd_vel_sub = self.node.create_subscription(
            Twist, 'cmd_vel', self.cmd_vel_callback, 10)
        latched_qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.mode_sub = self.node.create_subscription(
            String, 'mode', self.mode_callback, latched_qos)

    def imu_callback(self, msg):
        now = self.node.get_clock().now()

        # MEASURED dt, never assumed. The BNO085 is on I2C and clock stretching
        # drops samples, so the gap between callbacks is not reliably
        # 1/publish_rate. Clamped at 50 ms: one long stall must not integrate
        # x_home a third of a metre ahead of the robot in a single step.
        if self.last_step is None:
            dt = 0.0
        else:
            dt = min((now - self.last_step).nanoseconds * 1e-9, 0.05)
        self.last_step = now

        if self.mode == 'disabled':
            self.publisher.publish(Float64MultiArray(data=[0.0, 0.0]))
            self.x_home = self.x
            self.v_cmd = self.v_ramp = 0.0
            self.yaw_cmd = 0.0
            self.upright_since = None
            return

        q = msg.orientation
        sinp = 2.0 * (q.w * q.y - q.z * q.x)
        sinp = max(-1.0, min(1.0, sinp))   # clamp — floating point can exceed ±1
        pitch = math.asin(sinp)            # radians

        pitch_rate = msg.angular_velocity.y

        # fetch live parameter values (so ros2 param set takes effect instantly)
        k3 = self.node.get_parameter('k3').value
        k4 = self.node.get_parameter('k4').value
        a1 = self.node.get_parameter('a1').value
        a2 = self.node.get_parameter('a2').value
        k_yaw = self.node.get_parameter('k_yaw').value
        max_lean = self.node.get_parameter('max_lean').value
        max_torque = self.node.get_parameter('max_torque').value
        accel_limit = self.node.get_parameter('accel_limit').value
        jerk_tau = self.node.get_parameter('jerk_tau').value
        accel_to_lean = self.node.get_parameter('accel_to_lean').value
        yaw_accel_limit = self.node.get_parameter('yaw_accel_limit').value
        max_pos_error = self.node.get_parameter('max_pos_error').value
        cutoff_pitch = self.node.get_parameter('cutoff_pitch').value

        # watchdog: stale cmd_vel → zero the TARGET, keep balancing.
        #
        # It zeros the target and not v_cmd itself, so the ramp below coasts the
        # robot down at accel_limit instead of stepping to zero. That is also
        # WHY the rate limiter lives here rather than in a smoother node between
        # twist_mux and this node: a smoother would go on publishing fresh
        # messages after its own input died, this watchdog would never fire, and
        # staleness detection would have silently migrated out of the node
        # CLAUDE.md requires to own it.
        age = (now - self.last_cmd_time).nanoseconds * 1e-9
        v_target = self.v_ref if age < 0.5 else 0.0
        yaw_target = self.yaw_ref if age < 0.5 else 0.0

        # REFERENCE SHAPING, in two stages.
        #
        # Stage 1 bounds ACCELERATION: v_ref arrives as a 20 Hz staircase from
        # the joystick, and a step through a proportional term is a torque step.
        #
        # Stage 2 bounds JERK, and it is not optional. Rate-limiting velocity
        # turns acceleration into a SQUARE WAVE, and accel_to_lean feeds
        # acceleration straight into the lean target — so stage 1 alone merely
        # moves the discontinuity from v to a and the jerk survives intact.
        # Offline sim of this exact code: peak torque slew was 0.663 Nm/tick
        # with stage 1 only, 0.074 Nm/tick with both. Lagging v_cmd behind
        # v_ramp makes a_ref continuous — it rises to accel_limit over jerk_tau
        # instead of arriving as an edge.
        step = accel_limit * dt
        self.v_ramp += max(-step, min(step, v_target - self.v_ramp))
        beta = dt / (jerk_tau + dt) if (jerk_tau + dt) > 0.0 else 1.0
        dv = beta * (self.v_ramp - self.v_cmd)
        self.v_cmd += dv
        a_ref = dv / dt if dt > 0.0 else 0.0

        # Same treatment for yaw, and note it ramps toward yaw_TARGET — so a
        # stale cmd_vel decelerates the turn at yaw_accel_limit rather than
        # stepping the differential torque to zero, exactly as the linear
        # watchdog coasts v down instead of dropping it.
        yaw_step = yaw_accel_limit * dt
        self.yaw_cmd += max(-yaw_step, min(yaw_step, yaw_target - self.yaw_cmd))

        if abs(pitch) > cutoff_pitch:
            left = right = 0.0
            self.x_home = self.x    # home follows the robot while it's down
            self.v_cmd = self.v_ramp = 0.0
            self.yaw_cmd = 0.0
            self.upright_since = None
        else:
            if self.upright_since is None:      # just recovered: re-anchor, start settle timer
                self.upright_since = now
                self.x_home = self.x
            settled = (now - self.upright_since).nanoseconds * 1e-9 > 1.0

            if settled:
                # ── ONE law. No mode branch, no threshold. ──────────────────
                # This replaces the old driving/station-keeping split
                # (2026-08-01). That split had three faults:
                #
                # 1. v_ref appeared ONLY in the branch test, never in the
                #    station-keeping law. With scale_linear 0.15 and a
                #    threshold of 0.05, the bottom 33% of stick travel was
                #    silently discarded and then the law changed underneath
                #    you in one cycle. That was the "twitchy" feel.
                # 2. a2 did two contradictory jobs: DAMPING (D term) for
                #    station keeping, where big is good, and velocity-loop
                #    P GAIN for driving, where big is unstable. No single
                #    value could satisfy both.
                # 3. Driving threw position feedback away entirely
                #    (x_home = x every cycle), so releasing the stick snapped
                #    it back on with a step in pitch_target.
                #
                # Ramping the reference POSITION at the reference SPEED fixes
                # all three. At v_cmd = 0 this collapses exactly to the old
                # station-keeping law, so nothing that already worked is lost.
                # At steady v_cmd, x_home runs ahead, a1 leans harder until the
                # robot catches up -- a1 becomes integral action on speed and
                # a2 is pure damping in BOTH regimes.
                self.x_home += self.v_cmd * dt

                # ANTI-WINDUP. If the robot cannot reach v_cmd (blocked, held,
                # torque-saturated) x_home runs away, and every centimetre of
                # it is distance the robot will sprint to recover when the
                # obstruction clears. Cap how far the reference may lead.
                self.x_home = max(self.x - max_pos_error,
                                  min(self.x + max_pos_error, self.x_home))

                # FEEDFORWARD is what makes the ramp usable. A balancer is
                # non-minimum-phase: to speed up it must first drive its wheels
                # BACKWARD to tip forward, so measured velocity moves the wrong
                # way before the right way, and feedback that reacts to that
                # dip drives the surge. Worse, generating the cruise lean from
                # a1 alone needs a standing error of a_ref/(g*|a1|) = 0.44 m at
                # 0.3 m/s^2 -- a debt repaid as a lurch when the stick centres.
                # We already KNOW the commanded acceleration, so lean for it
                # directly and leave feedback only the mismatch to clean up.
                # accel_to_lean is 1/g = 0.102 rad per m/s^2, a checkable
                # physical constant rather than a tuned gain. Set it to 0.0 to
                # disable the feedforward without touching the rest.
                pitch_target = (a1 * (self.x - self.x_home)
                                + a2 * (self.v_f - self.v_cmd)
                                + accel_to_lean * a_ref)
            else:
                # still settling after a recovery: hold level, keep the
                # reference pinned to the robot so it does not engage with a
                # position error already banked.
                pitch_target = 0.0
                self.x_home = self.x
                self.v_cmd = self.v_ramp = 0.0
                self.yaw_cmd = 0.0

            # TRIM: the chassis angle at which the CoM sits over the contact
            # patch. It is NOT zero unless the robot is perfectly balanced fore
            # and aft — a back-heavy robot has to stand slightly nose-down to
            # put its mass over the wheels.
            #
            # Without this the inner loop holds pitch = 0, gravity applies a
            # constant torque about the contact point, and the only way to keep
            # that angle is to accelerate backwards forever. It looks exactly
            # like a tuning problem and no k3/k4 can fix it.
            #
            # The outer loop CAN absorb it, but only by carrying a permanent
            # position error of pitch_trim/a1 — it would sit parked off home,
            # leaning, with a1 fighting gravity full time. Stating the trim
            # explicitly frees the outer loop to handle actual disturbances.
            #
            # CLAMP FIRST, then trim. max_lean bounds how far the OUTER LOOP
            # may lean the robot away from its equilibrium — and equilibrium is
            # pitch_trim, not zero. Clamping the sum instead would make the
            # authority asymmetric: at trim 0.05 and max_lean 0.3 the outer loop
            # could lean 0.25 one way and 0.35 the other, for no stated reason.
            pitch_target = max(-max_lean, min(max_lean, pitch_target))
            pitch_target += self.node.get_parameter('pitch_trim').value

            # balance (common) + steering (differential)
            # SIGN verified empirically in headless Gazebo (2026-07-13): the
            # minus-law -(k3*(...)+k4*rate) slams the robot down in <1 s; this
            # plus-form balances and rejects 0.2 rad disturbances.
            torque = k3 * (pitch - pitch_target) + k4 * pitch_rate
            t_yaw = k_yaw * (self.yaw_cmd - msg.angular_velocity.z)

            # BALANCE OUTRANKS STEERING when the budget runs out.
            #
            # Clamping left and right INDEPENDENTLY looks equivalent and is not.
            # At torque 7.0 with t_yaw 3.0 you get left 4.0, right 10.0 -> 8.0,
            # and the common mode that actually holds the robot up has silently
            # become (8.0+4.0)/2 = 6.0. Asymmetric saturation steals 1 Nm from
            # BALANCING to pay for a turn the driver asked for — the failure is
            # invisible in the logs and shows up as the robot dropping a little
            # every time you yaw hard while also catching yourself.
            #
            # So spend the budget in priority order: balance takes what it needs,
            # yaw gets the headroom that is left. This also removes the need for
            # a second clamp — |t_yaw| <= max_torque - |torque| guarantees both
            # |left| and |right| <= max_torque by the triangle inequality.
            torque = max(-max_torque, min(max_torque, torque))
            headroom = max_torque - abs(torque)
            t_yaw = max(-headroom, min(headroom, t_yaw))
            left = torque - t_yaw
            right = torque + t_yaw

        self.publisher.publish(Float64MultiArray(data=[left, right]))

        self.node.get_logger().info(
            f'pitch {pitch:+.3f}  x {self.x:+.2f}  err {self.x - self.x_home:+.2f}  '
            f'v {self.v_f:+.2f}->{self.v_cmd:+.2f}  w {self.yaw_cmd:+.2f}  '
            f'L {left:+.1f} R {right:+.1f}',
            throttle_duration_sec=1.0)

        
    def joint_state_callback(self, msg):
        try:
            l = msg.name.index('left_wheel_joint')
            r = msg.name.index('right_wheel_joint')
        except ValueError:
            return
        now = self.node.get_clock().now()
        wheel_radius = self.node.get_parameter('wheel_radius').value

        self.x = wheel_radius * (msg.position[l] + msg.position[r]) / 2.0
        if not self.x_ready:
            self.x_home = self.x
            self.x_ready = True
        self.v = wheel_radius * (msg.velocity[l] + msg.velocity[r]) / 2.0

        # LOW-PASS expressed as a TIME CONSTANT, not a fixed coefficient.
        # A hardcoded 0.85/0.15 is only a known lag at a known rate, and the
        # rate assumption was wrong: /joint_states comes from odrive_bridge at
        # 50 Hz, so -0.02/ln(0.85) = 123 ms, not the 65 ms the old comment
        # claimed. 123 ms of phase lag inside the velocity loop was a large
        # part of why driving surged. Deriving alpha from the MEASURED interval
        # makes the lag independent of publish_rate, so changing that rate can
        # never again silently retune this filter.
        if self.last_js is None:
            self.v_f = self.v
        else:
            dt = min((now - self.last_js).nanoseconds * 1e-9, 0.1)
            tau = self.node.get_parameter('v_filter_tau').value
            alpha = dt / (tau + dt) if (tau + dt) > 0.0 else 1.0
            self.v_f += alpha * (self.v - self.v_f)
        self.last_js = now

    def cmd_vel_callback(self, msg):
        self.v_ref = msg.linear.x
        self.yaw_ref = msg.angular.z
        self.last_cmd_time = self.node.get_clock().now()

    def mode_callback(self, msg):
        self.mode = msg.data
        self.node.get_logger().info(f'mode {self.mode}')


def main(args=None):
    rclpy.init(args=args)
    balance_controller = BalanceController()
    rclpy.spin(balance_controller.node)
    rclpy.shutdown()
