"""SIM-ONLY parity bridge for the legs.

The real robot runs `leg_controller` (robot_base), which presents this interface
to the rest of the system:

    subscribes  leg_position_cmd   Float64      one target -> BOTH legs
    publishes   leg_states         JointState   ['left_leg_joint','right_leg_joint']

In sim there is no leg_controller. Instead gz_ros2_control runs a
`leg_position_controller`, which speaks a DIFFERENT dialect:

    subscribes  /leg_position_controller/commands   Float64MultiArray [left, right]
    (state)     /joint_states                       carries left_hip_joint, right_hip_joint

This node is the translator between the two, so the dashboard slider and the
real leg_controller are interchangeable — balance_controller, the dashboard,
and any test drive sim and hardware through the EXACT same two topics without
knowing which is underneath. Same idea as gz_ros2_control vs odrive_bridge both
serving balance_controller.

UNITS: the dashboard slider is in motor OUTPUT TURNS; the sim prismatic joint is
in METRES of extension. They are related by the REAL four-bar kinematics
(leg-forward-kinematics memory) — a nonlinear SINE map, NOT a scale factor:

    command   (forward): turns  --f-->    metres of extension   [to the sim]
    state pos (inverse): metres --f^-1--> turns                 [to the dashboard]
    state vel:           m/s    --/f'-->  turns/s   (rates use the DERIVATIVE)

The state path MUST convert back, because the dashboard computes leg sag as
(slider_target_in_turns - reported_position); reporting metres would make that
subtraction meaningless. Note position and velocity convert DIFFERENTLY:
position through the inverse map, velocity through the map's local slope f'.

Ramping is NOT done here. The real leg_controller ramps its setpoint in
software; the sim ramps in PHYSICS (the joint velocity limit + the position
controller's proportional gain), so a straight passthrough already moves the
sim leg at a finite, realistic rate.

Run (sim only — needs the Gazebo stack up so the controller topic exists):
    ros2 run robot_description sim_leg_bridge
"""
import math

import rclpy
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64, Float64MultiArray

# Structural, not tunable (like the odrive_can command IDs) — these are fixed by
# the URDF joint names and the real leg_controller's published names. Order is
# [left, right] and the two lists are positionally paired.
SIM_JOINTS = ['left_hip_joint', 'right_hip_joint']    # what /joint_states calls them
OUT_JOINTS = ['left_leg_joint', 'right_leg_joint']    # what the dashboard expects

# --- Real leg forward kinematics (leg-forward-kinematics memory) -------------
# The four-bar leg's wheel extension below the retracted hard stop, as a
# function of motor OUTPUT turns (0 turns = retracted):
#     ext(turns) = AMPLITUDE_M * (sin(RETRACTED_RAD + 2*pi*turns) - sin(RETRACTED_RAD))
# Physical constants of the mechanism, NOT tunables:
AMPLITUDE_M = 2 * 9.1 * 0.0254             # 2 x 9.1" link length, in metres (~0.4623)
RETRACTED_RAD = math.radians(20.2841444)   # crank angle at the retracted hard stop
TWO_PI = 2.0 * math.pi
SIN_RETRACTED = math.sin(RETRACTED_RAD)
# turns at full extension (crank 90deg) = the extended hard stop, ~0.1937.
# Beyond it the sine folds over (arcsin wrong-branches, cos flips sign).
TURNS_MAX = (0.5 * math.pi - RETRACTED_RAD) / TWO_PI


def extension_metres(turns):
    ext_meters = AMPLITUDE_M * (math.sin(RETRACTED_RAD + 2 * math.pi * turns) - SIN_RETRACTED)
    return ext_meters


def turns_from_extension(ext_m):
    """Inverse of extension_metres: extension [m] -> motor turns.

    Solve ext = AMPLITUDE_M * (sin(RETRACTED_RAD + 2pi*turns) - SIN_RETRACTED):
        sin(RETRACTED_RAD + 2pi*turns) = ext/AMPLITUDE_M + SIN_RETRACTED
    GOTCHA: a sim value a hair past the 0.302 m stop pushes that argument past
    1.0 and math.asin() raises ValueError — clamp into [-1, 1] first.
    """
    arg = ext_m / AMPLITUDE_M + SIN_RETRACTED
    arg = max(-1.0, min(1.0, arg))
    return (math.asin(arg) - RETRACTED_RAD) / TWO_PI


def turns_rate_from_extension_rate(ext_rate, turns):
    """Velocity [m/s] -> turns/s at the current pose.

    A rate transforms through the DERIVATIVE of the map, not the map itself:
        d(ext)/dt = f'(turns) * d(turns)/dt
        f'(turns) = AMPLITUDE_M * 2pi * cos(RETRACTED_RAD + 2pi*turns)
    GOTCHA: at full extension f'(turns) -> 0 (the linkage toggle), so guard the
    divide or turns/s blows up.
    """
    # clamp to the valid domain so cos() can't cross the 90deg toggle and flip
    # sign (turns_from_extension already returns values within [0, TURNS_MAX]).
    turns = min(max(turns, 0.0), TURNS_MAX)
    slope = AMPLITUDE_M * TWO_PI * math.cos(RETRACTED_RAD + TWO_PI * turns)
    if abs(slope) < 1e-6:
        return 0.0
    return ext_rate / slope


class SimLegBridge:
    def __init__(self):
        self.node = rclpy.create_node('sim_leg_bridge')

        # OUT command -> the gz position controller (Float64MultiArray [L, R]).
        self.cmd_pub = self.node.create_publisher(
            Float64MultiArray, '/leg_position_controller/commands', 10)
        # IN command <- the dashboard / any teleop (single Float64, in turns).
        self.cmd_sub = self.node.create_subscription(
            Float64, 'leg_position_cmd', self.command_callback, 10)

        # OUT state -> the dashboard, under the real leg_controller's names.
        self.state_pub = self.node.create_publisher(JointState, 'leg_states', 10)
        # IN state <- Gazebo's joint_state_broadcaster (all 4 joints, in metres).
        self.state_sub = self.node.create_subscription(
            JointState, 'joint_states', self.joint_state_callback, 10)

        self.node.get_logger().info(
            'sim_leg_bridge up: real four-bar kinematics '
            f'(0..{turns_from_extension(0.302):.4f} turns <-> 0..0.302 m)')

    def command_callback(self, msg):
        turns = msg.data
        metres = extension_metres(turns)
        cmd = Float64MultiArray()
        cmd.data = [metres, metres]
        self.cmd_pub.publish(cmd)

    def joint_state_callback(self, msg):
        try:
            left_hip = msg.name.index(SIM_JOINTS[0])
            right_hip = msg.name.index(SIM_JOINTS[1])
            left_position_m = msg.position[left_hip]
            right_position_m = msg.position[right_hip]
            left_velocity_m = msg.velocity[left_hip]
            right_velocity_m = msg.velocity[right_hip]
        except (ValueError, IndexError):
            return

        # position through the inverse map, velocity through the local slope
        left_position_turns = turns_from_extension(left_position_m)
        right_position_turns = turns_from_extension(right_position_m)
        left_velocity_turns = turns_rate_from_extension_rate(
            left_velocity_m, left_position_turns)
        right_velocity_turns = turns_rate_from_extension_rate(
            right_velocity_m, right_position_turns)

        joint_state_msg = JointState()
        joint_state_msg.header.stamp = self.node.get_clock().now().to_msg()
        joint_state_msg.name = OUT_JOINTS
        joint_state_msg.position = [left_position_turns, right_position_turns]
        joint_state_msg.velocity = [left_velocity_turns, right_velocity_turns]
        self.state_pub.publish(joint_state_msg)


def main(args=None):
    rclpy.init(args=args)
    bridge = SimLegBridge()
    try:
        rclpy.spin(bridge.node)
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
