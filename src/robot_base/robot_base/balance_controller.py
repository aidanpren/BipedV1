import math

import rclpy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu, JointState
from std_msgs.msg import Float64MultiArray

class BalanceController:
    def __init__(self):
        self.node = rclpy.create_node('balance_controller')

        # live-tunable: ros2 param set /balance_controller <name> <value>
        self.node.declare_parameter('k3', 20.0)
        self.node.declare_parameter('k4', 2.0)
        self.node.declare_parameter('a1', 0.0)     # outer loop starts OFF: bisect from pure PD
        self.node.declare_parameter('a2', 0.0)
        self.node.declare_parameter('k_yaw', 0.0)
        self.node.declare_parameter('max_lean', 0.15)
        self.node.declare_parameter('max_torque', 10.0)
        self.cutoff_pitch = 0.7
        self.wheel_radius = 0.105

        self.x_ready = False
        self.x = 0.0
        self.v = 0.0
        self.x_home = 0.0
        self.v_ref = 0.0
        self.yaw_ref = 0.0
        self.last_cmd_time = self.node.get_clock().now()

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

    def imu_callback(self, msg):
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

        # watchdog: stale cmd_vel → zero refs, keep balancing
        age = (self.node.get_clock().now() - self.last_cmd_time).nanoseconds * 1e-9
        v_ref = self.v_ref if age < 0.5 else 0.0
        yaw_ref = self.yaw_ref if age < 0.5 else 0.0

        if abs(pitch) > self.cutoff_pitch:
            left = right = 0.0
            self.x_home = self.x    # home follows the robot while it's down
        else:
            # mode: driving vs holding station
            if abs(v_ref) > 0.05:
                self.x_home = self.x                          # home follows you while driving
                pitch_target = a2 * (self.v - v_ref)
            else:
                pitch_target = a1 * (self.x - self.x_home) + a2 * self.v
            pitch_target = max(-max_lean, min(max_lean, pitch_target))

            # balance (common) + steering (differential)
            torque = -(k3 * (pitch - pitch_target) + k4 * pitch_rate)
            t_yaw = k_yaw * (yaw_ref - msg.angular_velocity.z)
            left = torque - t_yaw
            right = torque + t_yaw
            left = max(-max_torque, min(max_torque, left))
            right = max(-max_torque, min(max_torque, right))

        self.publisher.publish(Float64MultiArray(data=[left, right]))

        self.node.get_logger().info(
            f'pitch {pitch:+.3f}  x {self.x:+.2f}  v {self.v:+.2f}  L {left:+.1f} R {right:+.1f}',
            throttle_duration_sec=1.0)

        
    def joint_state_callback(self, msg):
        try:
            l = msg.name.index('left_wheel_joint')
            r = msg.name.index('right_wheel_joint')
        except ValueError:
            return
        self.x = self.wheel_radius * (msg.position[l] + msg.position[r]) / 2.0
        if not self.x_ready:
            self.x_home = self.x
            self.x_ready = True
        self.v = self.wheel_radius * (msg.velocity[l] + msg.velocity[r]) / 2.0

    def cmd_vel_callback(self, msg):
        self.v_ref = msg.linear.x
        self.yaw_ref = msg.angular.z
        self.last_cmd_time = self.node.get_clock().now()


def main(args=None):
    rclpy.init(args=args)
    balance_controller = BalanceController()
    rclpy.spin(balance_controller.node)
    rclpy.shutdown()
