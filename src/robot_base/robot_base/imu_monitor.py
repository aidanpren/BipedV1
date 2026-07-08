import rclpy
from sensor_msgs.msg import Imu


class ImuMonitor:
    def __init__(self):
        self.node = rclpy.create_node('imu_monitor')
        self.subscription = self.node.create_subscription(
            Imu,
            'imu/data',
            self.imu_callback,
            10
        )
        self.subscription  # prevent unused variable warning

    def imu_callback(self, msg):
        # Process the incoming IMU data here
        self.node.get_logger().info(f"Received IMU data: {msg.linear_acceleration.z}")

def main(args=None):
    rclpy.init(args=args)
    imu_monitor = ImuMonitor()
    rclpy.spin(imu_monitor.node)
    rclpy.shutdown()