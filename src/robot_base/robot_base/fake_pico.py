import rclpy
from sensor_msgs.msg import Imu

class FakePico:
    def __init__(self):
        self.node = rclpy.create_node('fake_pico')
        self.publisher = self.node.create_publisher(Imu, 'imu/data', 10)
        self.timer = self.node.create_timer(0.1, self.publish_imu_data)

    def publish_imu_data(self):
        imu_msg = Imu()
        # Fill in the IMU message with fake data
        imu_msg.header.stamp = self.node.get_clock().now().to_msg()
        imu_msg.header.frame_id = 'base_link'
        imu_msg.orientation.x = 0.0
        imu_msg.orientation.y = 0.0
        imu_msg.orientation.z = 0.0
        imu_msg.orientation.w = 1.0
        imu_msg.angular_velocity.x = 0.0
        imu_msg.angular_velocity.y = 0.0
        imu_msg.angular_velocity.z = 0.0
        imu_msg.linear_acceleration.x = 0.0
        imu_msg.linear_acceleration.y = 0.0
        imu_msg.linear_acceleration.z = 9.81

        self.publisher.publish(imu_msg)

def main(args=None):
    rclpy.init(args=args)
    fake_pico = FakePico()
    rclpy.spin(fake_pico.node)
    rclpy.shutdown()