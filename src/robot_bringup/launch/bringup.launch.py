from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='robot_base',
            executable='fake_pico',
            name='fake_pico_node',
            output='screen'
        ),
        Node(
            package='robot_base',
            executable='imu_monitor',
            name='imu_monitor_node',
            output='screen'
        )
    ])