import os

from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


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
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory('robot_teleop'), 'launch', 'teleop.launch.py')
            )
        )
    ])