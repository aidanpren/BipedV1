import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

urdf = os.path.join(get_package_share_directory('robot_description'), 'urdf', 'biped.urdf')
with open(urdf) as f:
    robot_desc = f.read()


def generate_launch_description(): 
    return LaunchDescription([
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{'robot_description': robot_desc}],
        ),
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui'
        ),
        Node(
            package='rviz2',
            executable='rviz2'
        ),
    ])

    