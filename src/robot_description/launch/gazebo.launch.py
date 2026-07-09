import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory('robot_description')
    urdf = os.path.join(pkg, 'urdf', 'biped_sim.urdf')
    with open(urdf) as f:
        robot_desc = f.read()

    # 1. Start Gazebo Harmonic with an empty world (-r = run immediately)
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')),
        launch_arguments={'gz_args': '-r empty.sdf'}.items(),
    )

    # 2. robot_state_publisher: publishes the URDF on /robot_description + TF
    rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_desc}],
    )

    # 3. Spawn the robot into Gazebo from the /robot_description topic,
    #    dropped 0.35 m above the ground so it settles under gravity.
    spawn = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-topic', 'robot_description', '-name', 'biped', '-z', '0.35'],
        output='screen',
    )

    return LaunchDescription([gz_sim, rsp, spawn])
