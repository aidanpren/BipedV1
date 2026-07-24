import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """The REAL robot. Mirror of sim.launch.py with the hardware swapped in.

    Compare the two: everything in the SHARED block below is identical to
    sim.launch.py. Only the bottom layer changes — Gazebo is replaced by
    odrive_bridge (wheels) + imu_node (pitch), which speak the same topics.
    balance_controller cannot tell the difference.

    Dry-run it on a laptop with NO hardware at all:
        sudo modprobe vcan
        sudo ip link add dev vcan0 type vcan && sudo ip link set up vcan0
        python3 tools/fake_odrive.py --channel vcan0 --node-id 1 &
        python3 tools/fake_odrive.py --channel vcan0 --node-id 2 &
        ros2 launch robot_bringup real.launch.py can_channel:=vcan0 imu_driver:=fake

    NOTE the arg names are deliberately specific (can_channel, imu_driver).
    A generic name here would leak into the included launch files and silently
    override THEIR defaults.
    """
    can_channel = DeclareLaunchArgument(
        'can_channel', default_value='can2',
        description='SocketCAN interface for the ODrives (vcan0 to dry-run).')
    imu_driver = DeclareLaunchArgument(
        'imu_driver', default_value='uart',
        description='BNO085 interface: uart | spi | i2c, or fake to dry-run. '
                    'I2C is unreliable on a Pi (clock stretching).')
    mount_rpy = DeclareLaunchArgument(
        'imu_mount_rpy', default_value='[0.0, 0.0, 0.0]',
        description='Rotation taking SENSOR axes to ROBOT axes. VERIFY ON A '
                    'BENCH before enabling torque.')

    # ---- HARDWARE LAYER (this is what sim.launch.py does with Gazebo) ------
    odrive = Node(
        package='robot_base', executable='odrive_bridge', output='screen',
        parameters=[{'can_channel': LaunchConfiguration('can_channel')}],
    )
    imu = Node(
        package='robot_base', executable='imu_node', output='screen',
        parameters=[{'driver': LaunchConfiguration('imu_driver'),
                     'mount_rpy': LaunchConfiguration('imu_mount_rpy')}],
    )
    # leg ODrives in POSITION mode. Shares the CAN bus with odrive_bridge —
    # SocketCAN gives each node its own socket on the same interface.
    legs = Node(
        package='robot_base', executable='leg_controller', output='screen',
        parameters=[{'can_channel': LaunchConfiguration('can_channel')}],
    )

    # ---- SHARED: byte-for-byte the same nodes sim.launch.py runs -----------
    balance = Node(package='robot_base', executable='balance_controller',
                   output='screen')
    mode_manager = Node(package='robot_teleop', executable='mode_manager',
                        output='screen')
    teleop = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('robot_teleop'),
            'launch', 'teleop.launch.py')))
    dashboard = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('robot_dashboard'),
            'launch', 'dashboard.launch.py')))

    return LaunchDescription([
        can_channel, imu_driver, mount_rpy,
        odrive, imu, legs,
        balance, mode_manager, teleop, dashboard,
    ])
