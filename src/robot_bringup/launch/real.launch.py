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
        'can_channel', default_value='can0',
        description='SocketCAN interface for the ODrives (vcan0 to dry-run). '
                    'slcand names it can0 on the Pi. NOTE this default is '
                    'applied as an override BELOW the YAML, so it wins over '
                    'real.yaml — the two must agree or the file loses.')
    imu_driver = DeclareLaunchArgument(
        'imu_driver', default_value='uart',
        description='BNO085 interface: uart | spi | i2c, or fake to dry-run. '
                    'I2C is unreliable on a Pi (clock stretching).')
    # NOTE: mount_rpy is deliberately NOT a launch argument. It is a double
    # ARRAY, and a launch arg is always a STRING — passing '[0.0, 0.0, 0.0]'
    # here would fail the node's type check, and because dict overrides beat
    # the YAML it would clobber the calibrated value on every launch. It is
    # also a bench-calibration constant, not a per-run toggle. It lives in
    # config/real.yaml. Edit it there.

    # Every tunable lives here, not in the node source. sim.launch.py loads
    # config/sim.yaml instead — `diff sim.yaml real.yaml` is the record of how
    # the real robot differs from the simulated model.
    params = os.path.join(
        get_package_share_directory('robot_bringup'), 'config', 'real.yaml')

    # ---- HARDWARE LAYER (this is what sim.launch.py does with Gazebo) ------
    # NOTE the ORDER in each `parameters=` list: the YAML loads first, then the
    # dict OVERRIDES it. Later entries win. That is what lets a launch arg
    # (can_channel:=vcan0) beat the file without editing the file — the file
    # holds the real robot's values, the arg is the dry-run escape hatch.
    odrive = Node(
        package='robot_base', executable='odrive_bridge', output='screen',
        parameters=[params,
                    {'can_channel': LaunchConfiguration('can_channel')}],
    )
    imu = Node(
        package='robot_base', executable='imu_node', output='screen',
        parameters=[params,
                    {'driver': LaunchConfiguration('imu_driver')}],
    )
    # leg ODrives in POSITION mode. Shares the CAN bus with odrive_bridge —
    # SocketCAN gives each node its own socket on the same interface.
    legs = Node(
        package='robot_base', executable='leg_controller', output='screen',
        parameters=[params,
                    {'can_channel': LaunchConfiguration('can_channel')}],
    )

    # ---- SHARED: byte-for-byte the same nodes sim.launch.py runs -----------
    balance = Node(package='robot_base', executable='balance_controller',
                   output='screen', parameters=[params])
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
        can_channel, imu_driver,
        odrive, imu, legs,
        balance, mode_manager, teleop, dashboard,
    ])
