import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
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
    # THIS DEFAULT MUST MATCH HOW THE SENSOR IS PHYSICALLY WIRED, because the
    # override dict below WINS over real.yaml. It used to say 'uart' — the
    # aspiration — while the board is wired I2C, so every bare launch (which
    # is every systemd boot: BIPED_LAUNCH_ARGS is empty) pointed imu_node at
    # a serial port nothing is connected to. The port opens, nothing answers,
    # and the first standalone boot (2026-08-02) came up with every service
    # healthy and the robot deaf. Debugging burned an hour on clock-stretch
    # and systemd theories; `ros2 param get /imu_node driver` would have
    # answered in one line. Change this to 'uart' only WHEN THE REWIRE
    # ACTUALLY HAPPENS, in the same commit as real.yaml's driver field.
    imu_driver = DeclareLaunchArgument(
        'imu_driver', default_value='i2c',
        description='BNO085 interface: uart | spi | i2c, or fake to dry-run. '
                    'MUST match the physical wiring — this beats the YAML.')
    # DEFAULT FALSE, deliberately (2026-08-02). The wheeled-biped goal —
    # turn on, pad on, press a button, drive — does not involve the legs at
    # all: balance_controller contains no leg references and the hips are a
    # separate ODrive pair on the same bus.
    #
    # This is a SAFETY default, not just a tidiness one. leg_controller arms in
    # POSITION mode and ramps to home_position the moment it starts, without
    # consulting the mode, so launching it means powered leg motion at every
    # boot. Not launching the node is a stronger guarantee than any check
    # inside it: there is no process to arm anything.
    #
    # Turning this on is a THREE-part prerequisite, not a flag flip:
    #   1. leg_controller gates arm() on mode (HANDOFF backlog 7), and
    #   2. leg bring-up has passed on a stand — zero measured, travel limits
    #      confirmed, the power-cycle test in HARDWARE_BRINGUP.md done, and
    #   3. you actually want the legs live in the balance stack.
    # For (2) do NOT use this flag: run the legs ALONE. See HARDWARE_BRINGUP.md.
    legs = DeclareLaunchArgument(
        'legs', default_value='false',
        description='Run leg_controller and the leg stick. FALSE until leg '
                    'bring-up is done — it moves the legs at startup.')
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
    # READ-ONLY electrical telemetry: per-motor amps, bus volts, axis faults.
    # A third socket on the same CAN interface, which SocketCAN is fine with —
    # the kernel copies received frames to every open socket, so this neither
    # steals frames from odrive_bridge nor needs to coordinate with it.
    #
    # Outside the balance loop on purpose: if it dies the robot balances
    # exactly as before. It also sends nothing but RTR read requests, so there
    # is no code path in it that can energise a motor. See the node's header.
    telemetry = Node(
        package='robot_base', executable='odrive_telemetry', output='screen',
        parameters=[params,
                    {'can_channel': LaunchConfiguration('can_channel')}],
    )
    # leg ODrives in POSITION mode. Shares the CAN bus with odrive_bridge —
    # SocketCAN gives each node its own socket on the same interface.
    # OFF by default; see the `legs` argument above for why.
    leg_controller = Node(
        package='robot_base', executable='leg_controller', output='screen',
        parameters=[params,
                    {'can_channel': LaunchConfiguration('can_channel')}],
        condition=IfCondition(LaunchConfiguration('legs')),
    )

    # ---- SHARED: byte-for-byte the same nodes sim.launch.py runs -----------
    balance = Node(package='robot_base', executable='balance_controller',
                   output='screen', parameters=[params])
    # takes params now (the DS4 mode buttons). Loaded from real.yaml via
    # --params-file, i.e. from SOURCE — so finding the right button index is
    # edit-and-restart, with no colcon build in the loop.
    mode_manager = Node(package='robot_teleop', executable='mode_manager',
                        output='screen', parameters=[params])
    # `legs` is passed EXPLICITLY rather than relied on to leak. A parent's
    # launch configurations do propagate into an include and do override the
    # child's own default — that is exactly the trap that once made rosbridge
    # bind the dashboard's port — so depending on it silently would be the same
    # bug wearing a different hat. Passing it means the coupling is visible
    # here, in the file you would read.
    teleop = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('robot_teleop'),
            'launch', 'teleop.launch.py')),
        launch_arguments={'legs': LaunchConfiguration('legs')}.items())
    # `dashboard_params` is passed EXPLICITLY for the same reason `legs` is
    # below: a parent's launch configurations DO leak into an include, so
    # relying on that leak would be the rosbridge-port bug wearing a new hat.
    # Passing it makes the coupling visible in the file you would actually read.
    dashboard = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('robot_dashboard'),
            'launch', 'dashboard.launch.py')),
        launch_arguments={'dashboard_params': params}.items())

    return LaunchDescription([
        can_channel, imu_driver, legs,
        odrive, imu, telemetry, leg_controller,
        balance, mode_manager, teleop, dashboard,
    ])
