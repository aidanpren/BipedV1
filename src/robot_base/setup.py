from setuptools import find_packages, setup

package_name = 'robot_base'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='roshub',
    maintainer_email='aidan.pren14@gmail.com',
    description='Balance loop, ODrive CAN bridges (wheels + legs), and IMU node.',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'balance_controller = robot_base.balance_controller:main',
            'odrive_bridge = robot_base.odrive_bridge:main',
            'odrive_telemetry = robot_base.odrive_telemetry:main',
            'imu_node = robot_base.imu_node:main',
            'leg_controller = robot_base.leg_controller:main'
        ],
    },
)
