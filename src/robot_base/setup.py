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
    description='Bridges and interfaces to the Pico w/ fake telemetry for now',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'fake_pico = robot_base.fake_pico:main',
            'imu_monitor = robot_base.imu_monitor:main'
        ],
    },
)
