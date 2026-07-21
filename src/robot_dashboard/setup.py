import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'robot_dashboard'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        # the web assets (index.html + vendored roslib.min.js) get installed so
        # the static server can serve them from the install space.
        (os.path.join('share', package_name, 'web'), glob('web/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='roshub',
    maintainer_email='aidan.pren14@gmail.com',
    description='Web dashboard (browser/phone) for monitoring and mode control',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
        ],
    },
)
