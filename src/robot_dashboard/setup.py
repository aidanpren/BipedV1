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
        # The web assets get installed so the static server can serve them from
        # the install space.
        #
        # NOTE the three separate entries: glob() does NOT recurse, and
        # data_files has no notion of a directory tree. A single 'web/*' line
        # installs index.html and silently drops css/ and js/ — the page then
        # loads, renders an unstyled top bar, and does nothing else, with no
        # error anywhere in the ROS logs. Every new subdirectory under web/
        # needs its own line here.
        (os.path.join('share', package_name, 'web'), glob('web/*.html') + glob('web/*.js')),
        (os.path.join('share', package_name, 'web', 'css'), glob('web/css/*.css')),
        (os.path.join('share', package_name, 'web', 'js'), glob('web/js/*.js')),
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
            'dashboard_backend = robot_dashboard.dashboard_backend:main',
        ],
    },
)
