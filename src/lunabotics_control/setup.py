from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'lunabotics_control'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'),
            glob('config/*.yaml')),
        (os.path.join('lib', package_name),
            glob('scripts/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='PFW Lunabotics',
    maintainer_email='lunabotics@pfw.edu',
    description='Control stack for Lunabotics 2026',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'cmd_vel_relay = lunabotics_control.cmd_vel_relay:main',
            'teleop_keyboard = lunabotics_control.teleop_keyboard:main',
            'safety_monitor = lunabotics_control.safety_monitor:main',
            'pico_bridge = lunabotics_control.pico_bridge:main',
            'servo_driver = lunabotics_control.servo_driver:main',
            'system_watchdog = lunabotics_control.system_watchdog:main',
            'teleop_joy = lunabotics_control.teleop_joy:main',
        ],
    },
)
