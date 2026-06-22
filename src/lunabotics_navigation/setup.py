from setuptools import setup
from glob import glob
import os

package_name = 'lunabotics_navigation'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Install launch files
        (os.path.join('share', package_name, 'launch'), 
            glob('launch/*.launch.py')),
        # Install config files (YAML + BT XML)
        (os.path.join('share', package_name, 'config'),
            glob('config/*.yaml') + glob('config/*.xml')),
        # Install map files
        (os.path.join('share', package_name, 'maps'),
            glob('maps/*')),
        # Install RViz configs
        (os.path.join('share', package_name, 'rviz'),
            glob('rviz/*.rviz')),
        # Install executables to lib/<pkg>/ so ros2 run can find them
        (os.path.join('lib', package_name),
            glob('scripts/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='PFW Lunabotics',
    maintainer_email='lunabotics@pfw.edu',
    description='Navigation for Lunabotics 2026',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'waypoint_navigator = lunabotics_navigation.waypoint_navigator:main',
            'wander_test = lunabotics_navigation.wander_test:main',
            'reactive_wander = lunabotics_navigation.reactive_wander:main',
        ],
    },
)
