import os
from glob import glob
from setuptools import setup, find_packages

package_name = 'lunabotics_autonomy'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'behavior_trees'), glob('behavior_trees/*.xml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='PFW Lunabotics Team',
    maintainer_email='lunabotics@pfw.edu',
    description='BT mission control for Lunabotics 2026',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'mission_controller = lunabotics_autonomy.autonomy.mission_controller:main',
            'excavation_bridge = lunabotics_autonomy.autonomy.excavation_bridge:main',
        ],
    },
)
