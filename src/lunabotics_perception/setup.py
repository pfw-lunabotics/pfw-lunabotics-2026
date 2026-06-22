from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'lunabotics_perception'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Install launch files
        (os.path.join('share', package_name, 'launch'), 
            glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'launch'), 
            glob('launch/*.py')),
        # Install config files
        (os.path.join('share', package_name, 'config'), 
            glob('config/*.yaml')),
        (os.path.join('share', package_name, 'config'), 
            glob('config/*.yml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='PFW Lunabotics Team',
    maintainer_email='lunabotics@pfw.edu',
    description='Unified obstacle perception for Lunabotics 2026',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'unified_obstacle_detector = lunabotics_perception.perception.unified_obstacle_detector:main',
            'zone_detector = lunabotics_perception.perception.zone_detector:main',
            'localization_quality_monitor = lunabotics_perception.perception.localization_quality_monitor:main',
        ],
    },
)
