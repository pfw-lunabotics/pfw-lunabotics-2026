#!/usr/bin/env python3
"""
Perception Launch File for Lunabotics 2026
===========================================

Launches the unified obstacle detection pipeline.

Usage:
    ros2 launch lunabotics_perception perception.launch.py
    ros2 launch lunabotics_perception perception.launch.py debug:=true
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # Get package directory
    pkg_dir = get_package_share_directory('lunabotics_perception')
    
    # Launch arguments
    debug_arg = DeclareLaunchArgument(
        'debug',
        default_value='false',
        description='Enable debug mode (publishes debug point clouds)'
    )
    
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation time (Gazebo clock)'
    )
    
    # Parameters file
    params_file = os.path.join(pkg_dir, 'config', 'perception_params.yaml')
    
    # Unified Obstacle Detector Node
    obstacle_detector_node = Node(
        package='lunabotics_perception',
        executable='unified_obstacle_detector',
        name='unified_obstacle_detector',
        output='screen',
        parameters=[
            params_file,
            {
                'debug_mode': LaunchConfiguration('debug'),
                'use_sim_time': LaunchConfiguration('use_sim_time'),
            }
        ],
        remappings=[
            # Unitree L2 publishes on /unilidar/cloud (both sim and real)
            # No remapping needed — node subscribes directly to /unilidar/cloud
        ]
    )

    # Zone Detector Node
    zone_detector_node = Node(
        package='lunabotics_perception',
        executable='zone_detector',
        name='zone_detector',
        output='screen',
        parameters=[
            params_file,
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
        ],
    )

    # Localization Quality Monitor Node
    localization_quality_node = Node(
        package='lunabotics_perception',
        executable='localization_quality_monitor',
        name='localization_quality_monitor',
        output='screen',
        parameters=[
            params_file,
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
        ],
    )

    return LaunchDescription([
        debug_arg,
        use_sim_time_arg,
        obstacle_detector_node,
        zone_detector_node,
        localization_quality_node,
    ])
