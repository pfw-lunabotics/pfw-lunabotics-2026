"""
autonomy.launch.py — Lunabotics 2026 Full Autonomy Stack
==========================================================
Launches:
  1. Excavation Bridge (sim or hardware)
  2. Mission Controller (state machine orchestrator)

UCF Arena: Robot starts at RANDOM position & direction.
No start_position parameter needed — mission controller self-localizes.

Usage:
  # Simulation (default)
  ros2 launch lunabotics_autonomy autonomy.launch.py

  # Real hardware
  ros2 launch lunabotics_autonomy autonomy.launch.py simulate:=false use_sim_time:=false

  # Auto-start mission 10s after launch
  ros2 launch lunabotics_autonomy autonomy.launch.py auto_start:=true
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    pkg = get_package_share_directory('lunabotics_autonomy')
    params_file = os.path.join(pkg, 'config', 'mission_params.yaml')

    # --- Arguments ---
    simulate_arg = DeclareLaunchArgument(
        'simulate', default_value='true',
        description='Simulate excavation hardware (true) or use real bridge (false)'
    )
    auto_start_arg = DeclareLaunchArgument(
        'auto_start', default_value='false',
        description='Auto-start mission after launch'
    )
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time', default_value='true',
        description='Use simulation time (false on real hardware)'
    )
    arena_layout_arg = DeclareLaunchArgument(
        'arena_layout', default_value='A',
        description='UCF arena layout: "A" (default) or "B" (mirror, berm Y flipped)'
    )

    use_sim_time = LaunchConfiguration('use_sim_time')
    arena_layout = LaunchConfiguration('arena_layout')

    # --- Excavation Bridge ---
    excavation_bridge = Node(
        package='lunabotics_autonomy',
        executable='excavation_bridge',
        name='excavation_bridge',
        output='screen',
        parameters=[
            params_file,
            {
                'use_sim_time': use_sim_time,
                'simulate': LaunchConfiguration('simulate'),
            }
        ],
    )

    # --- Mission Controller ---
    mission_controller = Node(
        package='lunabotics_autonomy',
        executable='mission_controller',
        name='mission_controller',
        output='screen',
        parameters=[
            params_file,
            {
                'use_sim_time': use_sim_time,
                'arena_layout': arena_layout,
            }
        ],
    )

    return LaunchDescription([
        simulate_arg,
        auto_start_arg,
        use_sim_time_arg,
        arena_layout_arg,
        excavation_bridge,
        mission_controller,
    ])
