"""
control.launch.py — Lunabotics 2026 Control Stack
===================================================
Launches the cmd_vel safety relay + optional gamepad teleop.

Usage:
  # Launch relay only (normal operation — Nav2 drives the robot)
  ros2 launch lunabotics_control control.launch.py

  # With gamepad teleop (Switch Pro / Xbox controller):
  ros2 launch lunabotics_control control.launch.py use_joystick:=true

  # Manual keyboard teleoperation (separate terminal):
  ros2 run lunabotics_control teleop_keyboard
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    pkg = get_package_share_directory('lunabotics_control')
    params = os.path.join(pkg, 'config', 'control_params.yaml')
    pico_params = os.path.join(pkg, 'config', 'pico_params.yaml')

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time', default_value='true',
        description='Use simulation clock'
    )

    use_joystick_arg = DeclareLaunchArgument(
        'use_joystick', default_value='false',
        description='Launch gamepad teleop (joy_node + teleop_joy)'
    )

    use_hardware_arg = DeclareLaunchArgument(
        'use_hardware', default_value='false',
        description='Launch Pico serial bridge for real motors'
    )

    require_sensors_arg = DeclareLaunchArgument(
        'require_sensors', default_value='true',
        description='Require LiDAR/IMU for safety (false for proof-of-life motor testing)'
    )

    require_localization_arg = DeclareLaunchArgument(
        'require_localization', default_value='true',
        description='Require localization quality for safety (false when testing without Point-LIO)'
    )

    relay = Node(
        package='lunabotics_control',
        executable='cmd_vel_relay',
        name='cmd_vel_relay',
        output='screen',
        parameters=[params, {'use_sim_time': LaunchConfiguration('use_sim_time')}]
    )

    require_odom_arg = DeclareLaunchArgument(
        'require_odom', default_value='false',
        description='Require wheel encoder odom (true when encoders are connected)'
    )

    safety_monitor = Node(
        package='lunabotics_control',
        executable='safety_monitor',
        name='safety_monitor',
        output='screen',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'require_sensors': LaunchConfiguration('require_sensors'),
            'require_localization': LaunchConfiguration('require_localization'),
            'require_odom': LaunchConfiguration('require_odom'),
        }]
    )

    joy_node = Node(
        package='joy',
        executable='joy_node',
        name='joy_node',
        output='screen',
        parameters=[params, {'use_sim_time': LaunchConfiguration('use_sim_time')}],
        condition=IfCondition(LaunchConfiguration('use_joystick'))
    )

    teleop_joy = Node(
        package='lunabotics_control',
        executable='teleop_joy',
        name='teleop_joy',
        output='screen',
        parameters=[params, {'use_sim_time': LaunchConfiguration('use_sim_time')}],
        condition=IfCondition(LaunchConfiguration('use_joystick'))
    )

    pico_bridge = Node(
        package='lunabotics_control',
        executable='pico_bridge',
        name='pico_bridge',
        output='screen',
        parameters=[pico_params, {'use_sim_time': LaunchConfiguration('use_sim_time')}],
        condition=IfCondition(LaunchConfiguration('use_hardware'))
    )

    servo_driver = Node(
        package='lunabotics_control',
        executable='servo_driver',
        name='servo_driver',
        output='screen',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'serial_port': '/dev/serial/by-id/usb-1a86_USB_Single_Serial_5B14032921-if00',
            'baud_rate': 1000000,
            'servo_id': 1,
            'stowed_pos': 3283,
            'poll_rate': 5.0,
        }],
        condition=IfCondition(LaunchConfiguration('use_hardware'))
    )

    system_watchdog = Node(
        package='lunabotics_control',
        executable='system_watchdog',
        name='system_watchdog',
        output='screen',
        condition=IfCondition(LaunchConfiguration('use_hardware')),
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'check_rate': 1.0,
            'lidar_ip': '192.168.1.62',
            'jetson_ip': '192.168.1.2',
            'network_interface': 'enP8p1s0',
            'enable_network_recovery': True,
            'enable_node_restart': True,
            'enable_nav_recovery': True,
            'startup_grace_period': 15.0,
        }]
    )

    return LaunchDescription([
        use_sim_time_arg,
        use_joystick_arg,
        use_hardware_arg,
        require_sensors_arg,
        require_localization_arg,
        require_odom_arg,
        relay,
        safety_monitor,
        joy_node,
        teleop_joy,
        pico_bridge,
        servo_driver,
        system_watchdog,
    ])
