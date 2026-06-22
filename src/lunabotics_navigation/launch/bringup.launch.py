"""
bringup.launch.py — Lunabotics 2026 Full System Bringup
=========================================================
Single launch file that brings up the complete stack:
  1. Gazebo simulation (UCF or easy arena) — SIM ONLY
  2. Robot state publisher (URDF TF tree) — HARDWARE ONLY
  3. Nav2 navigation stack (planner, controller, BT navigator)
  4. Perception stack (obstacle detector, zone detector, localization monitor)
  5. Localization (Point-LIO + EKF, or static TF + EKF in sim)
  6. Control (cmd_vel safety relay)
  7. Autonomy (mission controller + excavation bridge)
  8. Optional: legacy waypoint navigator (for testing without full autonomy)
  9. Optional RViz2

Usage:
  # Default: UCF arena simulation, full autonomy, no RViz
  ros2 launch lunabotics_navigation bringup.launch.py

  # With RViz, upper start (simulation)
  ros2 launch lunabotics_navigation bringup.launch.py start_position:=upper use_rviz:=true

  # Legacy waypoint mode (no excavation, just navigate)
  ros2 launch lunabotics_navigation bringup.launch.py use_autonomy:=false

  # REAL HARDWARE with Point-LIO (skips Gazebo, use_sim_time=false)
  ros2 launch lunabotics_navigation bringup.launch.py use_point_lio:=true

  # Start mission automatically 25s after launch
  ros2 launch lunabotics_navigation bringup.launch.py auto_start_mission:=true
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    IncludeLaunchDescription,
    OpaqueFunction,
    TimerAction,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    Command,
    LaunchConfiguration,
    PythonExpression,
)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():

    try:
        pkg_gazebo    = get_package_share_directory('lunabotics_gazebo')
    except Exception:
        pkg_gazebo    = None  # Gazebo not available (e.g. Jetson ARM64)
    pkg_nav           = get_package_share_directory('lunabotics_navigation')
    pkg_perception    = get_package_share_directory('lunabotics_perception')
    pkg_control       = get_package_share_directory('lunabotics_control')
    pkg_localization  = get_package_share_directory('lunabotics_localization')
    pkg_autonomy      = get_package_share_directory('lunabotics_autonomy')
    pkg_description   = get_package_share_directory('lunabotics_description')

    # ------------------------------------------------------------------ #
    # Launch arguments
    # ------------------------------------------------------------------ #
    arena_arg = DeclareLaunchArgument(
        'arena', default_value='ucf',
        description='Arena to simulate: "ucf" or "easy"'
    )
    start_position_arg = DeclareLaunchArgument(
        'start_position', default_value='lower',
        description='Robot start position: "lower" or "upper"'
    )
    debug_arg = DeclareLaunchArgument(
        'debug', default_value='false',
        description='Enable perception debug topics'
    )
    use_rviz_arg = DeclareLaunchArgument(
        'use_rviz', default_value='false',
        description='Launch RViz2 for visualisation'
    )
    use_point_lio_arg = DeclareLaunchArgument(
        'use_point_lio', default_value='false',
        description='Enable Point-LIO (true on real hardware)'
    )
    use_localization_arg = DeclareLaunchArgument(
        'use_localization', default_value='true',
        description='Run Point-LIO localization (false = static TF, for testing)'
    )
    use_autonomy_arg = DeclareLaunchArgument(
        'use_autonomy', default_value='true',
        description='Use full BT autonomy (true) or legacy waypoint navigator (false)'
    )
    auto_start_mission_arg = DeclareLaunchArgument(
        'auto_start_mission', default_value='false',
        description='Auto-start mission after launch'
    )
    simulate_arg = DeclareLaunchArgument(
        'simulate', default_value='true',
        description='Simulate excavation hardware (false = real motors/actuator/servo)'
    )
    arena_layout_arg = DeclareLaunchArgument(
        'arena_layout', default_value='A',
        description='UCF arena layout: "A" (default) or "B" (mirror pit, berm Y flipped)'
    )

    arena              = LaunchConfiguration('arena')
    start_position     = LaunchConfiguration('start_position')
    debug              = LaunchConfiguration('debug')
    use_rviz           = LaunchConfiguration('use_rviz')
    use_point_lio      = LaunchConfiguration('use_point_lio')
    use_localization   = LaunchConfiguration('use_localization')
    use_autonomy       = LaunchConfiguration('use_autonomy')
    auto_start_mission = LaunchConfiguration('auto_start_mission')
    simulate           = LaunchConfiguration('simulate')
    arena_layout       = LaunchConfiguration('arena_layout')

    # Derive use_sim_time: True in sim (no Point-LIO), False on real hardware
    # This is THE critical flag — if wrong, all nodes hang waiting for /clock
    use_sim_time_str = PythonExpression(["'true' if '", use_point_lio, "' != 'true' else 'false'"])

    # ------------------------------------------------------------------ #
    # 1a. Gazebo simulation — UCF arena (SKIPPED on real hardware)
    # ------------------------------------------------------------------ #
    # 1b. Gazebo simulation — easy arena (SKIPPED on real hardware)
    # Both guarded: pkg_gazebo is None on Jetson (no gazebo_ros package)
    # ------------------------------------------------------------------ #
    gazebo_actions = []
    if pkg_gazebo is not None:
        gazebo_actions.append(GroupAction(
            condition=IfCondition(PythonExpression([
                "'", arena, "' == 'ucf' and '", use_point_lio, "' != 'true'"
            ])),
            actions=[
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(
                        os.path.join(pkg_gazebo, 'launch', 'arena_ucf.launch.py')
                    ),
                    launch_arguments={'start_position': start_position}.items()
                )
            ]
        ))
        gazebo_actions.append(GroupAction(
            condition=IfCondition(PythonExpression([
                "'", arena, "' == 'easy' and '", use_point_lio, "' != 'true'"
            ])),
            actions=[
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(
                        os.path.join(pkg_gazebo, 'launch', 'arena_easy.launch.py')
                    )
                )
            ]
        ))

    # ------------------------------------------------------------------ #
    # 1c. Robot state publisher — HARDWARE ONLY
    # In sim, Gazebo's spawn_entity handles URDF publishing.
    # On hardware, we must publish the URDF TF tree ourselves.
    # ------------------------------------------------------------------ #
    xacro_file = os.path.join(pkg_description, 'urdf', 'lunabot.urdf.xacro')
    robot_description = Command(['xacro ', xacro_file])

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        condition=IfCondition(use_point_lio),
        parameters=[{
            'robot_description': ParameterValue(robot_description, value_type=str),
            'use_sim_time': False,
        }],
    )

    # Joint state publisher — publishes fixed joint states for TF tree
    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        output='screen',
        condition=IfCondition(use_point_lio),
        parameters=[{'use_sim_time': False}],
    )

    # Unitree L2 LiDAR driver — HARDWARE ONLY (Ethernet/UDP mode)
    lidar_driver = Node(
        package='unitree_lidar_ros2',
        executable='unitree_lidar_ros2_node',
        name='unitree_lidar_ros2_node',
        output='screen',
        condition=IfCondition(use_point_lio),
        parameters=[{
            'use_sim_time': False,
            'initialize_type': 2,       # 2 = ethernet (not serial)
            'work_mode': 0,
            'use_system_timestamp': True,
            'range_min': 0.0,
            'range_max': 100.0,
            'cloud_scan_num': 18,
            'lidar_port': 6101,
            'lidar_ip': '192.168.1.62',
            'local_port': 6201,
            'local_ip': '192.168.1.2',
            'cloud_frame': 'unilidar_lidar',
            'cloud_topic': 'unilidar/cloud',
            'imu_frame': 'unilidar_imu',
            'imu_topic': 'unilidar/imu',
        }],
    )

    # ------------------------------------------------------------------ #
    # 2. Nav2 (delayed to let Gazebo or hardware initialize)
    # ------------------------------------------------------------------ #
    def _create_navigation(context):
        on_hardware = LaunchConfiguration('use_point_lio').perform(context).lower() == 'true'
        # On hardware, Nav2 can start faster (no Gazebo spawn delay)
        delay = 5.0 if on_hardware else 20.0
        return [TimerAction(
            period=delay,
            actions=[
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(
                        os.path.join(pkg_nav, 'launch', 'navigation.launch.py')
                    ),
                    launch_arguments={
                        'use_sim_time': 'false' if on_hardware else 'true',
                    }.items()
                )
            ]
        )]

    # ------------------------------------------------------------------ #
    # 3. Perception — pass use_sim_time
    # ------------------------------------------------------------------ #
    perception = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_perception, 'launch', 'perception.launch.py')
        ),
        launch_arguments={
            'debug': debug,
            'use_sim_time': use_sim_time_str,
        }.items()
    )

    # ------------------------------------------------------------------ #
    # 4. Localization (Point-LIO + EKF or static TF + EKF)
    # When use_localization:=false on hardware, use static TF (no Point-LIO)
    # ------------------------------------------------------------------ #
    def _create_localization(context):
        on_hardware = LaunchConfiguration('use_point_lio').perform(context).lower() == 'true'
        run_point_lio = LaunchConfiguration('use_localization').perform(context).lower() == 'true'

        if on_hardware and not run_point_lio:
            # Hardware without Point-LIO: static map→odom + IMU-only EKF
            ekf_imu_config = os.path.join(pkg_localization, 'config', 'ekf_params_imu_only.yaml')
            return [
                Node(
                    package='tf2_ros',
                    executable='static_transform_publisher',
                    name='map_to_odom_static',
                    arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom'],
                    parameters=[{'use_sim_time': False}],
                    output='screen',
                ),
                Node(
                    package='robot_localization',
                    executable='ekf_node',
                    name='ekf_filter_node',
                    output='screen',
                    parameters=[
                        ekf_imu_config,
                        {'publish_tf': True, 'use_sim_time': False},
                    ],
                ),
            ]
        else:
            # Normal path: full localization launch
            effective_point_lio = 'true' if (on_hardware and run_point_lio) else 'false'
            return [
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(
                        os.path.join(pkg_localization, 'launch', 'localization.launch.py')
                    ),
                    launch_arguments={'use_point_lio': effective_point_lio}.items()
                )
            ]

    # ------------------------------------------------------------------ #
    # 5. Control (cmd_vel safety relay + Teensy bridge on hardware)
    # ------------------------------------------------------------------ #
    # require_localization = false when use_localization is false (no Point-LIO)
    require_localization_str = PythonExpression([
        "'true' if '", use_localization, "' == 'true' else 'false'"
    ])

    # require_sensors defaults to false on hardware so manual teleop works
    # without LiDAR/IMU connected (safety monitor won't e-stop drivetrain)
    require_sensors_str = PythonExpression([
        "'false' if '", use_point_lio, "' == 'true' else 'true'"
    ])

    # require_odom: true on hardware (encoders connected), false in sim
    require_odom_str = PythonExpression([
        "'true' if '", use_point_lio, "' == 'true' else 'false'"
    ])

    control = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_control, 'launch', 'control.launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time_str,
            'use_hardware': use_point_lio,
            'require_sensors': require_sensors_str,
            'require_localization': require_localization_str,
            'require_odom': require_odom_str,
        }.items()
    )

    # ------------------------------------------------------------------ #
    # 6a. Full autonomy: mission controller + excavation bridge
    # ------------------------------------------------------------------ #
    autonomy = GroupAction(
        condition=IfCondition(use_autonomy),
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(pkg_autonomy, 'launch', 'autonomy.launch.py')
                ),
                launch_arguments={
                    'simulate': simulate,
                    'use_sim_time': use_sim_time_str,
                    'arena_layout': arena_layout,
                }.items()
            )
        ]
    )

    # ------------------------------------------------------------------ #
    # 6b. Legacy waypoint navigator (when use_autonomy:=false)
    # ------------------------------------------------------------------ #
    def _create_waypoint_nav(context):
        on_hardware = LaunchConfiguration('use_point_lio').perform(context).lower() == 'true'
        return [Node(
            package='lunabotics_navigation',
            executable='waypoint_navigator',
            name='waypoint_navigator',
            output='screen',
            condition=UnlessCondition(LaunchConfiguration('use_autonomy')),
            parameters=[{
                'use_sim_time': not on_hardware,
                'start_position': LaunchConfiguration('start_position'),
                'auto_start': LaunchConfiguration('auto_start_mission'),
            }]
        )]

    # ------------------------------------------------------------------ #
    # 7. Optional RViz2
    # ------------------------------------------------------------------ #
    rviz_config = os.path.join(pkg_nav, 'rviz', 'lunabotics.rviz')

    def _create_rviz(context):
        on_hardware = LaunchConfiguration('use_point_lio').perform(context).lower() == 'true'
        return [Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config],
            parameters=[{'use_sim_time': not on_hardware}],
            condition=IfCondition(LaunchConfiguration('use_rviz'))
        )]

    return LaunchDescription([
        arena_arg,
        start_position_arg,
        debug_arg,
        use_rviz_arg,
        use_point_lio_arg,
        use_localization_arg,
        use_autonomy_arg,
        auto_start_mission_arg,
        simulate_arg,
        arena_layout_arg,
        # Simulation (Gazebo) — skipped if package not available
        *gazebo_actions,
        # Hardware (robot_state_publisher + LiDAR driver)
        robot_state_publisher,
        joint_state_publisher,
        lidar_driver,
        # Common stack
        OpaqueFunction(function=_create_navigation),
        perception,
        OpaqueFunction(function=_create_localization),
        control,
        autonomy,
        OpaqueFunction(function=_create_waypoint_nav),
        OpaqueFunction(function=_create_rviz),
    ])
