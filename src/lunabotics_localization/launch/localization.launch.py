"""
localization.launch.py — Lunabotics 2026 Localization Stack
============================================================
Two-layer localization:

  Layer 1: Point-LIO (LiDAR-inertial odometry)
    - Subscribes: /unilidar/cloud, /unilidar/imu
    - Publishes: map → odom TF, /point_lio/odom (Odometry)
    - Only runs on real hardware (use_point_lio:=true)

  Layer 2: EKF (robot_localization)
    - Sim: Fuses /odom (wheel encoders) + /unilidar/imu
    - Hardware: IMU-only (no wheel encoders on this robot)
    - Publishes: /odometry/filtered (Odometry)

In simulation: Point-LIO is disabled, static map→odom identity is used.
               EKF fuses wheel odom + IMU (Gazebo publishes odom→base_footprint).
On hardware:  Point-LIO provides map→odom.
               EKF runs IMU-only and publishes odom→base_footprint.
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    pkg_localization = get_package_share_directory('lunabotics_localization')
    ekf_config_sim = os.path.join(pkg_localization, 'config', 'ekf_params.yaml')
    ekf_config_hw  = os.path.join(pkg_localization, 'config', 'ekf_params_hardware.yaml')
    ekf_config_imu = os.path.join(pkg_localization, 'config', 'ekf_params_imu_only.yaml')

    # --- Launch arguments ---
    use_point_lio_arg = DeclareLaunchArgument(
        'use_point_lio', default_value='false',
        description='Enable Point-LIO (set true on real hardware, false in sim)'
    )
    use_point_lio = LaunchConfiguration('use_point_lio')

    # ------------------------------------------------------------------ #
    # Layer 1a: Point-LIO — real hardware only
    # Provides map → odom TF via LiDAR-inertial odometry
    # ------------------------------------------------------------------ #
    point_lio_config = PathJoinSubstitution([
        FindPackageShare('lunabotics_localization'),
        'config', 'point_lio_l2.yaml'
    ])

    point_lio_node = Node(
        package='point_lio',
        executable='pointlio_mapping',
        name='point_lio',
        output='screen',
        condition=IfCondition(use_point_lio),
        parameters=[
            point_lio_config,
            {
                'use_imu_as_input': False,
                'prop_at_freq_of_imu': True,
                'check_satu': True,
                'init_map_size': 10,
                'point_filter_num': 6,       # Keep every 6th point (aggressive downsample for Jetson)
                'space_down_sample': True,
                'filter_size_surf': 0.5,    # 50cm voxel grid (reduce CPU load)
                'filter_size_map': 0.5,     # 50cm map voxels
                'cube_side_length': 1000.0,
                'runtime_pos_log_enable': False,
                # Odometry-only mode: just TF + odom, no heavy map clouds
                'odom_only': True,
                'odom_header_frame_id': 'map',
                'odom_child_frame_id': 'odom',
            }
        ],
        remappings=[
            # Output odom topic renamed for clarity
            ('/odom_corrected', '/point_lio/odom'),
        ],
    )

    # ------------------------------------------------------------------ #
    # Layer 1b: Static map → odom TF — simulation fallback
    # Identity transform when Point-LIO is not running
    # ------------------------------------------------------------------ #
    static_map_to_odom = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='map_to_odom_tf',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom'],
        parameters=[{'use_sim_time': True}],
        condition=UnlessCondition(use_point_lio),
        output='screen',
    )

    # ------------------------------------------------------------------ #
    # Layer 2: EKF — fuses sensor data → /odometry/filtered
    #
    # Sim:      wheel odom + IMU, publish_tf=false (Gazebo handles TF)
    # Hardware: IMU-only (no encoders), publish_tf=true (EKF handles TF)
    # ------------------------------------------------------------------ #
    def _create_ekf_node(context):
        on_hardware = LaunchConfiguration('use_point_lio').perform(context).lower() == 'true'
        if on_hardware:
            # Check if Point-LIO will actually run — if not, use IMU-only config
            # Use hardware config (with Point-LIO odom source) by default;
            # EKF gracefully handles missing odom0 by running on IMU alone
            config = ekf_config_hw
        else:
            config = ekf_config_sim
        return [Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            output='screen',
            parameters=[
                config,
                {
                    'publish_tf': on_hardware,
                    'use_sim_time': not on_hardware,
                },
            ],
        )]

    return LaunchDescription([
        use_point_lio_arg,
        point_lio_node,
        static_map_to_odom,
        OpaqueFunction(function=_create_ekf_node),
    ])
