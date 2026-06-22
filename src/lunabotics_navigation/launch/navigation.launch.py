import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    pkg_nav = get_package_share_directory('lunabotics_navigation')

    nav2_params_file = os.path.join(pkg_nav, 'config', 'nav2_params.yaml')
    # Custom BT: replan-if-invalid + no spin recovery (regolith safe)
    nav_bt_xml = os.path.join(pkg_nav, 'config', 'nav_bt_no_spin.xml')
    # Blank map — no a priori wall info (rule 5.6.3 compliance)
    # Walls detected live by LiDAR via costmap obstacle layer
    default_map_file  = os.path.join(pkg_nav, 'maps', 'blank_arena.yaml')

    map_yaml_arg = DeclareLaunchArgument(
        'map_yaml_file',
        default_value=default_map_file,
        description='Full path to the Nav2 map YAML file'
    )
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time', default_value='true',
        description='Use simulation time (false on real hardware)'
    )

    map_yaml = LaunchConfiguration('map_yaml_file')
    use_sim_time = LaunchConfiguration('use_sim_time')

    return LaunchDescription([
        map_yaml_arg,
        use_sim_time_arg,

        # Map server — publishes /map for global costmap static_layer
        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            parameters=[nav2_params_file, {
                'yaml_filename': map_yaml,
                'use_sim_time': use_sim_time,
            }]
        ),

        Node(
            package='nav2_controller',
            executable='controller_server',
            output='screen',
            parameters=[nav2_params_file, {'use_sim_time': use_sim_time}]
        ),
        Node(
            package='nav2_planner',
            executable='planner_server',
            output='screen',
            parameters=[nav2_params_file, {'use_sim_time': use_sim_time}]
        ),

        # Behavior server — provides spin, backup, wait recovery actions
        Node(
            package='nav2_behaviors',
            executable='behavior_server',
            name='behavior_server',
            output='screen',
            parameters=[nav2_params_file, {'use_sim_time': use_sim_time}]
        ),

        Node(
            package='nav2_bt_navigator',
            executable='bt_navigator',
            output='screen',
            parameters=[nav2_params_file, {
                'use_sim_time': use_sim_time,
                'default_nav_to_pose_bt_xml': nav_bt_xml,
            }]
        ),
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'autostart': True,
                'bond_timeout': 20.0,
                'attempt_respawn_reconnection': True,
                'service_call_timeout': 20,
                'node_names': [
                    'map_server',
                    'controller_server',
                    'planner_server',
                    'behavior_server',
                    'bt_navigator',
                ]
            }]
        ),
    ])
