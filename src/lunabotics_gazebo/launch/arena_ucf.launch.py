import os
import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    IncludeLaunchDescription,
    DeclareLaunchArgument,
    OpaqueFunction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _spawn_robot(context):
    """Resolve start_position at launch time and return the spawn node."""
    start_pos = LaunchConfiguration('start_position').perform(context)
    use_sim_time = LaunchConfiguration('use_sim_time').perform(context)

    # Lower start: Y=-1.0   Upper start: Y=+1.0
    spawn_y = '-1.00' if start_pos == 'lower' else '1.00'

    pkg_description = get_package_share_directory('lunabotics_description')
    xacro_file = os.path.join(pkg_description, 'urdf', 'lunabot.urdf.xacro')
    robot_description_content = xacro.process_file(xacro_file).toxml()

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description_content,
            'use_sim_time': use_sim_time == 'true',
        }]
    )

    spawn_robot = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-topic', 'robot_description',
            '-entity', 'lunabot',
            '-x', '3.47',
            '-y', spawn_y,
            '-z', '0.20',
            '-Y', '3.14159',  # facing -X (toward construction zone)
        ],
        output='screen'
    )

    return [robot_state_publisher, spawn_robot]


def generate_launch_description():

    pkg_gazebo = get_package_share_directory('lunabotics_gazebo')
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
    pkg_description = get_package_share_directory('lunabotics_description')

    # Make our custom models available to Gazebo
    gazebo_model_path = os.path.join(pkg_gazebo, 'models')
    # Add lunabotics_description share parent so Gazebo resolves model://lunabotics_description/meshes/...
    pkg_description_parent = os.path.dirname(pkg_description)
    if 'GAZEBO_MODEL_PATH' in os.environ:
        os.environ['GAZEBO_MODEL_PATH'] += ':' + gazebo_model_path + ':' + pkg_description_parent
    else:
        os.environ['GAZEBO_MODEL_PATH'] = gazebo_model_path + ':' + pkg_description_parent

    # --- Launch arguments ---
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time', default_value='true',
        description='Use simulation clock'
    )
    verbose_arg = DeclareLaunchArgument(
        'verbose', default_value='false',
        description='Enable Gazebo verbose output'
    )
    start_position_arg = DeclareLaunchArgument(
        'start_position', default_value='lower',
        description='Robot starting position: "lower" (Y=-1.0) or "upper" (Y=1.0)'
    )

    verbose = LaunchConfiguration('verbose')

    # --- Gazebo with UCF arena ---
    world_file = os.path.join(pkg_gazebo, 'worlds', 'arena_ucf.world')

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gazebo.launch.py')
        ),
        launch_arguments={
            'world': world_file,
            'verbose': verbose,
        }.items()
    )

    return LaunchDescription([
        use_sim_time_arg,
        verbose_arg,
        start_position_arg,
        gazebo,
        OpaqueFunction(function=_spawn_robot),
    ])
