import os
import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    pkg_gazebo = get_package_share_directory('lunabotics_gazebo')
    pkg_description = get_package_share_directory('lunabotics_description')
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')

    # Make our custom models available to Gazebo
    gazebo_model_path = os.path.join(pkg_gazebo, 'models')
    if 'GAZEBO_MODEL_PATH' in os.environ:
        os.environ['GAZEBO_MODEL_PATH'] += ':' + gazebo_model_path
    else:
        os.environ['GAZEBO_MODEL_PATH'] = gazebo_model_path

    # --- Launch arguments ---
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time', default_value='true',
        description='Use simulation clock'
    )
    verbose_arg = DeclareLaunchArgument(
        'verbose', default_value='false',
        description='Enable Gazebo verbose output'
    )

    use_sim_time = LaunchConfiguration('use_sim_time')
    verbose = LaunchConfiguration('verbose')

    # --- Gazebo with easy arena ---
    world_file = os.path.join(pkg_gazebo, 'worlds', 'arena_easy.world')

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gazebo.launch.py')
        ),
        launch_arguments={
            'world': world_file,
            'verbose': verbose,
        }.items()
    )

    # --- Robot URDF ---
    xacro_file = os.path.join(pkg_description, 'urdf', 'lunabot.urdf.xacro')
    robot_description_content = xacro.process_file(xacro_file).toxml()

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description_content,
            'use_sim_time': use_sim_time,
        }]
    )

    # --- Spawn robot in starting zone ---
    # Position: (0.5, -1.5, 0.2), yaw=1.57 rad (facing +Y / toward obstacle zone)
    spawn_robot = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-topic', 'robot_description',
            '-entity', 'lunabot',
            '-x', '0.5',
            '-y', '-1.5',
            '-z', '0.2',
            '-Y', '1.57',
        ],
        output='screen'
    )

    return LaunchDescription([
        use_sim_time_arg,
        verbose_arg,
        gazebo,
        robot_state_publisher,
        spawn_robot,
    ])
