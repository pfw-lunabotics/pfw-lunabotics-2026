import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import xacro

def generate_launch_description():
    
    pkg_gazebo = get_package_share_directory('lunabotics_gazebo')
    pkg_description = get_package_share_directory('lunabotics_description')
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
    
    # Set Gazebo model path to include our models
    gazebo_model_path = os.path.join(pkg_gazebo, 'models')
    if 'GAZEBO_MODEL_PATH' in os.environ:
        os.environ['GAZEBO_MODEL_PATH'] += ':' + gazebo_model_path
    else:
        os.environ['GAZEBO_MODEL_PATH'] = gazebo_model_path
    
    # NASA Arena world file
    world_file = os.path.join(pkg_gazebo, 'worlds', 'arena_nasa.world')
    
    # Launch Gazebo with NASA arena
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gazebo.launch.py')
        ),
        launch_arguments={
            'world': world_file,
            'verbose': 'true'
        }.items()
    )
    
    # Process robot URDF
    xacro_file = os.path.join(pkg_description, 'urdf', 'lunabot.urdf.xacro')
    robot_description = xacro.process_file(xacro_file).toxml()
    
    # Robot State Publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': True
        }]
    )
    
    # Spawn robot at starting position (near AprilTag board)
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-topic', 'robot_description',
            '-entity', 'lunabot',
            '-x', '0.5',
            '-y', '-1.5',
            '-z', '0.2',
            '-Y', '1.57'  # Face forward
        ],
        output='screen'
    )
    
    return LaunchDescription([
        gazebo,
        robot_state_publisher,
        spawn_entity
    ])
