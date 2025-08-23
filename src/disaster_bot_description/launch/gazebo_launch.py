from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
    pkg_description = get_package_share_directory('disaster_bot_description')

    # Path to URDF and world files
    urdf_file = os.path.join(pkg_description, 'urdf', 'disaster_bot.urdf')
    world_file = os.path.join(pkg_description, 'worlds', 'disaster_world.world')

    return LaunchDescription([
        # Start Gazebo with the world file
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_gazebo_ros, 'launch', 'gazebo.launch.py')),
            launch_arguments={'world': world_file}.items(),
        ),

        # Robot State Publisher
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': open(urdf_file).read()}]
        ),

        # Spawn robot entity
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=['-entity', 'disaster_bot', '-file', urdf_file],
            output='screen'
        ),
    ])
