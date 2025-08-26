from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

from ros_gz_sim.actions import GzServer, GzSim


def generate_launch_description():
    pkg_description = get_package_share_directory('disaster_bot_description')

    urdf_file = os.path.join(pkg_description, 'urdf', 'disaster_bot.urdf')
    world_file = os.path.join(pkg_description, 'worlds', 'disaster_world.sdf')

    declare_world_arg = DeclareLaunchArgument(
        'world',
        default_value=world_file,
        description='Full path to the world file to load'
    )

    robot_description = open(urdf_file).read()

    return LaunchDescription([
        declare_world_arg,

        # Launch Gazebo server with the selected world
        GzServer(
            world=LaunchConfiguration('world')
        ),

        # Launch Gazebo client (GUI)
        GzSim(),

        # Robot State Publisher
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_description}],
        ),

        # Spawn entity into Gazebo Sim
        Node(
            package='ros_gz_sim',
            executable='create',
            arguments=['-name', 'disaster_bot', '-file', urdf_file],
            output='screen',
        ),
    ])
