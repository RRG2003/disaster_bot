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

        # Start Gazebo Sim with the chosen world
        GzServer(
            world=LaunchConfiguration('world'),
            verbose=True
        ),

        # Gazebo GUI
        GzSim(),

        # Robot State Publisher (sync to sim time)
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_description,
                         'use_sim_time': True}],
        ),

        # Spawn robot into simulation
        Node(
            package='ros_gz_sim',
            executable='create',
            arguments=[
                '-name', 'disaster_bot',
                '-file', urdf_file,
                '-x', '0', '-y', '0', '-z', '0.2'
            ],
            output='screen',
        ),
    ])
