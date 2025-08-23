from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command, LaunchConfiguration
from launch.actions import DeclareLaunchArgument

def generate_launch_description():
    robot_description = Command(['xacro ', LaunchConfiguration('model')])

    return LaunchDescription([
        DeclareLaunchArgument(
            'model',
            default_value='urdf/disaster_bot.urdf',
            description='URDF file path'),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            output='both',
            parameters=[{'robot_description': robot_description}],
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
        )
    ])
