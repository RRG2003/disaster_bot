from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    description_pkg = get_package_share_directory('disaster_bot_description')

    return LaunchDescription([
        # URDF + state publisher
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(description_pkg, 'launch', 'gazebo_launch.py')
            )
        ),

        # Control nodes
        Node(package='motor_control', executable='motor_control_node', output='screen'),
        Node(package='air_compressor_control', executable='air_compressor_control_node', output='screen'),
        Node(package='vine_control', executable='vine_control_node', output='screen'),
        Node(package='teleop_control', executable='teleop_control_node', output='screen'),

        # SLAM Toolbox
        Node(
            package='slam_toolbox',
            executable='sync_slam_toolbox_node',
            name='slam_toolbox',
            output='screen'
        ),
    ])
