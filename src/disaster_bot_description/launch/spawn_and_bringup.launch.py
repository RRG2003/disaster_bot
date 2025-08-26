from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_dir = get_package_share_directory('disaster_bot_description')
    urdf_path = os.path.join(pkg_dir, 'urdf', 'disaster_bot.urdf')

    # Start gz sim (empty world) using ros_gz default launch or using a direct process
    gz = Node(
        package='ros_gz_sim',
        executable='gzserver',  # if your ros_gz packages provide wrapper; else run gz sim externally
        arguments=['--verbose'],
        output='screen'
    )

    # robot_state_publisher
    rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'use_sim_time': True, 'robot_description': open(urdf_path).read()}],
        output='screen'
    )

    # spawn entity (this uses ros_gz's spawn entity script if available)
    spawn = Node(
        package='ros_gz_sim',
        executable='create',
        name='spawn_entity',
        output='screen',
        # Alternative: use 'ros_gz_sim.create' or spawn_entity.py script from ros_gz
        arguments=['-file', urdf_path, '-name', 'disaster_bot']
    )

    # Bring up core nodes
    motor = Node(package='motor_control', executable='motor_control_node', output='screen')
    teleop = Node(package='teleop_control', executable='teleop_node', output='screen')
    compressor = Node(package='air_compressor_control', executable='air_compressor_control_node', output='screen')

    return LaunchDescription([gz, rsp, spawn, motor, teleop, compressor])

# Note: the exact ros_gz executables / arguments might differ between installs. 
# If the spawn Node fails, use the ros2 run ros_gz_sim create CLI or gz sim in a
#  separate terminal and then call ros2 run ros_gz_sim create -file ... to spawn.