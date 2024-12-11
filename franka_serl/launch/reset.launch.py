import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    
    controller_manager = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['move_to_start_controller'],
        output='screen'
    )

    

    return LaunchDescription([
        controller_manager,
    ])
