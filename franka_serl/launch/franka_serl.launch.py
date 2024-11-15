import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():

    bringup = IncludeLaunchDescription(
      PythonLaunchDescriptionSource([os.path.join(
         get_package_share_directory('franka_bringup'), 'launch/real'), '/franka.launch.py']),
        launch_arguments = {
            'robot_ip': '172.16.0.2',
            'load_gripper': 'True'  
        }.items()                       
      )

    controller_manager = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['cartesian_impedance_controller'],
        output='screen'
    )

    gripper = Node(
            package='franka_gripper_server',
            executable='franka_gripper_server_node',
            name='franka_gripper_server_node',
            output='screen',
        )

    return LaunchDescription([
        bringup,
        controller_manager,
        gripper,
   ])