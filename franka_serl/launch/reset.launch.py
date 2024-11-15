import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # Declare the 'robot_ip' argument
    robot_ip_arg = DeclareLaunchArgument(
        'robot_ip',
        default_value='172.16.0.2',
        description='IP address of the Franka robot'
    )

    load_gripper_arg = DeclareLaunchArgument(
        'load_gripper',
        default_value='True',
        description='Load gripper configuration'
    )

    # Use LaunchConfiguration to get the value of the argument
    bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('franka_bringup'), 'launch/real'), '/franka.launch.py'
        ]),
        launch_arguments={
            'robot_ip': LaunchConfiguration('robot_ip'),
            'load_gripper': LaunchConfiguration('load_gripper')
        }.items()
    )

    controller_manager = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['move_to_start_example_controller'],
        output='screen'
    )

    gripper = Node(
        package='franka_gripper_server',
        executable='franka_gripper_server_node',
        name='franka_gripper_server_node',
        output='screen',
    )

    return LaunchDescription([
        robot_ip_arg,
        load_gripper_arg,
        bringup,
        controller_manager,
        gripper,
    ])
