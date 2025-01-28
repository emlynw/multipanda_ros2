import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution


def generate_launch_description():

    config = PathJoinSubstitution([
        get_package_share_directory('franka_serl'),
        'config',
        'cartesian_impedance_controller.yaml'
    ])

    bringup = IncludeLaunchDescription(
      PythonLaunchDescriptionSource([os.path.join(
         get_package_share_directory('franka_bringup'), 'launch/real'), '/franka.launch.py']),
        launch_arguments = {
            'robot_ip': '172.16.0.2',
            'load_gripper': 'True'  
        }.items()                       
      )

    set_load = ExecuteProcess(
        cmd=[
            'ros2', 'service', 'call',
            '/panda_param_service_server/set_load', 'franka_msgs/srv/SetLoad',
            '{mass: 0.23, center_of_mass: [0.0, 0.0, 0.1], load_inertia: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]}'
        ],
        output='screen'
    )

    delayed_set_load = TimerAction(period=4.0, actions=[set_load])

    controller_manager = Node(
    package='controller_manager',
    executable='spawner',
    arguments=[
        'cartesian_impedance_controller',
        '--controller-manager', '/controller_manager',
        '--param-file', config,
        '--controller-manager-timeout', '50'  # Correct argument name
    ],
    output='screen'
    )

    delayed_controller_manager = TimerAction(period=8.0, actions=[controller_manager])

    move_to_start_controller = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'move_to_start_controller',
            '--controller-manager', '/controller_manager',
            '--controller-manager-timeout', '50',
            '--inactive'  # Ensure this controller starts in an inactive state
        ],
        output='screen'
    )

    delayed_move_to_start_controller = TimerAction(period=10.0, actions=[move_to_start_controller])

    gripper = Node(
            package='franka_serl',
            executable='franka_gripper_server_node',
            name='franka_gripper_server_node',
            output='screen',
        )

    return LaunchDescription([
        bringup,
        delayed_set_load,
        delayed_controller_manager,
        delayed_move_to_start_controller,
        gripper,
   ])