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
            '{mass: 0.216, center_of_mass: [0.0, 0.0, 0.1], load_inertia: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]}'
        ],
        output='screen'
    )

    delayed_set_load = TimerAction(period=4.0, actions=[set_load])

    controller_manager = Node(
        package='controller_manager',
        executable='spawner',
        parameters=[config],
        arguments=['cartesian_impedance_controller'],
        output='screen'
    )

    delayed_controller_manager = TimerAction(period=8.0, actions=[controller_manager])

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
        gripper,
   ])