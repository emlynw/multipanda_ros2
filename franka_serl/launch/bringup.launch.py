def generate_launch_description():

    bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                os.path.join(
                    get_package_share_directory('franka_bringup'), 'launch/real'
                ),
                '/franka.launch.py'
            ]
        ),
        launch_arguments={
            'robot_ip': '172.16.0.2',
            'load_gripper': 'True',
            'use_rviz': 'True',
        }.items()
    )

    # Set payload using ros2 service call
    set_payload = ExecuteProcess(
        cmd=[
            'ros2', 'service', 'call', '/panda_param_service_server/set_load', 'franka_msgs/srv/SetLoad',
            '"{mass: 0.3, center_of_mass: [0.0, 0.0, 0.1], load_inertia: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]}"'
        ],
        shell=True
    )

    # Add a delay to ensure bringup completes before setting the payload
    set_payload_with_delay = TimerAction(
        period=5.0,  # Wait 5 seconds for bringup to complete
        actions=[set_payload]
    )

    # Gripper node
    gripper = Node(
        package='franka_gripper_server',
        executable='franka_gripper_server_node',
        name='franka_gripper_server_node',
        output='screen',
    )

    return LaunchDescription([
        bringup,
        set_payload_with_delay,  # Delay payload setting until bringup finishes
    ])
