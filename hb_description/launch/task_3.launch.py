#!/usr/bin/env python3
"""
Task 3 Launch File for Holo Battalion Theme.
This launches the Gazebo simulation with the Task 3 robot (with arm/gripper) 
and the Task 3 world environment.
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import AppendEnvironmentVariable
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.actions import TimerAction
import xacro


def generate_launch_description():
    ros_gz_sim = get_package_share_directory('ros_gz_sim')
    launch_file_dir = os.path.join(get_package_share_directory('hb_description'), 'launch')
    pkg_holo_share = get_package_share_directory('hb_description')

    # Task 3 World file
    world = os.path.join(
        get_package_share_directory('hb_description'),
        'worlds',
        'task_3.world'
    )

    gui_config = os.path.join(
        get_package_share_directory('hb_description'),
        'config',
        'gui.config'
    )

    # Task 3 Controllers configuration
    controllers_file = os.path.join(pkg_holo_share, 'config', 'controllers_task3.yaml')

    set_env_vars_resources = AppendEnvironmentVariable(
        'GZ_SIM_RESOURCE_PATH',
        os.path.join(get_package_share_directory('hb_description'), 'models'))
    
    # Launch Gazebo Server
    gzserver_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': ['-r -s -v4 ', world], 'on_exit_shutdown': 'true'}.items()
    )
    
    # Launch Gazebo Client (GUI)
    gzclient_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={
            'gz_args': f'-g --gui-config {gui_config} -v4'
        }.items()
    )
    
    # Gazebo-ROS Bridge for clock and camera
    gz_sim_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
            "/camera_sensor@sensor_msgs/msg/Image[gz.msgs.Image",
            "/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo"
        ],
        remappings=[
            ("/camera_sensor", "/camera/image_raw"),
            ("/camera_info", "/camera/camera_info"),
        ],
        output="screen",
    )

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    
    # Robot State Publisher with Task 3 model
    robot_state_publisher_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_file_dir, 'rsp_task3.launch.py')
        ),
        launch_arguments={'use_sim_time': use_sim_time}.items()
    )

    # Spawn the robot in Gazebo
    start_gazebo_ros_spawner_cmd = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', 'hb_task3_bot', 
            '-topic', 'robot_description', 
            '-x', '0.5', '-y', '0.5', '-z', '0.1',
            '-R', "0.0", '-P', "0.0", '-Y', "-1.5708"
        ],
        output='screen',
    )

    # Spawn wheel velocity controller
    spawn_velocity_controller = Node(
        package='controller_manager', 
        executable='spawner', 
        output='screen',
        arguments=['forward_velocity_controller']
    )

    # Spawn arm position controller
    spawn_arm_controller = Node(
        package='controller_manager', 
        executable='spawner', 
        output='screen',
        arguments=['arm_position_controller']
    )

    # Spawn gripper position controller
    spawn_gripper_controller = Node(
        package='controller_manager', 
        executable='spawner', 
        output='screen',
        arguments=['gripper_position_controller']
    )

    # Spawn joint state broadcaster
    spawn_joint_state_broadcaster = Node(
        package='controller_manager', 
        executable='spawner', 
        output='screen',
        arguments=['joint_state_broadcaster']
    )

    ld = LaunchDescription()
    ld.add_action(set_env_vars_resources)
    ld.add_action(gzserver_cmd)
    ld.add_action(gzclient_cmd)
    ld.add_action(gz_sim_bridge)
    ld.add_action(robot_state_publisher_cmd)
    ld.add_action(start_gazebo_ros_spawner_cmd)
    
    # Add controller spawners with delay to ensure robot is loaded
    ld.add_action(TimerAction(
        period=3.0,
        actions=[
            spawn_velocity_controller,
            spawn_arm_controller,
            spawn_gripper_controller,
            spawn_joint_state_broadcaster
        ]
    ))
    
    return ld
