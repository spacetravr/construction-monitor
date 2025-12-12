#!/usr/bin/env python3
"""
Step 7.6: Launch file to spawn TWO TurtleBot3 robots with namespaces.

Robot1 topics: /robot1/cmd_vel, /robot1/scan, /robot1/odom
Robot2 topics: /robot2/cmd_vel, /robot2/scan, /robot2/odom

Robot1 spawns at (-2, -3) - bottom left
Robot2 spawns at (2, -3) - bottom right
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Get package directories
    construction_monitor_dir = get_package_share_directory('construction_monitor')
    gazebo_ros_dir = get_package_share_directory('gazebo_ros')
    turtlebot3_gazebo_dir = get_package_share_directory('turtlebot3_gazebo')

    # Get TurtleBot3 model from environment
    TURTLEBOT3_MODEL = os.environ.get('TURTLEBOT3_MODEL', 'burger')

    # Path to robot URDF
    urdf_file_name = 'turtlebot3_' + TURTLEBOT3_MODEL + '.urdf'
    urdf_path = os.path.join(turtlebot3_gazebo_dir, 'urdf', urdf_file_name)

    # Read URDF file
    with open(urdf_path, 'r') as urdf_file:
        robot_description = urdf_file.read()

    # Path to robot SDF for Gazebo spawning
    model_folder = 'turtlebot3_' + TURTLEBOT3_MODEL
    sdf_path = os.path.join(turtlebot3_gazebo_dir, 'models', model_folder, 'model.sdf')

    # World file
    world_file = os.path.join(construction_monitor_dir, 'worlds', 'construction_incomplete.world')

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    # ========== GAZEBO SERVER & CLIENT ==========
    gzserver_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_ros_dir, 'launch', 'gzserver.launch.py')
        ),
        launch_arguments={'world': world_file}.items()
    )

    gzclient_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_ros_dir, 'launch', 'gzclient.launch.py')
        )
    )

    # ========== ROBOT 1 (with namespace) ==========
    # Spawn robot1 in Gazebo at position (-2, -3) - BOTTOM LEFT
    spawn_robot1 = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        name='spawn_robot1',
        output='screen',
        arguments=[
            '-entity', 'robot1',
            '-file', sdf_path,
            '-x', '-2.0',
            '-y', '-3.0',
            '-z', '0.01',
            '-robot_namespace', 'robot1'
        ]
    )

    # Robot state publisher for robot1
    robot1_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        namespace='robot1',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'robot_description': robot_description,
            'frame_prefix': 'robot1/'
        }]
    )

    # ========== ROBOT 2 (with namespace) ==========
    # Spawn robot2 in Gazebo at position (2, -3) - BOTTOM RIGHT
    spawn_robot2 = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        name='spawn_robot2',
        output='screen',
        arguments=[
            '-entity', 'robot2',
            '-file', sdf_path,
            '-x', '2.0',
            '-y', '-3.0',
            '-z', '0.01',
            '-robot_namespace', 'robot2'
        ]
    )

    # Robot state publisher for robot2
    robot2_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        namespace='robot2',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'robot_description': robot_description,
            'frame_prefix': 'robot2/'
        }]
    )

    # ========== BUILD LAUNCH DESCRIPTION ==========
    ld = LaunchDescription()

    # Gazebo
    ld.add_action(gzserver_cmd)
    ld.add_action(gzclient_cmd)

    # Robot 1
    ld.add_action(spawn_robot1)
    ld.add_action(robot1_state_publisher)

    # Robot 2
    ld.add_action(spawn_robot2)
    ld.add_action(robot2_state_publisher)

    return ld
