#!/usr/bin/env python3
"""
Two Robots with SLAM - Map Merging for Construction Progress Monitoring

This launch file:
1. Spawns two TurtleBot3 robots with NAMESPACED TF frames
2. Runs SLAM Toolbox for EACH robot (separate map frames: robot1/map, robot2/map)
3. Map merger combines both maps into /map
4. Static TF connects both map frames to 'world' for unified visualization

TF Tree Structure:
  world
    ├── robot1/map -> robot1/odom -> robot1/base_footprint -> robot1/base_scan
    └── robot2/map -> robot2/odom -> robot2/base_footprint -> robot2/base_scan

Topics:
  /robot1/map - Map from robot1's SLAM
  /robot2/map - Map from robot2's SLAM
  /map        - Merged map from both robots

Usage:
  # Complete world (for creating blueprint)
  CONSTRUCTION_WORLD=my_house_small ros2 launch construction_monitor two_robots_slam.launch.py

  # 70% incomplete world (for measuring progress)
  CONSTRUCTION_WORLD=my_house_small_70 ros2 launch construction_monitor two_robots_slam.launch.py

Available worlds:
  - my_house_small (default, 15m x 15m, complete construction)
  - my_house_small_70 (15m x 15m, 70% complete)

Then in separate terminals to start exploration:
  ros2 run construction_monitor auto_explorer_zone --ros-args -p robot_namespace:=robot1 -p zone:=left
  ros2 run construction_monitor auto_explorer_zone --ros-args -p robot_namespace:=robot2 -p zone:=right

View in RViz:
  rviz2 -d ~/ros2_ws/install/construction_monitor/share/construction_monitor/config/two_robots_slam.rviz
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction, SetEnvironmentVariable, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Get package directories
    construction_monitor_dir = get_package_share_directory('construction_monitor')
    gazebo_ros_dir = get_package_share_directory('gazebo_ros')
    turtlebot3_gazebo_dir = get_package_share_directory('turtlebot3_gazebo')

    # Custom model paths
    models_dir = os.path.join(construction_monitor_dir, 'models')
    robot1_sdf = os.path.join(models_dir, 'robot1', 'model.sdf')
    robot2_sdf = os.path.join(models_dir, 'robot2', 'model.sdf')

    # Get URDF for robot_state_publisher (static TF)
    TURTLEBOT3_MODEL = os.environ.get('TURTLEBOT3_MODEL', 'burger')
    urdf_file = os.path.join(turtlebot3_gazebo_dir, 'urdf', f'turtlebot3_{TURTLEBOT3_MODEL}.urdf')
    with open(urdf_file, 'r') as f:
        robot_desc = f.read()

    # Create URDF with namespaced frames for robot1
    robot1_urdf = robot_desc.replace('base_footprint', 'robot1/base_footprint')
    robot1_urdf = robot1_urdf.replace('base_link', 'robot1/base_link')
    robot1_urdf = robot1_urdf.replace('base_scan', 'robot1/base_scan')
    robot1_urdf = robot1_urdf.replace('wheel_left_link', 'robot1/wheel_left_link')
    robot1_urdf = robot1_urdf.replace('wheel_right_link', 'robot1/wheel_right_link')
    robot1_urdf = robot1_urdf.replace('caster_back_link', 'robot1/caster_back_link')
    robot1_urdf = robot1_urdf.replace('imu_link', 'robot1/imu_link')

    # Create URDF with namespaced frames for robot2
    robot2_urdf = robot_desc.replace('base_footprint', 'robot2/base_footprint')
    robot2_urdf = robot2_urdf.replace('base_link', 'robot2/base_link')
    robot2_urdf = robot2_urdf.replace('base_scan', 'robot2/base_scan')
    robot2_urdf = robot2_urdf.replace('wheel_left_link', 'robot2/wheel_left_link')
    robot2_urdf = robot2_urdf.replace('wheel_right_link', 'robot2/wheel_right_link')
    robot2_urdf = robot2_urdf.replace('caster_back_link', 'robot2/caster_back_link')
    robot2_urdf = robot2_urdf.replace('imu_link', 'robot2/imu_link')

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    # World selection via environment variable CONSTRUCTION_WORLD
    # Options: my_house_small (default, complete), my_house_small_70 (70% complete)
    world_name = os.environ.get('CONSTRUCTION_WORLD', 'my_house_small')

    # World configurations: world_name -> (world_file, robot1_x, robot2_x, robot_y, world_size)
    world_configs = {
        'my_house_small': ('my_house_small.world', -3.75, 3.75, -5.0, 15.0),
        'my_house_small_70': ('my_house_small_70.world', -3.75, 3.75, -5.0, 15.0),
    }

    # Get world configuration
    if world_name in world_configs:
        world_filename, robot1_x, robot2_x, robot_y, world_size = world_configs[world_name]
    else:
        print(f"Warning: Unknown world '{world_name}', using my_house_small")
        world_filename, robot1_x, robot2_x, robot_y, world_size = world_configs['my_house_small']

    world_file = os.path.join(construction_monitor_dir, 'worlds', world_filename)
    print(f"=== Loading world: {world_name} ({world_size}m x {world_size}m) ===")

    # Set Gazebo model path to include turtlebot3 models
    turtlebot3_gazebo_dir = get_package_share_directory('turtlebot3_gazebo')
    gazebo_model_path = os.path.join(turtlebot3_gazebo_dir, 'models')
    set_gazebo_model_path = SetEnvironmentVariable(
        'GAZEBO_MODEL_PATH',
        gazebo_model_path + ':' + os.environ.get('GAZEBO_MODEL_PATH', '')
    )

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

    # ========== ROBOT 1 (RED - Left Zone) ==========
    spawn_robot1 = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        name='spawn_robot1',
        output='screen',
        arguments=[
            '-entity', 'robot1',
            '-file', robot1_sdf,
            '-x', str(robot1_x),
            '-y', str(robot_y),
            '-z', '0.01',
        ]
    )

    # Robot state publisher for Robot 1 (publishes static TF to global /tf)
    # Uses namespaced frames (robot1/base_link, etc.) already defined in URDF
    robot1_state_pub = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        namespace='robot1',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'robot_description': robot1_urdf,
        }]
    )

    # SLAM Toolbox for Robot 1
    # Each robot has its own map_frame to avoid conflicts
    slam_robot1 = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        namespace='robot1',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'odom_frame': 'robot1/odom',
            'base_frame': 'robot1/base_footprint',
            'map_frame': 'robot1/map',
            'scan_topic': '/robot1/scan',
            'mode': 'mapping',
            'resolution': 0.05,
            'max_laser_range': 3.5,
            'minimum_travel_distance': 0.2,
            'minimum_travel_heading': 0.2,
            'transform_publish_period': 0.02,
            'map_update_interval': 0.5,
        }],
        remappings=[
            ('/map', '/robot1/map'),
            ('/map_metadata', '/robot1/map_metadata'),
        ]
    )

    # ========== ROBOT 2 (BLUE - Right Zone) ==========
    spawn_robot2 = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        name='spawn_robot2',
        output='screen',
        arguments=[
            '-entity', 'robot2',
            '-file', robot2_sdf,
            '-x', str(robot2_x),
            '-y', str(robot_y),
            '-z', '0.01',
        ]
    )

    # Robot state publisher for Robot 2 (publishes static TF to global /tf)
    # Uses namespaced frames (robot2/base_link, etc.) already defined in URDF
    robot2_state_pub = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        namespace='robot2',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'robot_description': robot2_urdf,
        }]
    )

    # SLAM Toolbox for Robot 2
    # Each robot has its own map_frame to avoid conflicts
    slam_robot2 = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        namespace='robot2',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'odom_frame': 'robot2/odom',
            'base_frame': 'robot2/base_footprint',
            'map_frame': 'robot2/map',
            'scan_topic': '/robot2/scan',
            'mode': 'mapping',
            'resolution': 0.05,
            'max_laser_range': 3.5,
            'minimum_travel_distance': 0.2,
            'minimum_travel_heading': 0.2,
            'transform_publish_period': 0.02,
            'map_update_interval': 0.5,
        }],
        remappings=[
            ('/map', '/robot2/map'),
            ('/map_metadata', '/robot2/map_metadata'),
        ]
    )

    # ========== MAP MERGER ==========
    # Combines /robot1/map and /robot2/map into /map
    map_merger = Node(
        package='construction_monitor',
        executable='map_merger',
        name='map_merger',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'world_size': world_size,  # Pass world size for correct % calculation
        }]
    )

    # ========== STATIC TF: Connect map frames to world ==========
    # world frame = Gazebo world origin (0,0)
    # robot1/map frame = SLAM map origin (starts at 0,0 in robot's local frame)
    #
    # The offset here should be the robot's spawn position so that:
    #   world -> robot1/map -> robot1/odom -> robot1/base_footprint
    # gives the actual Gazebo world coordinates.
    #
    # Robot1 spawns at Gazebo world (-3.75, -5.0), SLAM map starts at (0,0)
    # So world -> robot1/map offset = (-3.75, -5.0)
    world_to_robot1_map = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='world_to_robot1_map',
        arguments=[str(robot1_x), str(robot_y), '0', '0', '0', '0', 'world', 'robot1/map'],
        parameters=[{'use_sim_time': True}]
    )

    world_to_robot2_map = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='world_to_robot2_map',
        arguments=[str(robot2_x), str(robot_y), '0', '0', '0', '0', 'world', 'robot2/map'],
        parameters=[{'use_sim_time': True}]
    )

    # ========== BUILD LAUNCH DESCRIPTION ==========
    ld = LaunchDescription()

    # Set model path
    ld.add_action(set_gazebo_model_path)

    # Gazebo
    ld.add_action(gzserver_cmd)
    ld.add_action(gzclient_cmd)

    # Spawn robots
    ld.add_action(spawn_robot1)
    ld.add_action(spawn_robot2)

    # Robot state publishers (for static TF)
    ld.add_action(robot1_state_pub)
    ld.add_action(robot2_state_pub)

    # Static TF to connect map frames
    ld.add_action(world_to_robot1_map)
    ld.add_action(world_to_robot2_map)

    # Delay SLAM nodes to let robots spawn first
    delayed_slam1 = TimerAction(
        period=5.0,
        actions=[slam_robot1]
    )
    delayed_slam2 = TimerAction(
        period=6.0,
        actions=[slam_robot2]
    )

    ld.add_action(delayed_slam1)
    ld.add_action(delayed_slam2)

    # Delay map merger to let SLAM nodes start first
    delayed_map_merger = TimerAction(
        period=8.0,
        actions=[map_merger]
    )
    ld.add_action(delayed_map_merger)

    return ld
