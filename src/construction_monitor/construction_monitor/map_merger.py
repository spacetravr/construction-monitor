#!/usr/bin/env python3
"""
Map Merger Node

Combines maps from two robots into one shared map.
Auto-saves when exploration reaches threshold (default 95%).

Subscribes to:
  /robot1/map - Map from robot1 (left zone)
  /robot2/map - Map from robot2 (right zone)

Publishes:
  /map - Combined map from both robots
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import Bool
import numpy as np
import subprocess
import os
from datetime import datetime


class MapMerger(Node):
    def __init__(self):
        super().__init__('map_merger')

        # Store latest maps
        self.robot1_map = None
        self.robot2_map = None

        # World size for percentage calculation (configurable via parameter)
        self.declare_parameter('world_size', 10.0)  # Default 10m x 10m
        self.world_size_meters = self.get_parameter('world_size').get_parameter_value().double_value
        self.map_resolution = 0.05     # 5cm per cell
        self.expected_cells = int((self.world_size_meters / self.map_resolution) ** 2)
        self.get_logger().info(f'World size: {self.world_size_meters}m x {self.world_size_meters}m = {self.expected_cells} cells')

        # Auto-save settings
        self.declare_parameter('save_threshold', 95.0)  # Save when exploration reaches this %
        self.declare_parameter('save_path', os.path.expanduser('~/construction_maps'))
        self.save_threshold = self.get_parameter('save_threshold').get_parameter_value().double_value
        self.save_path = self.get_parameter('save_path').get_parameter_value().string_value
        self.map_saved = False
        self.exploration_complete = False

        # Create save directory if it doesn't exist
        os.makedirs(self.save_path, exist_ok=True)

        # QoS for map - must match RViz expectations (Transient Local durability)
        map_qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE
        )

        # Publisher for merged map
        self.merged_map_pub = self.create_publisher(OccupancyGrid, '/map', map_qos)

        # Publisher for exploration complete signal (to stop robots)
        self.exploration_complete_pub = self.create_publisher(Bool, '/exploration_complete', 10)

        # Subscribers - must use TRANSIENT_LOCAL to receive from SLAM
        self.robot1_sub = self.create_subscription(
            OccupancyGrid,
            '/robot1/map',
            self.robot1_callback,
            map_qos
        )

        self.robot2_sub = self.create_subscription(
            OccupancyGrid,
            '/robot2/map',
            self.robot2_callback,
            map_qos
        )

        # Timer to publish merged map every second
        self.timer = self.create_timer(1.0, self.merge_and_publish)

        self.get_logger().info('Map Merger started')
        self.get_logger().info(f'Auto-save threshold: {self.save_threshold}%')
        self.get_logger().info(f'Save path: {self.save_path}')
        self.get_logger().info('Waiting for /robot1/map and /robot2/map...')

    def robot1_callback(self, msg):
        self.robot1_map = msg

    def robot2_callback(self, msg):
        self.robot2_map = msg

    def merge_and_publish(self):
        # Need at least one map
        if self.robot1_map is None and self.robot2_map is None:
            return

        # If only one map available, publish that
        if self.robot1_map is None:
            self.merged_map_pub.publish(self.robot2_map)
            return
        if self.robot2_map is None:
            self.merged_map_pub.publish(self.robot1_map)
            return

        # Both maps available - merge them
        merged = self.merge_maps(self.robot1_map, self.robot2_map)
        self.merged_map_pub.publish(merged)

    def merge_maps(self, map1, map2):
        """
        Merge two maps into one.

        OccupancyGrid values:
          -1 = Unknown
           0 = Free space
         100 = Wall/Occupied

        Merge logic:
          - Wall (100) wins over everything
          - Free (0) wins over unknown (-1)
          - Unknown stays unknown
        """
        # Create merged map based on map1's structure
        merged = OccupancyGrid()
        merged.header.stamp = self.get_clock().now().to_msg()
        merged.header.frame_id = 'world'  # Use world frame for RViz visualization
        merged.info = map1.info

        # Get data as numpy arrays
        data1 = np.array(map1.data, dtype=np.int8)
        data2 = np.array(map2.data, dtype=np.int8)

        # Handle different sizes
        if len(data1) != len(data2):
            # Use larger map as base
            if len(data1) > len(data2):
                padded = np.full(len(data1), -1, dtype=np.int8)
                padded[:len(data2)] = data2
                data2 = padded
            else:
                padded = np.full(len(data2), -1, dtype=np.int8)
                padded[:len(data1)] = data1
                data1 = padded
                merged.info = map2.info

        # Merge: wall > free > unknown
        merged_data = np.full(len(data1), -1, dtype=np.int8)

        for i in range(len(data1)):
            v1, v2 = data1[i], data2[i]

            if v1 == 100 or v2 == 100:
                merged_data[i] = 100  # Wall
            elif v1 == 0 or v2 == 0:
                merged_data[i] = 0    # Free
            else:
                merged_data[i] = -1   # Unknown

        merged.data = merged_data.tolist()

        # Log stats - use FIXED expected world size for percentage
        walls = np.sum(merged_data == 100)
        free = np.sum(merged_data == 0)
        explored_cells = walls + free

        # Calculate percentage based on fixed world size (40000 cells for 10m x 10m)
        explored_percent = (explored_cells / self.expected_cells) * 100
        # Cap at 100% in case map grows beyond expected
        explored_percent = min(explored_percent, 100.0)

        self.get_logger().info(
            f'Explored: {explored_percent:.1f}% of construction site | Walls:{walls} Free:{free} | Map cells:{len(merged_data)}',
            throttle_duration_sec=5.0
        )

        # Check if exploration threshold reached
        if explored_percent >= self.save_threshold and not self.map_saved:
            self.save_map()
            self.map_saved = True
            self.exploration_complete = True

            # Publish exploration complete signal
            complete_msg = Bool()
            complete_msg.data = True
            self.exploration_complete_pub.publish(complete_msg)

        return merged

    def save_map(self):
        """Save the current map using map_saver_cli"""
        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        map_name = f'construction_map_{timestamp}'
        full_path = os.path.join(self.save_path, map_name)

        self.get_logger().info('=' * 50)
        self.get_logger().info('EXPLORATION COMPLETE!')
        self.get_logger().info(f'Saving map to: {full_path}')
        self.get_logger().info('=' * 50)

        try:
            # Use map_saver_cli to save the map
            result = subprocess.run(
                ['ros2', 'run', 'nav2_map_server', 'map_saver_cli', '-f', full_path],
                capture_output=True,
                text=True,
                timeout=30
            )

            if result.returncode == 0:
                self.get_logger().info(f'Map saved successfully!')
                self.get_logger().info(f'  - Image: {full_path}.pgm')
                self.get_logger().info(f'  - Metadata: {full_path}.yaml')
            else:
                self.get_logger().error(f'Failed to save map: {result.stderr}')

        except subprocess.TimeoutExpired:
            self.get_logger().error('Map save timed out')
        except Exception as e:
            self.get_logger().error(f'Error saving map: {str(e)}')


def main(args=None):
    rclpy.init(args=args)
    node = MapMerger()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
