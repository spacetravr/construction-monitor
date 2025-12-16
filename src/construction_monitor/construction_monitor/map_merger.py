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
        Merge two maps into one using proper coordinate transformation.

        Each robot's SLAM creates a map with its own origin. This function:
        1. Calculates the world-frame bounding box that contains both maps
        2. Creates a merged map in that bounding box
        3. Transforms each map's cells to the correct world position

        OccupancyGrid values:
          -1 = Unknown
           0 = Free space
         100 = Wall/Occupied

        Merge logic:
          - Wall (100) wins over everything
          - Free (0) wins over unknown (-1)
          - Unknown stays unknown
        """
        resolution = map1.info.resolution

        # Get map1 bounds in world coordinates
        map1_origin_x = map1.info.origin.position.x
        map1_origin_y = map1.info.origin.position.y
        map1_width = map1.info.width
        map1_height = map1.info.height
        map1_max_x = map1_origin_x + map1_width * resolution
        map1_max_y = map1_origin_y + map1_height * resolution

        # Get map2 bounds in world coordinates
        map2_origin_x = map2.info.origin.position.x
        map2_origin_y = map2.info.origin.position.y
        map2_width = map2.info.width
        map2_height = map2.info.height
        map2_max_x = map2_origin_x + map2_width * resolution
        map2_max_y = map2_origin_y + map2_height * resolution

        # Calculate merged map bounds (union of both maps)
        merged_origin_x = min(map1_origin_x, map2_origin_x)
        merged_origin_y = min(map1_origin_y, map2_origin_y)
        merged_max_x = max(map1_max_x, map2_max_x)
        merged_max_y = max(map1_max_y, map2_max_y)

        # Calculate merged map dimensions
        merged_width = int(np.ceil((merged_max_x - merged_origin_x) / resolution))
        merged_height = int(np.ceil((merged_max_y - merged_origin_y) / resolution))

        # Initialize merged map with unknown (-1)
        merged_data = np.full((merged_height, merged_width), -1, dtype=np.int8)

        # Convert map1 data to 2D array
        data1 = np.array(map1.data, dtype=np.int8).reshape((map1_height, map1_width))

        # Calculate offset of map1 in merged map
        offset1_x = int(round((map1_origin_x - merged_origin_x) / resolution))
        offset1_y = int(round((map1_origin_y - merged_origin_y) / resolution))

        # Copy map1 data to merged map using NumPy slicing (much faster than loops)
        # Calculate the valid region to copy
        src_y_start = max(0, -offset1_y)
        src_y_end = min(map1_height, merged_height - offset1_y)
        src_x_start = max(0, -offset1_x)
        src_x_end = min(map1_width, merged_width - offset1_x)

        dst_y_start = offset1_y + src_y_start
        dst_y_end = offset1_y + src_y_end
        dst_x_start = offset1_x + src_x_start
        dst_x_end = offset1_x + src_x_end

        if src_y_end > src_y_start and src_x_end > src_x_start:
            merged_data[dst_y_start:dst_y_end, dst_x_start:dst_x_end] = \
                data1[src_y_start:src_y_end, src_x_start:src_x_end]

        # Convert map2 data to 2D array
        data2 = np.array(map2.data, dtype=np.int8).reshape((map2_height, map2_width))

        # Calculate offset of map2 in merged map
        offset2_x = int(round((map2_origin_x - merged_origin_x) / resolution))
        offset2_y = int(round((map2_origin_y - merged_origin_y) / resolution))

        # Calculate valid region for map2
        src2_y_start = max(0, -offset2_y)
        src2_y_end = min(map2_height, merged_height - offset2_y)
        src2_x_start = max(0, -offset2_x)
        src2_x_end = min(map2_width, merged_width - offset2_x)

        dst2_y_start = offset2_y + src2_y_start
        dst2_y_end = offset2_y + src2_y_end
        dst2_x_start = offset2_x + src2_x_start
        dst2_x_end = offset2_x + src2_x_end

        # Merge map2 using vectorized NumPy operations (much faster)
        if src2_y_end > src2_y_start and src2_x_end > src2_x_start:
            region1 = merged_data[dst2_y_start:dst2_y_end, dst2_x_start:dst2_x_end]
            region2 = data2[src2_y_start:src2_y_end, src2_x_start:src2_x_end]

            # Merge logic using NumPy: wall (100) > free (0) > unknown (-1)
            # Start with current values (from map1)
            result = region1.copy()

            # Wall wins over everything
            wall_mask = (region1 == 100) | (region2 == 100)
            result[wall_mask] = 100

            # Free wins over unknown (but not over wall)
            free_mask = ~wall_mask & ((region1 == 0) | (region2 == 0))
            result[free_mask] = 0

            # Apply merged result
            merged_data[dst2_y_start:dst2_y_end, dst2_x_start:dst2_x_end] = result

        # Create merged OccupancyGrid message
        merged = OccupancyGrid()
        merged.header.stamp = self.get_clock().now().to_msg()
        merged.header.frame_id = 'world'
        merged.info.resolution = resolution
        merged.info.width = merged_width
        merged.info.height = merged_height
        merged.info.origin.position.x = merged_origin_x
        merged.info.origin.position.y = merged_origin_y
        merged.info.origin.position.z = 0.0
        merged.info.origin.orientation.w = 1.0

        # Flatten back to 1D list
        merged.data = merged_data.flatten().tolist()

        # Log stats - use FIXED expected world size for percentage
        walls = np.sum(merged_data == 100)
        free = np.sum(merged_data == 0)
        explored_cells = walls + free

        # Calculate percentage based on fixed world size (40000 cells for 10m x 10m)
        explored_percent = (explored_cells / self.expected_cells) * 100
        # Cap at 100% in case map grows beyond expected
        explored_percent = min(explored_percent, 100.0)

        self.get_logger().info(
            f'Explored: {explored_percent:.1f}% of construction site | Walls:{walls} Free:{free} | Map cells:{merged_data.size}',
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
