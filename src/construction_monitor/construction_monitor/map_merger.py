#!/usr/bin/env python3
"""
Map Merger Node

Combines maps from two robots into one shared map.

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
import numpy as np


class MapMerger(Node):
    def __init__(self):
        super().__init__('map_merger')

        # Store latest maps
        self.robot1_map = None
        self.robot2_map = None

        # QoS for map - must match RViz expectations (Transient Local durability)
        map_qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE
        )

        # Publisher for merged map
        self.merged_map_pub = self.create_publisher(OccupancyGrid, '/map', map_qos)

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

        # Log stats
        total = len(merged_data)
        walls = np.sum(merged_data == 100)
        free = np.sum(merged_data == 0)
        unknown = np.sum(merged_data == -1)
        explored = ((total - unknown) / total) * 100

        self.get_logger().info(
            f'Merged: {explored:.1f}% explored | Walls:{walls} Free:{free} Unknown:{unknown}',
            throttle_duration_sec=5.0
        )

        return merged


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
