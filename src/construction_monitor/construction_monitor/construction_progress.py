#!/usr/bin/env python3
"""
Construction Progress Comparison Node

Compares the scanned map (from incomplete construction) with the complete blueprint
to calculate how much of the building has been constructed.

This is the REAL goal - not exploration progress, but CONSTRUCTION PROGRESS.

Usage:
  ros2 run construction_monitor construction_progress --ros-args -p blueprint_map:=/path/to/blueprint.yaml

The node:
1. Loads a complete blueprint map (from my_house world)
2. Subscribes to the live merged map /map (scanned from my_house_70)
3. Compares wall positions between blueprint and scanned map
4. Calculates: "Construction Progress: X%"
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import Float32
import numpy as np
import yaml
import os
from PIL import Image


class ConstructionProgress(Node):
    def __init__(self):
        super().__init__('construction_progress')

        # Parameters
        self.declare_parameter('blueprint_map', '')  # Path to blueprint .yaml file
        self.declare_parameter('update_interval', 5.0)  # How often to calculate progress

        blueprint_path = self.get_parameter('blueprint_map').get_parameter_value().string_value
        self.update_interval = self.get_parameter('update_interval').get_parameter_value().double_value

        # Blueprint data
        self.blueprint_walls = None  # Set of (x, y) cells that should be walls
        self.blueprint_loaded = False
        self.blueprint_info = None  # resolution, origin, etc.

        # Load blueprint if provided
        if blueprint_path:
            self.load_blueprint(blueprint_path)
        else:
            self.get_logger().warn('No blueprint_map parameter provided!')
            self.get_logger().warn('Usage: --ros-args -p blueprint_map:=/path/to/complete_map.yaml')

        # QoS for map
        map_qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE
        )

        # Subscriber for live merged map
        self.map_sub = self.create_subscription(
            OccupancyGrid,
            '/map',
            self.map_callback,
            map_qos
        )

        # Publisher for construction progress percentage
        self.progress_pub = self.create_publisher(Float32, '/construction_progress', 10)

        # Store latest map
        self.current_map = None

        # Timer to periodically calculate and publish progress
        self.timer = self.create_timer(self.update_interval, self.calculate_progress)

        self.get_logger().info('=' * 60)
        self.get_logger().info('Construction Progress Monitor Started')
        self.get_logger().info('=' * 60)
        self.get_logger().info('Comparing scanned map with blueprint to calculate')
        self.get_logger().info('how much of the building has been CONSTRUCTED.')
        self.get_logger().info('')

    def load_blueprint(self, yaml_path):
        """Load the complete blueprint map from a .yaml + .pgm file pair"""
        try:
            self.get_logger().info(f'Loading blueprint from: {yaml_path}')

            # Read YAML metadata
            with open(yaml_path, 'r') as f:
                map_metadata = yaml.safe_load(f)

            # Get image path (relative to yaml file)
            yaml_dir = os.path.dirname(yaml_path)
            image_file = map_metadata['image']
            if not os.path.isabs(image_file):
                image_path = os.path.join(yaml_dir, image_file)
            else:
                image_path = image_file

            self.get_logger().info(f'Loading blueprint image: {image_path}')

            # Read PGM image
            img = Image.open(image_path)
            img_array = np.array(img)

            # Store metadata
            self.blueprint_info = {
                'resolution': map_metadata['resolution'],
                'origin': map_metadata['origin'],
                'width': img_array.shape[1],
                'height': img_array.shape[0],
                'occupied_thresh': map_metadata.get('occupied_thresh', 0.65),
                'free_thresh': map_metadata.get('free_thresh', 0.25),
            }

            # Convert PGM to occupancy values
            # PGM: 0 = black = occupied, 255 = white = free, 205 = gray = unknown
            # OccupancyGrid: 100 = occupied (wall), 0 = free, -1 = unknown
            occupied_threshold = int((1.0 - self.blueprint_info['occupied_thresh']) * 255)

            # Find all wall cells in blueprint
            self.blueprint_walls = set()
            wall_count = 0

            for y in range(img_array.shape[0]):
                for x in range(img_array.shape[1]):
                    pixel_value = img_array[y, x]
                    # Check if this cell is a wall (occupied)
                    if pixel_value <= occupied_threshold:
                        self.blueprint_walls.add((x, y))
                        wall_count += 1

            self.blueprint_loaded = True
            self.get_logger().info(f'Blueprint loaded successfully!')
            self.get_logger().info(f'  - Size: {self.blueprint_info["width"]} x {self.blueprint_info["height"]} cells')
            self.get_logger().info(f'  - Resolution: {self.blueprint_info["resolution"]} m/cell')
            self.get_logger().info(f'  - Total blueprint walls: {wall_count} cells')
            self.get_logger().info(f'  - Origin: {self.blueprint_info["origin"]}')

        except FileNotFoundError as e:
            self.get_logger().error(f'Blueprint file not found: {e}')
        except Exception as e:
            self.get_logger().error(f'Error loading blueprint: {e}')

    def map_callback(self, msg):
        """Store the latest merged map"""
        self.current_map = msg

    def calculate_progress(self):
        """Compare current scanned map with blueprint to calculate construction progress"""
        if not self.blueprint_loaded:
            self.get_logger().warn('Blueprint not loaded - cannot calculate progress', throttle_duration_sec=10.0)
            return

        if self.current_map is None:
            self.get_logger().info('Waiting for scanned map...', throttle_duration_sec=5.0)
            return

        # Convert current map to numpy array
        map_data = np.array(self.current_map.data, dtype=np.int8)
        map_width = self.current_map.info.width
        map_height = self.current_map.info.height
        map_resolution = self.current_map.info.resolution
        map_origin_x = self.current_map.info.origin.position.x
        map_origin_y = self.current_map.info.origin.position.y

        # Blueprint info
        bp_resolution = self.blueprint_info['resolution']
        bp_origin = self.blueprint_info['origin']
        bp_width = self.blueprint_info['width']
        bp_height = self.blueprint_info['height']

        # Count matching walls
        walls_found = 0
        walls_expected = len(self.blueprint_walls)
        walls_scanned = 0  # Walls detected in scan that match blueprint position

        # For each wall in the blueprint, check if it exists in the scanned map
        for (bp_x, bp_y) in self.blueprint_walls:
            # Convert blueprint cell to world coordinates
            # Note: PGM y-axis is flipped (0 is top, but in ROS y increases upward)
            world_x = bp_origin[0] + (bp_x + 0.5) * bp_resolution
            world_y = bp_origin[1] + ((bp_height - 1 - bp_y) + 0.5) * bp_resolution

            # Convert world coordinates to scanned map cell
            scan_x = int((world_x - map_origin_x) / map_resolution)
            scan_y = int((world_y - map_origin_y) / map_resolution)

            # Check if within scanned map bounds
            if 0 <= scan_x < map_width and 0 <= scan_y < map_height:
                idx = scan_y * map_width + scan_x
                cell_value = map_data[idx]

                # Check neighboring cells too (tolerance for alignment differences)
                found = False
                for dx in range(-2, 3):
                    for dy in range(-2, 3):
                        nx, ny = scan_x + dx, scan_y + dy
                        if 0 <= nx < map_width and 0 <= ny < map_height:
                            nidx = ny * map_width + nx
                            if map_data[nidx] == 100:  # Wall detected
                                found = True
                                break
                    if found:
                        break

                if found:
                    walls_found += 1

        # Calculate construction progress
        if walls_expected > 0:
            progress_percent = (walls_found / walls_expected) * 100.0
        else:
            progress_percent = 0.0

        # Publish progress
        progress_msg = Float32()
        progress_msg.data = progress_percent
        self.progress_pub.publish(progress_msg)

        # Log progress with visual bar
        bar_length = 30
        filled = int(bar_length * progress_percent / 100)
        bar = '=' * filled + '-' * (bar_length - filled)

        self.get_logger().info('')
        self.get_logger().info('=' * 60)
        self.get_logger().info(f'  CONSTRUCTION PROGRESS: {progress_percent:.1f}%')
        self.get_logger().info(f'  [{bar}]')
        self.get_logger().info(f'  Walls detected: {walls_found} / {walls_expected} blueprint walls')
        self.get_logger().info('=' * 60)
        self.get_logger().info('')


def main(args=None):
    rclpy.init(args=args)
    node = ConstructionProgress()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
