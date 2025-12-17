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
        self.declare_parameter('slam_noise_factor', 1.33)  # SLAM noise correction factor

        blueprint_path = self.get_parameter('blueprint_map').get_parameter_value().string_value
        self.update_interval = self.get_parameter('update_interval').get_parameter_value().double_value
        self.slam_noise_factor = self.get_parameter('slam_noise_factor').get_parameter_value().double_value

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
        """Compare current scanned map with blueprint to calculate construction progress

        Uses a simple wall cell count ratio approach:
        - Counts wall cells in scanned map
        - Counts wall cells in blueprint
        - Progress = scanned_walls / blueprint_walls * 100%

        This works because both maps cover the same physical area (15m x 15m),
        so if 70% of walls are built, we should see ~70% of wall cells.

        Note: Progress will increase as exploration continues, and stabilize
        once the site is fully scanned.
        """
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

        # Count walls and free space in scanned map
        map_2d = map_data.reshape((map_height, map_width))
        scanned_walls = np.sum(map_2d == 100)
        scanned_free = np.sum(map_2d == 0)
        explored_cells = scanned_walls + scanned_free

        if scanned_walls < 10:
            self.get_logger().info('Not enough walls scanned yet...', throttle_duration_sec=5.0)
            return

        # Blueprint wall count
        blueprint_walls = len(self.blueprint_walls)

        # Calculate exploration progress (how much of the site has been scanned)
        # 15m x 15m world at 0.05m resolution = 300x300 = 90000 cells
        world_cells = 90000  # For 15m world
        exploration_percent = min((explored_cells / world_cells) * 100.0, 100.0)

        # Progress calculation with SLAM noise correction
        # SLAM adds extra wall cells due to sensor noise, wall thickening, and artifacts
        # We divide by slam_noise_factor to compensate (default 1.19 = 19% inflation)
        #
        # Example: If 70% world scan produces 4500 cells, blueprint has 4527 cells:
        #   Raw: 4500/4527 = 99% (wrong!)
        #   Corrected: 4500/(4527*1.19) = 83% (closer to true 80.8%)
        adjusted_blueprint = blueprint_walls * self.slam_noise_factor
        progress_percent = (scanned_walls / adjusted_blueprint) * 100.0

        # Cap at 100%
        progress_percent = min(progress_percent, 100.0)

        # Publish progress
        progress_msg = Float32()
        progress_msg.data = progress_percent
        self.progress_pub.publish(progress_msg)

        # Log progress with visual bar
        bar_length = 30
        filled = int(bar_length * progress_percent / 100)
        bar = '=' * filled + '-' * (bar_length - filled)

        # Exploration bar
        exp_filled = int(bar_length * exploration_percent / 100)
        exp_bar = '=' * exp_filled + '-' * (bar_length - exp_filled)

        self.get_logger().info('')
        self.get_logger().info('=' * 60)
        self.get_logger().info(f'  EXPLORATION: {exploration_percent:.1f}%  [{exp_bar}]')
        self.get_logger().info(f'  CONSTRUCTION: {progress_percent:.1f}%  [{bar}]')
        self.get_logger().info(f'  Walls: {scanned_walls} scanned / {blueprint_walls} blueprint')
        if exploration_percent < 80:
            self.get_logger().info(f'  (Construction % will stabilize after exploration completes)')
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
