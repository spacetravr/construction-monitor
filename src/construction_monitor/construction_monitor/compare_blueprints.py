#!/usr/bin/env python3
"""
Compare Blueprints - Direct Construction Progress

Compares two clean blueprint maps (generated from world files) to calculate
the true construction progress without SLAM noise.

Usage:
  ros2 run construction_monitor compare_blueprints --ros-args \
    -p complete_blueprint:=~/construction_maps/blueprint_clean.yaml \
    -p incomplete_blueprint:=~/construction_maps/blueprint_70_clean.yaml
"""

import rclpy
from rclpy.node import Node
import numpy as np
import yaml
import os
from PIL import Image


class BlueprintComparator(Node):
    def __init__(self):
        super().__init__('blueprint_comparator')

        # Parameters
        self.declare_parameter('complete_blueprint', '')
        self.declare_parameter('incomplete_blueprint', '')

        complete_path = self.get_parameter('complete_blueprint').get_parameter_value().string_value
        incomplete_path = self.get_parameter('incomplete_blueprint').get_parameter_value().string_value

        # Expand paths
        complete_path = os.path.expanduser(complete_path)
        incomplete_path = os.path.expanduser(incomplete_path)

        if not complete_path or not incomplete_path:
            self.get_logger().error('Both blueprint paths required!')
            self.get_logger().error('Usage: --ros-args -p complete_blueprint:=/path/to/complete.yaml -p incomplete_blueprint:=/path/to/incomplete.yaml')
            return

        # Load and compare
        complete_walls = self.count_walls(complete_path, "Complete (100%)")
        incomplete_walls = self.count_walls(incomplete_path, "Incomplete (70%)")

        if complete_walls > 0 and incomplete_walls > 0:
            progress = (incomplete_walls / complete_walls) * 100.0

            self.get_logger().info('')
            self.get_logger().info('=' * 60)
            self.get_logger().info('  CONSTRUCTION PROGRESS COMPARISON')
            self.get_logger().info('=' * 60)
            self.get_logger().info(f'  Complete blueprint walls:   {complete_walls} cells')
            self.get_logger().info(f'  Incomplete blueprint walls: {incomplete_walls} cells')
            self.get_logger().info('')

            # Progress bar
            bar_length = 40
            filled = int(bar_length * progress / 100)
            bar = '=' * filled + '-' * (bar_length - filled)

            self.get_logger().info(f'  CONSTRUCTION PROGRESS: {progress:.1f}%')
            self.get_logger().info(f'  [{bar}]')
            self.get_logger().info('')
            self.get_logger().info('  This is the TRUE construction progress based on')
            self.get_logger().info('  actual wall definitions in the world files.')
            self.get_logger().info('=' * 60)

    def count_walls(self, yaml_path, label):
        """Load a blueprint and count wall cells"""
        try:
            self.get_logger().info(f'Loading {label}: {yaml_path}')

            # Read YAML metadata
            with open(yaml_path, 'r') as f:
                metadata = yaml.safe_load(f)

            # Get image path
            yaml_dir = os.path.dirname(yaml_path)
            image_file = metadata['image']
            if not os.path.isabs(image_file):
                image_path = os.path.join(yaml_dir, image_file)
            else:
                image_path = image_file

            # Read PGM
            img = Image.open(image_path)
            img_array = np.array(img)

            # Count wall cells (black pixels, value 0)
            # In PGM: 0 = occupied (wall), 254 = free
            wall_cells = np.sum(img_array == 0)

            self.get_logger().info(f'  Size: {img_array.shape[1]} x {img_array.shape[0]} cells')
            self.get_logger().info(f'  Wall cells: {wall_cells}')

            return wall_cells

        except Exception as e:
            self.get_logger().error(f'Error loading {yaml_path}: {e}')
            return 0


def main(args=None):
    rclpy.init(args=args)
    node = BlueprintComparator()
    # No spinning needed - just compare and exit
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
