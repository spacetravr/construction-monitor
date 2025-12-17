#!/usr/bin/env python3
"""
Generate Blueprint from World File

Creates a clean occupancy grid blueprint directly from Gazebo world file.
No SLAM needed - reads wall positions from the .world XML file.

Usage:
  ros2 run construction_monitor generate_blueprint --ros-args -p world_file:=/path/to/world.world -p output_path:=~/construction_maps/blueprint

This creates:
  - blueprint.pgm (image file)
  - blueprint.yaml (metadata file)
"""

import rclpy
from rclpy.node import Node
import numpy as np
import xml.etree.ElementTree as ET
import os
import yaml


class BlueprintGenerator(Node):
    def __init__(self):
        super().__init__('blueprint_generator')

        # Parameters
        self.declare_parameter('world_file', '')
        self.declare_parameter('output_path', os.path.expanduser('~/construction_maps/blueprint'))
        self.declare_parameter('resolution', 0.05)  # 5cm per cell
        self.declare_parameter('world_size', 15.0)  # 15m x 15m

        self.world_file = self.get_parameter('world_file').get_parameter_value().string_value
        self.output_path = self.get_parameter('output_path').get_parameter_value().string_value
        self.resolution = self.get_parameter('resolution').get_parameter_value().double_value
        self.world_size = self.get_parameter('world_size').get_parameter_value().double_value

        if not self.world_file:
            self.get_logger().error('No world_file parameter provided!')
            self.get_logger().error('Usage: --ros-args -p world_file:=/path/to/world.world')
            return

        self.generate_blueprint()

    def generate_blueprint(self):
        """Parse world file and generate occupancy grid"""
        self.get_logger().info(f'Reading world file: {self.world_file}')

        # Parse world file
        try:
            tree = ET.parse(self.world_file)
            root = tree.getroot()
        except Exception as e:
            self.get_logger().error(f'Failed to parse world file: {e}')
            return

        # Calculate map dimensions
        # World center is at (0,0), so map goes from -world_size/2 to +world_size/2
        half_size = self.world_size / 2
        map_size = int(self.world_size / self.resolution)

        # Origin is bottom-left corner of map in world coordinates
        origin_x = -half_size
        origin_y = -half_size

        self.get_logger().info(f'Map size: {map_size} x {map_size} cells')
        self.get_logger().info(f'Resolution: {self.resolution} m/cell')
        self.get_logger().info(f'Origin: ({origin_x}, {origin_y})')

        # Initialize map with free space (white = 254 in PGM)
        # PGM format: 0 = black (occupied), 254 = white (free), 205 = gray (unknown)
        occupancy_map = np.full((map_size, map_size), 254, dtype=np.uint8)

        # Find all models with collision geometry (walls, pillars, etc.)
        walls_found = 0
        for model in root.findall('.//model'):
            model_name = model.get('name', 'unknown')

            # Skip ground plane and other non-wall models
            if 'ground' in model_name.lower() or 'sun' in model_name.lower():
                continue

            # Get model pose
            pose_elem = model.find('pose')
            if pose_elem is not None:
                pose_parts = pose_elem.text.split()
                model_x = float(pose_parts[0])
                model_y = float(pose_parts[1])
            else:
                model_x, model_y = 0.0, 0.0

            # Find collision geometry
            for collision in model.findall('.//collision'):
                box = collision.find('.//box/size')
                if box is not None:
                    size_parts = box.text.split()
                    box_x = float(size_parts[0])  # width
                    box_y = float(size_parts[1])  # depth

                    # Draw this wall/pillar on the map
                    self.draw_box(occupancy_map, model_x, model_y, box_x, box_y,
                                  origin_x, origin_y, map_size)
                    walls_found += 1
                    self.get_logger().info(f'  Found: {model_name} at ({model_x:.2f}, {model_y:.2f}) size ({box_x:.2f} x {box_y:.2f})')

        self.get_logger().info(f'Total walls/obstacles found: {walls_found}')

        # Count occupied cells
        occupied_cells = np.sum(occupancy_map == 0)
        self.get_logger().info(f'Occupied cells (walls): {occupied_cells}')

        # Save PGM file
        self.save_pgm(occupancy_map)

        # Save YAML metadata
        self.save_yaml(origin_x, origin_y)

        self.get_logger().info('')
        self.get_logger().info('=' * 50)
        self.get_logger().info('Blueprint generated successfully!')
        self.get_logger().info(f'  PGM: {self.output_path}.pgm')
        self.get_logger().info(f'  YAML: {self.output_path}.yaml')
        self.get_logger().info(f'  Wall cells: {occupied_cells}')
        self.get_logger().info('=' * 50)

    def draw_box(self, occupancy_map, center_x, center_y, width, height,
                 origin_x, origin_y, map_size):
        """Draw a filled box on the occupancy map"""
        # Calculate box corners in world coordinates
        half_w = width / 2
        half_h = height / 2

        min_x = center_x - half_w
        max_x = center_x + half_w
        min_y = center_y - half_h
        max_y = center_y + half_h

        # Convert to map coordinates (cells)
        # Map cell (0,0) is at origin (bottom-left in world)
        cell_min_x = int((min_x - origin_x) / self.resolution)
        cell_max_x = int((max_x - origin_x) / self.resolution)
        cell_min_y = int((min_y - origin_y) / self.resolution)
        cell_max_y = int((max_y - origin_y) / self.resolution)

        # Clamp to map bounds
        cell_min_x = max(0, min(map_size - 1, cell_min_x))
        cell_max_x = max(0, min(map_size - 1, cell_max_x))
        cell_min_y = max(0, min(map_size - 1, cell_min_y))
        cell_max_y = max(0, min(map_size - 1, cell_max_y))

        # Fill cells with occupied (0 = black in PGM)
        # Note: PGM y-axis is inverted (row 0 is top)
        for y in range(cell_min_y, cell_max_y + 1):
            for x in range(cell_min_x, cell_max_x + 1):
                # Flip y for PGM format
                pgm_y = map_size - 1 - y
                occupancy_map[pgm_y, x] = 0

    def save_pgm(self, occupancy_map):
        """Save occupancy map as PGM file"""
        output_dir = os.path.dirname(self.output_path)
        if output_dir:
            os.makedirs(output_dir, exist_ok=True)

        pgm_path = f'{self.output_path}.pgm'

        # Write PGM (P5 binary format)
        height, width = occupancy_map.shape
        with open(pgm_path, 'wb') as f:
            # Header
            f.write(f'P5\n{width} {height}\n255\n'.encode())
            # Data
            f.write(occupancy_map.tobytes())

        self.get_logger().info(f'Saved PGM: {pgm_path}')

    def save_yaml(self, origin_x, origin_y):
        """Save YAML metadata file"""
        yaml_path = f'{self.output_path}.yaml'
        pgm_filename = os.path.basename(f'{self.output_path}.pgm')

        metadata = {
            'image': pgm_filename,
            'resolution': self.resolution,
            'origin': [origin_x, origin_y, 0.0],
            'negate': 0,
            'occupied_thresh': 0.65,
            'free_thresh': 0.25,
        }

        with open(yaml_path, 'w') as f:
            yaml.dump(metadata, f, default_flow_style=False)

        self.get_logger().info(f'Saved YAML: {yaml_path}')


def main(args=None):
    rclpy.init(args=args)
    node = BlueprintGenerator()
    # No spinning needed - just generate and exit
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
