#!/usr/bin/env python3
"""
Auto Explorer with Namespace Support for Multi-Robot SLAM

Usage:
  ros2 run construction_monitor auto_explorer_ns --ros-args -p robot_namespace:=robot1
  ros2 run construction_monitor auto_explorer_ns --ros-args -p robot_namespace:=robot2
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid
import random
import math
import numpy as np


class AutoExplorerNS(Node):
    def __init__(self):
        super().__init__('auto_explorer_ns')

        # Declare and get namespace parameter
        self.declare_parameter('robot_namespace', '')
        self.robot_ns = self.get_parameter('robot_namespace').get_parameter_value().string_value

        # Build topic names based on namespace
        if self.robot_ns:
            cmd_vel_topic = f'/{self.robot_ns}/cmd_vel'
            scan_topic = f'/{self.robot_ns}/scan'
            self.get_logger().info(f'Starting explorer for namespace: {self.robot_ns}')
        else:
            cmd_vel_topic = '/cmd_vel'
            scan_topic = '/scan'
            self.get_logger().info('Starting explorer WITHOUT namespace (single robot mode)')

        # Publisher for robot movement
        self.cmd_vel_pub = self.create_publisher(Twist, cmd_vel_topic, 10)

        # Subscriber for laser scan data
        self.scan_sub = self.create_subscription(
            LaserScan,
            scan_topic,
            self.scan_callback,
            10
        )

        # Robot state
        self.state = 'FORWARD'
        self.obstacle_distance = 1.0
        self.min_front_distance = float('inf')

        # Movement parameters
        self.linear_speed = 0.12
        self.angular_speed = 1.0

        # Timer for state changes
        self.turn_duration = 0
        self.turn_start_time = None

        # Exploration
        self.forward_counter = 0
        self.max_forward_time = 150

        self.get_logger().info(f'Topics: cmd_vel={cmd_vel_topic}, scan={scan_topic}')
        self.get_logger().info(f'Obstacle avoidance distance: {self.obstacle_distance}m')

    def scan_callback(self, msg):
        """Process laser scan data and make movement decisions"""
        ranges = msg.ranges
        num_ranges = len(ranges)

        # FRONT sector: indices 315-360 and 0-45
        front_right = ranges[315:360]
        front_left = ranges[0:45]
        front_ranges = list(front_right) + list(front_left)

        # Filter out inf and nan values
        valid_front_ranges = [r for r in front_ranges if not math.isinf(r) and not math.isnan(r) and r > 0.0]

        if valid_front_ranges:
            self.min_front_distance = min(valid_front_ranges)
        else:
            self.min_front_distance = float('inf')

        # LEFT and RIGHT sides
        left_ranges = ranges[45:135]
        right_ranges = ranges[225:315]

        valid_left_ranges = [r for r in left_ranges if not math.isinf(r) and not math.isnan(r) and r > 0.0]
        valid_right_ranges = [r for r in right_ranges if not math.isinf(r) and not math.isnan(r) and r > 0.0]

        avg_left_dist = sum(valid_left_ranges) / len(valid_left_ranges) if valid_left_ranges else float('inf')
        avg_right_dist = sum(valid_right_ranges) / len(valid_right_ranges) if valid_right_ranges else float('inf')

        # Make movement decision
        cmd = Twist()

        # EMERGENCY STOP
        if self.min_front_distance < 0.3:
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0
            self.cmd_vel_pub.publish(cmd)
            self.get_logger().warn(f'[{self.robot_ns}] EMERGENCY STOP! Wall at {self.min_front_distance:.2f}m')
            self.state = 'TURN_LEFT'
            self.turn_duration = 3.14
            self.turn_start_time = self.get_clock().now()
            return

        if self.state == 'FORWARD':
            if self.min_front_distance < self.obstacle_distance:
                # Obstacle detected
                corner_threshold = 0.6
                is_in_corner = (avg_left_dist < corner_threshold and avg_right_dist < corner_threshold)

                if is_in_corner:
                    self.state = 'TURN_LEFT'
                    self.turn_duration = 3.14
                    self.get_logger().info(f'[{self.robot_ns}] CORNER! - 180° turn')
                else:
                    if avg_left_dist > avg_right_dist:
                        self.state = 'TURN_LEFT'
                        self.get_logger().info(f'[{self.robot_ns}] Obstacle - Turn LEFT')
                    else:
                        self.state = 'TURN_RIGHT'
                        self.get_logger().info(f'[{self.robot_ns}] Obstacle - Turn RIGHT')
                    self.turn_duration = random.uniform(0.7, 0.8)

                self.turn_start_time = self.get_clock().now()
                self.forward_counter = 0
            else:
                # Path is clear - move forward
                cmd.linear.x = self.linear_speed
                cmd.angular.z = 0.0

                # Random exploration turn
                self.forward_counter += 1
                if self.forward_counter >= self.max_forward_time:
                    self.state = random.choice(['TURN_LEFT', 'TURN_RIGHT'])
                    self.turn_duration = random.uniform(0.25, 0.35)
                    self.turn_start_time = self.get_clock().now()
                    self.forward_counter = 0
                    self.get_logger().info(f'[{self.robot_ns}] Exploration turn')

        elif self.state == 'TURN_LEFT':
            cmd.linear.x = 0.0
            cmd.angular.z = self.angular_speed
            elapsed = (self.get_clock().now() - self.turn_start_time).nanoseconds / 1e9
            if elapsed >= self.turn_duration:
                self.state = 'FORWARD'
                self.turn_duration = 0

        elif self.state == 'TURN_RIGHT':
            cmd.linear.x = 0.0
            cmd.angular.z = -self.angular_speed
            elapsed = (self.get_clock().now() - self.turn_start_time).nanoseconds / 1e9
            if elapsed >= self.turn_duration:
                self.state = 'FORWARD'
                self.turn_duration = 0

        self.cmd_vel_pub.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    explorer = AutoExplorerNS()

    try:
        rclpy.spin(explorer)
    except KeyboardInterrupt:
        pass
    finally:
        stop_cmd = Twist()
        explorer.cmd_vel_pub.publish(stop_cmd)
        explorer.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
