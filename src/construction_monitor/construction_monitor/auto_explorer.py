#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
import random
import math

class AutoExplorer(Node):
    def __init__(self):
        super().__init__('auto_explorer')

        # Publisher for robot movement
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # Subscriber for laser scan data
        self.scan_sub = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10
        )

        # Robot state
        self.state = 'FORWARD'  # States: FORWARD, TURN_LEFT, TURN_RIGHT (REMOVED BACKUP)
        self.obstacle_distance = 1.8  # meters - React EARLY at 1.8m for maximum safety
        self.min_front_distance = float('inf')

        # Movement parameters
        self.linear_speed = 0.15  # m/s - Faster for quicker exploration
        self.angular_speed = 1.0  # rad/s - Very fast turning for efficiency

        # Timer for state changes
        self.turn_duration = 0
        self.turn_start_time = None

        # Random exploration to cover unmapped areas
        self.forward_counter = 0  # Count how long we've been going straight
        self.max_forward_time = 50  # Turn after ~5 seconds of straight movement

        self.get_logger().info('Auto Explorer Node Started!')
        self.get_logger().info(f'Obstacle avoidance distance: {self.obstacle_distance}m')

    def scan_callback(self, msg):
        """Process laser scan data and make movement decisions"""

        # Get the minimum distance in front of the robot
        # LiDAR scan: index 0 = RIGHT, index 90 = BACK, index 180 = LEFT, index 270 = FRONT
        # We want to check FRONT of robot (around index 270-360 and 0-90)
        ranges = msg.ranges
        num_ranges = len(ranges)

        # FRONT sector: -45 to +45 degrees from forward direction
        # For 360 ranges: front is around index 0 (straight ahead)
        # Check indices 315-360 and 0-45 (90 degrees total centered at 0)
        front_right = ranges[315:360]  # -45 to 0 degrees
        front_left = ranges[0:45]      # 0 to +45 degrees
        front_ranges = list(front_right) + list(front_left)

        # Debug: Log scan range info (only once at start)
        if not hasattr(self, '_logged_scan_info'):
            self.get_logger().info(f'LiDAR total ranges: {num_ranges}, front sector: 315-360 and 0-45')
            self._logged_scan_info = True

        # Filter out inf and nan values
        valid_front_ranges = [r for r in front_ranges if not math.isinf(r) and not math.isnan(r) and r > 0.0]

        if valid_front_ranges:
            self.min_front_distance = min(valid_front_ranges)
            # Debug: Log distance every 10 callbacks (~1 second at 10Hz) for better visibility
            if not hasattr(self, '_callback_count'):
                self._callback_count = 0
            self._callback_count += 1
            if self._callback_count % 10 == 0:
                self.get_logger().info(f'Min front distance: {self.min_front_distance:.2f}m (threshold: {self.obstacle_distance}m)')
        else:
            self.min_front_distance = float('inf')

        # Check LEFT and RIGHT sides for decision making
        # LEFT: indices 45-135 (90 degrees on left side)
        # RIGHT: indices 225-315 (90 degrees on right side)
        left_ranges = ranges[45:135]
        right_ranges = ranges[225:315]

        valid_left_ranges = [r for r in left_ranges if not math.isinf(r) and not math.isnan(r) and r > 0.0]
        valid_right_ranges = [r for r in right_ranges if not math.isinf(r) and not math.isnan(r) and r > 0.0]

        avg_left_dist = sum(valid_left_ranges) / len(valid_left_ranges) if valid_left_ranges else float('inf')
        avg_right_dist = sum(valid_right_ranges) / len(valid_right_ranges) if valid_right_ranges else float('inf')

        # Make movement decision
        cmd = Twist()

        if self.state == 'FORWARD':
            if self.min_front_distance < self.obstacle_distance:
                # Obstacle ahead - START TURNING IMMEDIATELY (NO BACKUP)

                # Check if robot is in a CORNER (walls on multiple sides)
                corner_threshold = 0.8  # If both sides < 0.8m, it's a corner
                is_in_corner = (avg_left_dist < corner_threshold and avg_right_dist < corner_threshold)

                if is_in_corner:
                    # CORNER DETECTED - do full 180 turn to escape
                    self.state = 'TURN_LEFT'  # Always turn left for 180
                    self.turn_duration = 3.14  # Turn for ~180 degrees (1.0 rad/s * 3.14s = π rad)
                    self.get_logger().info(f'CORNER DETECTED! (L:{avg_left_dist:.2f}m R:{avg_right_dist:.2f}m F:{self.min_front_distance:.2f}m) - 180° turn')
                else:
                    # Normal obstacle - choose better direction and turn SMALL angle (60 degrees)
                    if avg_left_dist > avg_right_dist:
                        self.state = 'TURN_LEFT'
                        self.get_logger().info(f'Obstacle at {self.min_front_distance:.2f}m (L:{avg_left_dist:.2f}m > R:{avg_right_dist:.2f}m) - Turning LEFT')
                    else:
                        self.state = 'TURN_RIGHT'
                        self.get_logger().info(f'Obstacle at {self.min_front_distance:.2f}m (R:{avg_right_dist:.2f}m > L:{avg_left_dist:.2f}m) - Turning RIGHT')

                    # Set turn duration for SMALL 60 degree turn (more accurate mapping)
                    self.turn_duration = random.uniform(0.9, 1.1)  # ~60 degrees (1.0 rad/s * 1.0s ≈ 60°)

                self.turn_start_time = self.get_clock().now()
                self.forward_counter = 0  # Reset forward counter after turning
            else:
                # Path is clear - move forward
                cmd.linear.x = self.linear_speed
                cmd.angular.z = 0.0

                # Random exploration: turn slightly every ~5 seconds to explore new areas
                self.forward_counter += 1
                if self.forward_counter >= self.max_forward_time:
                    # Randomly turn left or right to explore unmapped areas
                    self.state = random.choice(['TURN_LEFT', 'TURN_RIGHT'])
                    self.turn_duration = random.uniform(0.4, 0.6)  # Small 30° turn
                    self.turn_start_time = self.get_clock().now()
                    self.forward_counter = 0
                    self.get_logger().info(f'Random exploration turn - exploring new area')

        elif self.state == 'TURN_LEFT':
            cmd.linear.x = 0.0
            cmd.angular.z = self.angular_speed

            # Use actual time instead of counter
            elapsed = (self.get_clock().now() - self.turn_start_time).nanoseconds / 1e9

            if elapsed >= self.turn_duration:
                self.state = 'FORWARD'
                self.turn_duration = 0  # Reset for next obstacle
                self.get_logger().info('Turn complete - Moving FORWARD')

        elif self.state == 'TURN_RIGHT':
            cmd.linear.x = 0.0
            cmd.angular.z = -self.angular_speed

            # Use actual time instead of counter
            elapsed = (self.get_clock().now() - self.turn_start_time).nanoseconds / 1e9

            if elapsed >= self.turn_duration:
                self.state = 'FORWARD'
                self.turn_duration = 0  # Reset for next obstacle
                self.get_logger().info('Turn complete - Moving FORWARD')

        # Publish movement command
        self.cmd_vel_pub.publish(cmd)

def main(args=None):
    rclpy.init(args=args)
    explorer = AutoExplorer()

    try:
        rclpy.spin(explorer)
    except KeyboardInterrupt:
        pass
    finally:
        # Stop the robot
        stop_cmd = Twist()
        explorer.cmd_vel_pub.publish(stop_cmd)
        explorer.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
