#!/usr/bin/env python3
"""
Auto Explorer with Zone Support for Multi-Robot SLAM

Each robot is assigned a zone (left or right) and stays within it.
This prevents overlap and makes exploration more efficient.

Usage:
  ros2 run construction_monitor auto_explorer_zone --ros-args -p robot_namespace:=robot1 -p zone:=left
  ros2 run construction_monitor auto_explorer_zone --ros-args -p robot_namespace:=robot2 -p zone:=right

Zones:
  - left: robot stays in x < 0 area
  - right: robot stays in x > 0 area
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
import math
import random


class AutoExplorerZone(Node):
    def __init__(self):
        super().__init__('auto_explorer_zone')

        # Declare parameters
        self.declare_parameter('robot_namespace', '')
        self.declare_parameter('zone', '')  # 'left' or 'right'

        # Get parameters
        self.robot_ns = self.get_parameter('robot_namespace').get_parameter_value().string_value
        self.zone = self.get_parameter('zone').get_parameter_value().string_value

        # Validate zone parameter
        if self.zone not in ['left', 'right']:
            self.get_logger().error(f'Invalid zone: {self.zone}. Must be "left" or "right"')
            self.get_logger().error('Usage: --ros-args -p zone:=left  OR  -p zone:=right')
            raise ValueError(f'Invalid zone parameter: {self.zone}')

        # Build topic names based on namespace
        if self.robot_ns:
            cmd_vel_topic = f'/{self.robot_ns}/cmd_vel'
            scan_topic = f'/{self.robot_ns}/scan'
            odom_topic = f'/{self.robot_ns}/odom'
            self.get_logger().info(f'Starting zone explorer for namespace: {self.robot_ns}')
        else:
            cmd_vel_topic = '/cmd_vel'
            scan_topic = '/scan'
            odom_topic = '/odom'
            self.get_logger().info('Starting zone explorer WITHOUT namespace (single robot mode)')

        self.get_logger().info(f'=== ZONE: {self.zone.upper()} ===')
        if self.zone == 'left':
            self.get_logger().info('Robot will stay in x < 0 area')
        else:
            self.get_logger().info('Robot will stay in x > 0 area')

        # Publisher for robot movement
        self.cmd_vel_pub = self.create_publisher(Twist, cmd_vel_topic, 10)

        # Subscriber for laser scan data
        self.scan_sub = self.create_subscription(
            LaserScan,
            scan_topic,
            self.scan_callback,
            10
        )

        # Subscriber for odometry (to track position)
        self.odom_sub = self.create_subscription(
            Odometry,
            odom_topic,
            self.odom_callback,
            10
        )

        # Robot position
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_yaw = 0.0

        # Robot state
        self.state = 'FORWARD'
        self.obstacle_distance = 1.0  # Back to 1.0m for better detection
        self.min_front_distance = float('inf')

        # Movement parameters
        self.linear_speed = 0.15  # Moderate speed (safer for detection)
        self.angular_speed = 1.0  # Normal turning speed

        # Timer for state changes
        self.turn_duration = 0
        self.turn_start_time = None

        # Exploration
        self.forward_counter = 0
        self.max_forward_time = 150

        # Zone boundary
        self.zone_boundary = 0.0  # x = 0 is the boundary
        self.zone_margin = 0.5   # Start turning back when within 0.5m of boundary
        self.returning_to_zone = False
        self.boundary_turn_cooldown = 0  # Cooldown counter after boundary turn

        self.get_logger().info(f'Topics: cmd_vel={cmd_vel_topic}, scan={scan_topic}, odom={odom_topic}')
        self.get_logger().info(f'Obstacle avoidance distance: {self.obstacle_distance}m')

    def odom_callback(self, msg):
        """Track robot position from odometry"""
        self.robot_x = msg.pose.pose.position.x
        self.robot_y = msg.pose.pose.position.y

        # Extract yaw from quaternion
        q = msg.pose.pose.orientation
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        self.robot_yaw = math.atan2(siny_cosp, cosy_cosp)

    def is_in_wrong_zone(self):
        """Check if robot has crossed into the wrong zone"""
        if self.zone == 'left':
            # Left zone robot should stay in x < 0
            return self.robot_x > self.zone_boundary + self.zone_margin
        else:
            # Right zone robot should stay in x > 0
            return self.robot_x < self.zone_boundary - self.zone_margin

    def is_near_boundary(self):
        """Check if robot is near the zone boundary - simple position-based check"""
        # Simple and reliable: just check distance to boundary
        # Use larger margin (1.0m) to catch diagonal approaches

        margin = 1.0  # Always use 1.0m margin - catches all angles

        if self.zone == 'left':
            # Left zone robot - boundary is at x=0
            # Trigger when x > -1.0 (within 1m of boundary)
            return self.robot_x > self.zone_boundary - margin
        else:
            # Right zone robot - boundary is at x=0
            # Trigger when x < 1.0 (within 1m of boundary)
            return self.robot_x < self.zone_boundary + margin

    def get_turn_direction_to_zone(self):
        """Determine which way to turn to get back to assigned zone"""
        if self.zone == 'left':
            # Need to go back to negative x (turn to face left)
            # If facing right (yaw near 0), turn left (positive angular)
            if -math.pi/2 < self.robot_yaw < math.pi/2:
                return 'TURN_LEFT'
            else:
                return 'TURN_RIGHT'
        else:
            # Need to go back to positive x (turn to face right)
            # If facing left (yaw near pi or -pi), turn right
            if self.robot_yaw > math.pi/2 or self.robot_yaw < -math.pi/2:
                return 'TURN_LEFT'
            else:
                return 'TURN_RIGHT'

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

        # ZONE CHECK - Priority 1: Stay in assigned zone
        if self.is_in_wrong_zone():
            if not self.returning_to_zone:
                # First time detecting out of zone - start turning back
                self.get_logger().warn(f'[{self.robot_ns}] OUT OF ZONE! x={self.robot_x:.2f} - Returning to {self.zone} zone')
                self.state = self.get_turn_direction_to_zone()
                self.turn_duration = 1.57  # 90 degree turn
                self.turn_start_time = self.get_clock().now()
                self.returning_to_zone = True
            # Keep returning_to_zone True until we're back in zone
        else:
            # Back in correct zone - reset flag
            if self.returning_to_zone:
                self.get_logger().info(f'[{self.robot_ns}] Back in {self.zone.upper()} zone! x={self.robot_x:.2f}')
                self.returning_to_zone = False

        # EMERGENCY STOP
        if self.min_front_distance < 0.3:
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0
            self.cmd_vel_pub.publish(cmd)
            self.get_logger().warn(f'[{self.robot_ns}] EMERGENCY STOP! Wall at {self.min_front_distance:.2f}m')
            self.state = 'TURN_LEFT'
            self.turn_duration = 3.14
            self.turn_start_time = self.get_clock().now()
            # Don't reset returning_to_zone - it resets when robot is back in zone
            return

        if self.state == 'FORWARD':
            # Decrease cooldown counter
            if self.boundary_turn_cooldown > 0:
                self.boundary_turn_cooldown -= 1

            # Check if approaching zone boundary (only if not in cooldown AND not returning to zone)
            # Skip this check if returning_to_zone is True to avoid confusion
            if self.is_near_boundary() and self.boundary_turn_cooldown == 0 and not self.returning_to_zone:
                # Turn away from boundary with a bigger turn
                if self.zone == 'left':
                    self.state = 'TURN_LEFT'  # Turn back into left zone
                    self.get_logger().info(f'[{self.robot_ns}] Near boundary (x={self.robot_x:.2f}) - Turning LEFT into zone')
                else:
                    self.state = 'TURN_RIGHT'  # Turn back into right zone
                    self.get_logger().info(f'[{self.robot_ns}] Near boundary (x={self.robot_x:.2f}) - Turning RIGHT into zone')
                self.turn_duration = random.uniform(0.4, 0.5)  # Smaller turn (~25-30 degrees)
                self.turn_start_time = self.get_clock().now()
                self.forward_counter = 0
                self.boundary_turn_cooldown = 100  # Don't check boundary for next 100 callbacks (~10 seconds)

            elif self.min_front_distance < self.obstacle_distance:
                # Obstacle detected - ALWAYS turn toward side with MORE space
                corner_threshold = 0.6
                is_in_corner = (avg_left_dist < corner_threshold and avg_right_dist < corner_threshold)

                if is_in_corner:
                    self.state = 'TURN_LEFT'
                    self.turn_duration = 3.14
                    self.get_logger().info(f'[{self.robot_ns}] CORNER! (L:{avg_left_dist:.2f}m R:{avg_right_dist:.2f}m) - 180 turn')
                else:
                    # Turn toward side with MORE free space (smarter decision)
                    if avg_left_dist > avg_right_dist:
                        self.state = 'TURN_LEFT'
                        self.get_logger().info(f'[{self.robot_ns}] Obstacle - Turn LEFT (more space: L:{avg_left_dist:.2f}m > R:{avg_right_dist:.2f}m)')
                    else:
                        self.state = 'TURN_RIGHT'
                        self.get_logger().info(f'[{self.robot_ns}] Obstacle - Turn RIGHT (more space: R:{avg_right_dist:.2f}m > L:{avg_left_dist:.2f}m)')
                    self.turn_duration = random.uniform(0.4, 0.5)  # Smaller turn (~25-30 degrees)

                self.turn_start_time = self.get_clock().now()
                self.forward_counter = 0
            else:
                # Path is clear - move forward
                cmd.linear.x = self.linear_speed
                cmd.angular.z = 0.0

                # Random exploration turn (bias toward our zone)
                self.forward_counter += 1
                if self.forward_counter >= self.max_forward_time:
                    if self.zone == 'left':
                        self.state = 'TURN_LEFT'  # Bias toward left zone
                    else:
                        self.state = 'TURN_RIGHT'  # Bias toward right zone
                    self.turn_duration = random.uniform(0.25, 0.35)
                    self.turn_start_time = self.get_clock().now()
                    self.forward_counter = 0
                    self.get_logger().info(f'[{self.robot_ns}] Exploration turn toward {self.zone} zone')

        elif self.state == 'TURN_LEFT':
            cmd.linear.x = 0.0
            cmd.angular.z = self.angular_speed
            elapsed = (self.get_clock().now() - self.turn_start_time).nanoseconds / 1e9
            if elapsed >= self.turn_duration:
                self.state = 'FORWARD'
                self.turn_duration = 0
                # Don't reset returning_to_zone here - it resets when robot is back in zone

        elif self.state == 'TURN_RIGHT':
            cmd.linear.x = 0.0
            cmd.angular.z = -self.angular_speed
            elapsed = (self.get_clock().now() - self.turn_start_time).nanoseconds / 1e9
            if elapsed >= self.turn_duration:
                self.state = 'FORWARD'
                self.turn_duration = 0
                # Don't reset returning_to_zone here - it resets when robot is back in zone

        self.cmd_vel_pub.publish(cmd)

        # Log position occasionally
        if not hasattr(self, '_log_count'):
            self._log_count = 0
        self._log_count += 1
        if self._log_count % 50 == 0:
            self.get_logger().info(f'[{self.robot_ns}] Zone: {self.zone.upper()} | Position: x={self.robot_x:.2f}, y={self.robot_y:.2f}')


def main(args=None):
    rclpy.init(args=args)
    explorer = AutoExplorerZone()

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
