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
from std_msgs.msg import Bool
import tf2_ros
from tf2_ros import TransformException
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

        # Subscriber for exploration complete signal
        self.exploration_complete_sub = self.create_subscription(
            Bool,
            '/exploration_complete',
            self.exploration_complete_callback,
            10
        )
        self.exploration_stopped = False

        # Robot position
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_yaw = 0.0

        # Robot state
        self.state = 'FORWARD'
        self.obstacle_distance = 1.0  # Back to 1.0m for better detection
        self.min_front_distance = float('inf')

        # Movement parameters
        self.linear_speed = 0.18  # Slightly faster
        self.angular_speed = 1.0  # Normal turning speed

        # Timer for state changes
        self.turn_duration = 0
        self.turn_start_time = None

        # Exploration
        self.forward_counter = 0
        self.max_forward_time = 150

        # Zone boundary configuration
        # spawn_offset = how far from center (world x=0) the robot spawns
        # For 10m world: robots spawn at x=-2 and x=+2, so spawn_offset=2.0
        # For 20m world: robots spawn at x=-5 and x=+5, so spawn_offset=5.0
        self.declare_parameter('spawn_offset', 2.0)
        self.spawn_offset = self.get_parameter('spawn_offset').get_parameter_value().double_value

        # IMPORTANT: Odometry starts at (0,0) at spawn position!
        # Robot1 spawns at world x = -spawn_offset, odom x = 0
        # Robot2 spawns at world x = +spawn_offset, odom x = 0
        #
        # To convert: world_x = odom_x + spawn_world_x
        #   Robot1: world_x = odom_x + (-spawn_offset) = odom_x - spawn_offset
        #   Robot2: world_x = odom_x + (+spawn_offset) = odom_x + spawn_offset
        #
        # Boundary is at world x = 0
        #   Robot1 (left): boundary when odom_x = +spawn_offset (traveled right to center)
        #   Robot2 (right): boundary when odom_x = -spawn_offset (traveled left to center)

        if self.zone == 'left':
            # Robot1: spawns at world x=-spawn_offset
            # Boundary at world x=0 means odom x = spawn_offset
            self.spawn_world_x = -self.spawn_offset
            self.boundary_odom_x = self.spawn_offset  # Exact boundary at world x=0
        else:
            # Robot2: spawns at world x=+spawn_offset
            # Boundary at world x=0 means odom x = -spawn_offset
            self.spawn_world_x = self.spawn_offset
            self.boundary_odom_x = -self.spawn_offset  # Exact boundary at world x=0

        self.get_logger().info(f'Spawn world x: {self.spawn_world_x}')
        self.get_logger().info(f'Boundary odom x: {self.boundary_odom_x}')

        self.returning_to_zone = False
        self.boundary_turn_cooldown = 0  # Cooldown counter after boundary turn

        # Stuck detection - if robot stays in same spot for 10 seconds, turn around
        self.stuck_check_position = (0.0, 0.0)
        self.stuck_check_time = self.get_clock().now()
        self.stuck_threshold = 0.3  # Must move at least 0.3m in 10 seconds
        self.stuck_timeout = 10.0  # seconds

        # TF2 for getting actual world position (more accurate than odom math)
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # World position from TF (updated in timer)
        self.world_x = 0.0
        self.world_y = 0.0
        self.tf_available = False

        # Timer to check TF at high frequency for boundary detection
        self.tf_timer = self.create_timer(0.05, self.tf_callback)  # 20Hz TF check

        # Escape state for stuck situations
        self.escape_state = None  # 'REVERSE', 'TURN'
        self.escape_start_time = None

        # Robot-to-robot collision detection
        self.other_robot_ns = 'robot2' if self.robot_ns == 'robot1' else 'robot1'
        self.other_robot_x = 0.0
        self.other_robot_y = 0.0
        self.other_robot_available = False
        self.collision_distance = 0.6  # Distance at which to trigger collision avoidance
        self.collision_warning_distance = 1.0  # Distance at which to slow down
        self.collision_state = None  # 'REVERSE', 'TURN'
        self.collision_start_time = None

        self.get_logger().info(f'Topics: cmd_vel={cmd_vel_topic}, scan={scan_topic}, odom={odom_topic}')
        self.get_logger().info(f'Obstacle avoidance distance: {self.obstacle_distance}m')
        self.get_logger().info(f'Stuck detection: {self.stuck_timeout}s timeout, {self.stuck_threshold}m threshold')
        self.get_logger().info(f'Using TF for world position (frame: world -> {self.robot_ns}/base_footprint)')

    def tf_callback(self):
        """Get robot's actual world position from TF - triggers boundary response and collision check"""
        if self.exploration_stopped:
            return

        try:
            # Get transform from world to this robot's base
            transform = self.tf_buffer.lookup_transform(
                'world',
                f'{self.robot_ns}/base_footprint',
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.1)
            )
            self.world_x = transform.transform.translation.x
            self.world_y = transform.transform.translation.y
            self.tf_available = True

            # IMMEDIATE boundary check using ACTUAL world coordinates from TF!
            if self.is_in_wrong_zone_tf() and not self.returning_to_zone:
                self.get_logger().warn(f'[{self.robot_ns}] !!BOUNDARY CROSSED!! World x={self.world_x:.2f} (TF) - IMMEDIATE TURN')

                # IMMEDIATELY start turning back
                self.state = self.get_turn_direction_to_zone()
                self.turn_duration = 1.57  # 90 degree turn
                self.turn_start_time = self.get_clock().now()
                self.returning_to_zone = True

                # Publish turn command immediately
                cmd = Twist()
                cmd.linear.x = 0.0
                if self.state == 'TURN_LEFT':
                    cmd.angular.z = self.angular_speed
                else:
                    cmd.angular.z = -self.angular_speed
                self.cmd_vel_pub.publish(cmd)

        except TransformException:
            # TF not available yet, use odom-based calculation as fallback
            if not self.tf_available:
                pass  # Don't spam logs during startup

        # Also track other robot's position for collision detection
        try:
            other_transform = self.tf_buffer.lookup_transform(
                'world',
                f'{self.other_robot_ns}/base_footprint',
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.05)
            )
            self.other_robot_x = other_transform.transform.translation.x
            self.other_robot_y = other_transform.transform.translation.y
            self.other_robot_available = True

            # Check distance to other robot
            if self.tf_available:
                distance = self.get_distance_to_other_robot()
                if distance < self.collision_distance and self.collision_state is None:
                    self.get_logger().warn(f'[{self.robot_ns}] !!COLLISION WARNING!! Distance to {self.other_robot_ns}: {distance:.2f}m - AVOIDING!')
                    self.collision_state = 'REVERSE'
                    self.collision_start_time = self.get_clock().now()

                    # Immediately start reversing
                    cmd = Twist()
                    cmd.linear.x = -0.15
                    cmd.angular.z = 0.0
                    self.cmd_vel_pub.publish(cmd)

        except TransformException:
            # Other robot TF not available
            pass

    def get_distance_to_other_robot(self):
        """Calculate distance to the other robot"""
        dx = self.world_x - self.other_robot_x
        dy = self.world_y - self.other_robot_y
        return math.sqrt(dx*dx + dy*dy)

    def get_turn_away_from_other_robot(self):
        """Determine which way to turn to move away from the other robot"""
        # Calculate angle to other robot
        dx = self.other_robot_x - self.world_x
        dy = self.other_robot_y - self.world_y
        angle_to_other = math.atan2(dy, dx)

        # Turn in the opposite direction
        # If other robot is to our left (positive angle relative to us), turn right
        # If other robot is to our right (negative angle relative to us), turn left
        angle_diff = angle_to_other - self.robot_yaw
        # Normalize to [-pi, pi]
        while angle_diff > math.pi:
            angle_diff -= 2 * math.pi
        while angle_diff < -math.pi:
            angle_diff += 2 * math.pi

        if angle_diff > 0:
            return 'TURN_RIGHT'  # Other robot is to our left, turn right
        else:
            return 'TURN_LEFT'   # Other robot is to our right, turn left

    def odom_callback(self, msg):
        """Track robot position from odometry"""
        self.robot_x = msg.pose.pose.position.x
        self.robot_y = msg.pose.pose.position.y

        # Extract yaw from quaternion
        q = msg.pose.pose.orientation
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        self.robot_yaw = math.atan2(siny_cosp, cosy_cosp)

        # Fallback boundary check if TF not available
        if not self.tf_available and self.is_in_wrong_zone() and not self.returning_to_zone and not self.exploration_stopped:
            world_x = self.robot_x + self.spawn_world_x
            self.get_logger().warn(f'[{self.robot_ns}] !!BOUNDARY CROSSED!! Odom x={self.robot_x:.2f}, World x={world_x:.2f} (odom fallback)')

            self.state = self.get_turn_direction_to_zone()
            self.turn_duration = 1.57
            self.turn_start_time = self.get_clock().now()
            self.returning_to_zone = True

            cmd = Twist()
            cmd.linear.x = 0.0
            if self.state == 'TURN_LEFT':
                cmd.angular.z = self.angular_speed
            else:
                cmd.angular.z = -self.angular_speed
            self.cmd_vel_pub.publish(cmd)

    def exploration_complete_callback(self, msg):
        """Handle exploration complete signal"""
        if msg.data and not self.exploration_stopped:
            self.exploration_stopped = True
            self.get_logger().info('=' * 50)
            self.get_logger().info('EXPLORATION COMPLETE - STOPPING ROBOT')
            self.get_logger().info('=' * 50)
            # Stop the robot
            stop_cmd = Twist()
            self.cmd_vel_pub.publish(stop_cmd)

    def is_in_wrong_zone(self):
        """Check if robot has crossed into the wrong zone using odom (fallback)"""
        if self.zone == 'left':
            return self.robot_x > self.boundary_odom_x
        else:
            return self.robot_x < self.boundary_odom_x

    def is_in_wrong_zone_tf(self):
        """Check if robot has crossed into the wrong zone using TF world coordinates"""
        # Boundary is at world x = 0
        if self.zone == 'left':
            # Left zone robot should stay in x < 0
            return self.world_x > 0.0
        else:
            # Right zone robot should stay in x > 0
            return self.world_x < 0.0

    def is_near_boundary(self):
        """Check if robot is approaching the zone boundary (within 0.5m)"""
        if self.tf_available:
            # Use TF world coordinates (more accurate)
            if self.zone == 'left':
                return self.world_x > -0.5  # Within 0.5m of x=0
            else:
                return self.world_x < 0.5   # Within 0.5m of x=0
        else:
            # Fallback to odom calculation
            if self.zone == 'left':
                return self.robot_x > self.boundary_odom_x - 0.5
            else:
                return self.robot_x < self.boundary_odom_x + 0.5

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

    def check_if_stuck(self):
        """Check if robot has been stuck in same position for too long"""
        current_time = self.get_clock().now()
        elapsed = (current_time - self.stuck_check_time).nanoseconds / 1e9

        if elapsed >= self.stuck_timeout:
            # Calculate distance moved since last check
            dx = self.robot_x - self.stuck_check_position[0]
            dy = self.robot_y - self.stuck_check_position[1]
            distance_moved = math.sqrt(dx*dx + dy*dy)

            # Update check position and time
            self.stuck_check_position = (self.robot_x, self.robot_y)
            self.stuck_check_time = current_time

            if distance_moved < self.stuck_threshold:
                return True  # Robot is stuck!

        return False

    def scan_callback(self, msg):
        """Process laser scan data and make movement decisions"""
        # Stop processing if exploration is complete
        if self.exploration_stopped:
            return

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

        # Use TF-based check if available, otherwise fallback to odom
        in_wrong_zone = self.is_in_wrong_zone_tf() if self.tf_available else self.is_in_wrong_zone()

        # ZONE CHECK - Check if we're back in zone
        if not in_wrong_zone and self.returning_to_zone:
            # Back in correct zone - reset flag
            pos_str = f'TF x={self.world_x:.2f}' if self.tf_available else f'Odom x={self.robot_x:.2f}'
            self.get_logger().info(f'[{self.robot_ns}] Back in {self.zone.upper()} zone! {pos_str}')
            self.returning_to_zone = False

        # PRIORITY: If still in wrong zone after turning, keep trying to return!
        if in_wrong_zone and self.returning_to_zone and self.state == 'FORWARD':
            # Still in wrong zone - check if we're facing the right direction
            if self.zone == 'left':
                facing_correct = self.robot_yaw > math.pi/2 or self.robot_yaw < -math.pi/2
            else:
                facing_correct = -math.pi/2 < self.robot_yaw < math.pi/2

            if not facing_correct:
                # Still facing wrong way - turn again!
                pos_str = f'TF x={self.world_x:.2f}' if self.tf_available else f'Odom x={self.robot_x + self.spawn_world_x:.2f}'
                self.get_logger().warn(f'[{self.robot_ns}] Still in WRONG ZONE ({pos_str})! Turning again...')
                self.state = self.get_turn_direction_to_zone()
                self.turn_duration = 1.57
                self.turn_start_time = self.get_clock().now()
            else:
                # Facing correct direction - move forward to return
                cmd.linear.x = self.linear_speed
                cmd.angular.z = 0.0
                self.cmd_vel_pub.publish(cmd)
                return

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

        # COLLISION AVOIDANCE - robot-to-robot collision handling
        if self.collision_state == 'REVERSE':
            elapsed = (self.get_clock().now() - self.collision_start_time).nanoseconds / 1e9
            if elapsed < 1.5:  # Reverse for 1.5 seconds
                cmd.linear.x = -0.15  # Slow reverse
                cmd.angular.z = 0.0
                self.cmd_vel_pub.publish(cmd)
                return
            else:
                # Done reversing, now turn away from other robot
                self.collision_state = 'TURN'
                self.collision_start_time = self.get_clock().now()
                self.get_logger().info(f'[{self.robot_ns}] Collision avoidance: Done reversing, turning away...')

        if self.collision_state == 'TURN':
            elapsed = (self.get_clock().now() - self.collision_start_time).nanoseconds / 1e9
            if elapsed < 1.5:  # Turn for 1.5 seconds (~90 degrees)
                cmd.linear.x = 0.0
                turn_dir = self.get_turn_away_from_other_robot()
                if turn_dir == 'TURN_LEFT':
                    cmd.angular.z = self.angular_speed
                else:
                    cmd.angular.z = -self.angular_speed
                self.cmd_vel_pub.publish(cmd)
                return
            else:
                # Done collision avoidance
                self.collision_state = None
                self.state = 'FORWARD'
                self.get_logger().info(f'[{self.robot_ns}] Collision avoidance complete! Resuming exploration.')

        # ESCAPE STATE HANDLING - reverse first, then turn (for stuck situations)
        if self.escape_state == 'REVERSE':
            elapsed = (self.get_clock().now() - self.escape_start_time).nanoseconds / 1e9
            if elapsed < 1.0:  # Reverse for 1 second
                cmd.linear.x = -0.15  # Slow reverse
                cmd.angular.z = 0.0
                self.cmd_vel_pub.publish(cmd)
                return
            else:
                # Done reversing, now turn
                self.escape_state = 'TURN'
                self.escape_start_time = self.get_clock().now()
                self.get_logger().info(f'[{self.robot_ns}] Escape: Done reversing, now turning...')

        if self.escape_state == 'TURN':
            elapsed = (self.get_clock().now() - self.escape_start_time).nanoseconds / 1e9
            if elapsed < 2.0:  # Turn for 2 seconds (~180 degrees)
                cmd.linear.x = 0.0
                cmd.angular.z = self.angular_speed
                self.cmd_vel_pub.publish(cmd)
                return
            else:
                # Done escaping
                self.escape_state = None
                self.state = 'FORWARD'
                self.get_logger().info(f'[{self.robot_ns}] Escape complete! Resuming exploration.')

        # STUCK DETECTION - if robot hasn't moved for 10 seconds, start escape sequence
        if self.check_if_stuck() and self.state == 'FORWARD' and self.escape_state is None:
            self.get_logger().warn(f'[{self.robot_ns}] STUCK DETECTED! Starting escape: reverse then turn')
            self.escape_state = 'REVERSE'
            self.escape_start_time = self.get_clock().now()
            self.forward_counter = 0
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

        # Log position frequently to debug boundary detection
        if not hasattr(self, '_log_count'):
            self._log_count = 0
        self._log_count += 1
        if self._log_count % 20 == 0:  # More frequent logging
            if self.tf_available:
                status = "OK" if not self.is_in_wrong_zone_tf() else "**WRONG ZONE**"
                self.get_logger().info(f'[{self.robot_ns}] {status} | TF World x={self.world_x:.2f} | Boundary=0.0')
            else:
                world_x = self.robot_x + self.spawn_world_x
                status = "OK" if not self.is_in_wrong_zone() else "**WRONG ZONE**"
                self.get_logger().info(f'[{self.robot_ns}] {status} | Odom x={self.robot_x:.2f} | World x={world_x:.2f} (calc) | Boundary={self.boundary_odom_x:.2f}')


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
