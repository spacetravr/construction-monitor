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

        # Robot position (from odom)
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_yaw = 0.0

        # Initial odom position - captured on first odom message
        # Used to calculate world position correctly
        self.initial_odom_x = None
        self.initial_odom_y = None

        # Robot state
        self.state = 'FORWARD'
        self.obstacle_distance = 0.6  # Reduced to 0.6m to explore tighter spaces
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

        # Center exploration - periodically drive toward zone center
        # Less frequent to allow more wall-following exploration
        self.center_explore_timer = self.get_clock().now()
        self.center_explore_interval = 90.0  # Every 90 seconds (was 30)
        self.center_explore_active = False
        self.center_explore_start_time = None

        # Zone boundary configuration
        # spawn_offset = how far from center (world x=0) the robot spawns
        # For my_house_small (15m world): robots spawn at x=-3.75 and x=+3.75, so spawn_offset=3.75
        # For 20m world: robots spawn at x=-5 and x=+5, so spawn_offset=5.0
        self.declare_parameter('spawn_offset', 3.75)  # Default for my_house_small (15m world)
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

        # TF2 for collision detection with other robot only
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # World position calculated from odom (reliable for boundary detection)
        # TF from SLAM doesn't match Gazebo world coordinates!
        self.world_x = self.spawn_world_x  # Start at spawn position
        self.world_y = 0.0

        # Timer to check for robot-to-robot collision
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
        self.get_logger().info(f'Using ODOM-based world position: world_x = odom_x + spawn_world_x ({self.spawn_world_x})')
        self.get_logger().info(f'Boundary at world x=0, which is odom x={self.boundary_odom_x}')

    def tf_callback(self):
        """Check for robot-to-robot collision using TF (not used for boundary detection)"""
        if self.exploration_stopped:
            return

        # Track other robot's position for collision detection
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

            # Check distance to other robot using odom-based positions
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
        """Track robot position from odometry and calculate world position"""
        self.robot_x = msg.pose.pose.position.x
        self.robot_y = msg.pose.pose.position.y

        # Capture initial odom position on first message
        # Odom doesn't always start at (0,0) - Gazebo may have an offset
        if self.initial_odom_x is None:
            self.initial_odom_x = self.robot_x
            self.initial_odom_y = self.robot_y
            self.get_logger().info(f'[{self.robot_ns}] Initial odom captured: x={self.initial_odom_x:.2f}, y={self.initial_odom_y:.2f}')

        # Calculate world position from odom DELTA (not absolute odom)
        # world_x = spawn_world_x + (current_odom - initial_odom)
        # This way, at spawn: world_x = spawn_world_x + 0 = spawn_world_x (correct!)
        odom_delta_x = self.robot_x - self.initial_odom_x
        odom_delta_y = self.robot_y - self.initial_odom_y
        self.world_x = self.spawn_world_x + odom_delta_x
        self.world_y = odom_delta_y  # Y spawn offset not needed for boundary

        # Extract yaw from quaternion
        q = msg.pose.pose.orientation
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        self.robot_yaw = math.atan2(siny_cosp, cosy_cosp)

        # BOUNDARY CHECK using odom-based world position
        if self.is_in_wrong_zone() and not self.returning_to_zone and not self.exploration_stopped:
            self.get_logger().warn(f'[{self.robot_ns}] !!BOUNDARY CROSSED!! Odom x={self.robot_x:.2f} -> World x={self.world_x:.2f} (boundary at 0)')

            # CANCEL center exploration if active
            if self.center_explore_active:
                self.center_explore_active = False
                self.get_logger().warn(f'[{self.robot_ns}] CENTER EXPLORATION CANCELLED - Boundary crossed!')

            self.state = self.get_turn_direction_to_zone()
            self.turn_duration = 1.57  # 90 degree turn
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
        """Check if robot has crossed into the wrong zone using odom-based world_x"""
        # Boundary is at world x = 0
        # world_x is calculated as: odom_x + spawn_world_x
        if self.zone == 'left':
            # Left zone robot should stay in world x < 0
            return self.world_x > 0.0
        else:
            # Right zone robot should stay in world x > 0
            return self.world_x < 0.0

    def is_near_boundary(self):
        """Check if robot is approaching the zone boundary (within 0.5m of world x=0)"""
        # Use odom-based world_x calculation
        if self.zone == 'left':
            return self.world_x > -0.5  # Within 0.5m of x=0
        else:
            return self.world_x < 0.5   # Within 0.5m of x=0

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

    def get_turn_toward_zone_center(self):
        """Calculate turn direction to face toward the center of assigned zone"""
        # Zone centers:
        # Left zone: center around x = -half of zone width (e.g., x = -2.5 for 5m half-width)
        # Right zone: center around x = +half of zone width (e.g., x = +2.5 for 5m half-width)
        # Y center is typically around 0

        if self.zone == 'left':
            # Target is center of left zone (negative x, middle y)
            target_x = -self.spawn_offset / 2  # Middle of left zone
        else:
            # Target is center of right zone (positive x, middle y)
            target_x = self.spawn_offset / 2  # Middle of right zone

        target_y = 0.0  # Center of y axis

        # Calculate angle to target from current position using odom-based world position
        dx = target_x - self.world_x
        dy = target_y - self.world_y

        target_angle = math.atan2(dy, dx)

        # Calculate angle difference
        angle_diff = target_angle - self.robot_yaw
        # Normalize to [-pi, pi]
        while angle_diff > math.pi:
            angle_diff -= 2 * math.pi
        while angle_diff < -math.pi:
            angle_diff += 2 * math.pi

        # Return turn direction
        if angle_diff > 0:
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

        # Use odom-based world_x for boundary check (reliable)
        in_wrong_zone = self.is_in_wrong_zone()

        # ZONE CHECK - Check if we're back in zone
        if not in_wrong_zone and self.returning_to_zone:
            # Back in correct zone - reset flag
            self.get_logger().info(f'[{self.robot_ns}] Back in {self.zone.upper()} zone! World x={self.world_x:.2f}')
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
                self.get_logger().warn(f'[{self.robot_ns}] Still in WRONG ZONE (World x={self.world_x:.2f})! Turning again...')
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
                # CANCEL center exploration if active - boundary takes priority!
                if self.center_explore_active:
                    self.center_explore_active = False
                    self.get_logger().warn(f'[{self.robot_ns}] CENTER EXPLORATION CANCELLED - Near boundary!')

                # Turn away from boundary with a bigger turn
                if self.zone == 'left':
                    self.state = 'TURN_LEFT'  # Turn back into left zone
                    self.get_logger().warn(f'[{self.robot_ns}] !! NEAR BOUNDARY !! World x={self.world_x:.2f} - Turning LEFT into zone')
                else:
                    self.state = 'TURN_RIGHT'  # Turn back into right zone
                    self.get_logger().warn(f'[{self.robot_ns}] !! NEAR BOUNDARY !! World x={self.world_x:.2f} - Turning RIGHT into zone')
                self.turn_duration = random.uniform(0.8, 1.2)  # Bigger turn (~45-70 degrees) to get away from boundary
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

                # CENTER EXPLORATION - periodically turn toward zone center
                current_time = self.get_clock().now()
                center_elapsed = (current_time - self.center_explore_timer).nanoseconds / 1e9

                if center_elapsed >= self.center_explore_interval and not self.center_explore_active:
                    # Time to explore center!
                    self.center_explore_active = True
                    self.center_explore_start_time = current_time
                    self.center_explore_timer = current_time  # Reset timer

                    # Turn toward zone center
                    self.state = self.get_turn_toward_zone_center()
                    self.turn_duration = random.uniform(0.8, 1.5)  # ~45-90 degree turn
                    self.turn_start_time = current_time
                    self.forward_counter = 0
                    self.get_logger().info(f'[{self.robot_ns}] CENTER EXPLORATION: Turning toward zone center!')

                elif self.center_explore_active:
                    # Currently in center exploration mode - drive forward for a while
                    explore_elapsed = (current_time - self.center_explore_start_time).nanoseconds / 1e9
                    if explore_elapsed > 8.0:  # Drive toward center for 8 seconds
                        self.center_explore_active = False
                        self.get_logger().info(f'[{self.robot_ns}] CENTER EXPLORATION complete, resuming normal exploration')

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
            status = "OK" if not self.is_in_wrong_zone() else "**WRONG ZONE**"
            if self.initial_odom_x is not None:
                delta_x = self.robot_x - self.initial_odom_x
                self.get_logger().info(f'[{self.robot_ns}] {status} | Delta x={delta_x:.2f} -> World x={self.world_x:.2f} | Boundary=0.0')
            else:
                self.get_logger().info(f'[{self.robot_ns}] Waiting for initial odom...')


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
