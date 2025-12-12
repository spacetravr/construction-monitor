#!/usr/bin/env python3
"""
TF Relay Node

Relays transforms from namespaced TF topics (/robot1/tf, /robot2/tf)
to the global /tf topic for RViz visualization.

This is needed when using TF remapping for multi-robot SLAM isolation.
"""

import rclpy
from rclpy.node import Node
from tf2_msgs.msg import TFMessage


class TFRelay(Node):
    def __init__(self):
        super().__init__('tf_relay')

        # Publishers to global TF
        self.tf_pub = self.create_publisher(TFMessage, '/tf', 10)
        self.tf_static_pub = self.create_publisher(TFMessage, '/tf_static', 10)

        # Subscribe to robot1 TF
        self.robot1_tf_sub = self.create_subscription(
            TFMessage,
            '/robot1/tf',
            lambda msg: self.tf_callback(msg, 'robot1'),
            10
        )
        self.robot1_tf_static_sub = self.create_subscription(
            TFMessage,
            '/robot1/tf_static',
            lambda msg: self.tf_static_callback(msg, 'robot1'),
            10
        )

        # Subscribe to robot2 TF
        self.robot2_tf_sub = self.create_subscription(
            TFMessage,
            '/robot2/tf',
            lambda msg: self.tf_callback(msg, 'robot2'),
            10
        )
        self.robot2_tf_static_sub = self.create_subscription(
            TFMessage,
            '/robot2/tf_static',
            lambda msg: self.tf_static_callback(msg, 'robot2'),
            10
        )

        self.get_logger().info('TF Relay started')
        self.get_logger().info('Relaying /robot1/tf and /robot2/tf to /tf')

    def tf_callback(self, msg, robot_name):
        """Relay dynamic TF."""
        self.tf_pub.publish(msg)

    def tf_static_callback(self, msg, robot_name):
        """Relay static TF."""
        self.tf_static_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = TFRelay()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
