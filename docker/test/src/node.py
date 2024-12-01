#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import socket
import random

class TestNode(Node):
    def __init__(self):
        # Use hostname as node name for unique identification
        self.hostname = socket.gethostname()
        super().__init__(f'test_node_{self.hostname}')

        # Create publisher
        self.publisher = self.create_publisher(
            String,
            'test_topic',
            10
        )

        # Create subscriber
        self.subscription = self.create_subscription(
            String,
            'test_topic',
            self.listener_callback,
            10
        )

        # Create timer for publishing messages
        self.timer = self.create_timer(1.0, self.timer_callback)
        self.msg_count = 0

        self.get_logger().info(f'Node initialized on {self.hostname}')

    def timer_callback(self):
        msg = String()
        msg.data = f'Hello from {self.hostname}! Count: {self.msg_count}'
        self.publisher.publish(msg)
        self.msg_count += 1

    def listener_callback(self, msg):
        if self.hostname not in msg.data:
            self.get_logger().info(f'Received: {msg.data}')

def main(args=None):
    rclpy.init(args=args)
    node = TestNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
