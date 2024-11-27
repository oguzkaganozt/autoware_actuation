#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class MinimalListener(Node):
    def __init__(self):
        super().__init__('minimal_listener')
        self.subscription = self.create_subscription(
            String,
            'chatter',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        self.get_logger().info('I heard: "%s"' % msg.data)

def main(args=None):
    rclpy.init(args=args)
    minimal_listener = MinimalListener()
    rclpy.spin(minimal_listener)
    minimal_listener.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()