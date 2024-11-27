#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import time
from collections import deque
from statistics import mean
import sys
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
import os

class MinimalListener(Node):
    def __init__(self):
        super().__init__('minimal_listener')
        qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        # Get configuration from environment variables
        msg_size_mb = float(os.getenv('MSG_SIZE_MB', '2'))  # Default 2MB
        msg_frequency = float(os.getenv('MSG_FREQUENCY', '10'))  # Default 10Hz
        
        self.get_logger().info(
            f'Starting listener with:\n'
            f'Expected message size: {msg_size_mb:.2f} MB\n'
            f'Expected frequency: {msg_frequency:.1f} Hz'
        )
        
        self.subscription = self.create_subscription(
            String,
            'chatter',
            self.listener_callback,
            qos)
        self.response_publisher = self.create_publisher(
            String,
            'chatter_response',
            qos)
        self.latencies = deque(maxlen=100)
        self.bytes_received = 0
        self.start_time = time.time()
        self.msg_count = 0
        
        self.create_timer(5.0, self.report_stats)

    def listener_callback(self, msg):
        seq_num = msg.data.split('|')[0]
        msg_size_mb = sys.getsizeof(msg.data) / (1024 * 1024)
        
        response = String()
        response.data = seq_num
        self.response_publisher.publish(response)
        
        self.bytes_received += sys.getsizeof(msg.data)
        self.msg_count += 1
        
        self.get_logger().info(
            f'Received point cloud {seq_num}, '
            f'size: {msg_size_mb:.2f} MB'
        )

    def report_stats(self):
        elapsed_time = time.time() - self.start_time
        throughput_mbps = (self.bytes_received * 8) / (1024 * 1024 * elapsed_time)
        msg_rate = self.msg_count / elapsed_time
        avg_msg_size = self.bytes_received / (self.msg_count * 1024 * 1024) if self.msg_count > 0 else 0
        
        self.get_logger().info(
            f'\nPerformance Stats:\n'
            f'Throughput: {throughput_mbps:.2f} Mbps\n'
            f'Message Rate: {msg_rate:.2f} Hz\n'
            f'Average Message Size: {avg_msg_size:.2f} MB\n'
            f'Messages Received: {self.msg_count}\n'
            f'Total Data: {self.bytes_received/(1024*1024):.2f} MB\n'
            f'Time Elapsed: {elapsed_time:.1f}s'
        )

def main(args=None):
    rclpy.init(args=args)
    minimal_listener = MinimalListener()
    rclpy.spin(minimal_listener)
    minimal_listener.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()