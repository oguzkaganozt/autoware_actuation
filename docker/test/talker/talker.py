#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import time
import random
import sys
import os
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from collections import deque

class MinimalTalker(Node):
    def __init__(self):
        super().__init__('minimal_talker')
        qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10
        )
        self.response_subscription = self.create_subscription(
            String,
            'chatter_response',
            self.response_callback,
            qos)
        self.publisher = self.create_publisher(String, 'chatter', qos)
        
        # Get configuration from environment variables
        msg_size_mb = float(os.getenv('MSG_SIZE_MB', '2'))  # Default 2MB
        msg_frequency = float(os.getenv('MSG_FREQUENCY', '10'))  # Default 10Hz
        
        # Convert MB to bytes for message size
        self.msg_size = int(msg_size_mb * 1024 * 1024)
        timer_period = 1.0 / msg_frequency
        
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0
        self.pending_msgs = {}
        self.rtts = deque(maxlen=100)
        self.create_timer(5.0, self.report_stats)
        
        self.get_logger().info(
            f'Starting talker with:\n'
            f'Message size: {msg_size_mb:.2f} MB\n'
            f'Frequency: {msg_frequency:.1f} Hz\n'
            f'Period: {timer_period*1000:.1f} ms'
        )

    def timer_callback(self):
        msg = String()
        send_time = time.time()
        # Simulate point cloud data with x,y,z,intensity format
        point_data = ''.join(random.choices('0123456789', k=self.msg_size))
        msg.data = f'{self.i}|{point_data}'
        
        self.pending_msgs[self.i] = send_time
        self.publisher.publish(msg)
        msg_size_mb = sys.getsizeof(msg.data) / (1024 * 1024)
        self.get_logger().info(f'Publishing point cloud {self.i}, size: {msg_size_mb:.2f} MB')
        self.i += 1

    def report_stats(self):
        if self.rtts:
            avg_rtt = sum(self.rtts) / len(self.rtts)
            min_rtt = min(self.rtts)
            max_rtt = max(self.rtts)
            self.get_logger().info(
                f'\nRTT Stats:\n'
                f'Average: {avg_rtt*1000:.2f}ms\n'
                f'Min: {min_rtt*1000:.2f}ms\n'
                f'Max: {max_rtt*1000:.2f}ms\n'
                f'Samples: {len(self.rtts)}'
            )

    def response_callback(self, msg):
        seq_num = int(msg.data)
        if seq_num in self.pending_msgs:
            rtt = time.time() - self.pending_msgs[seq_num]
            msg_size_mb = sys.getsizeof(msg.data) / (1024 * 1024)
            self.get_logger().info(
                f'RTT for point cloud {seq_num}: {rtt*1000:.2f}ms, '
                f'Size: {msg_size_mb:.2f}MB, '
                f'Throughput: {(msg_size_mb/rtt):.2f}MB/s'
            )
            del self.pending_msgs[seq_num]
            self.rtts.append(rtt)

def main(args=None):
    rclpy.init(args=args)
    minimal_talker = MinimalTalker()
    rclpy.spin(minimal_talker)
    minimal_talker.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()