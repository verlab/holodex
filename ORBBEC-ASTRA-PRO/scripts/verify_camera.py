#!/usr/bin/env python3
"""
Verification script for Orbbec Astra Pro on ROS 2.
Subscribes to Depth, RGB and PointCloud2 topics to confirm live streaming.
"""
import sys
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, Image

class CameraVerifier(Node):
    def __init__(self):
        super().__init__('camera_verifier')
        self.depth_count = 0
        self.color_count = 0
        self.cloud_count = 0

        self.sub_depth = self.create_subscription(
            Image, '/camera/depth/image_raw', self.depth_callback, 10)
        self.sub_color = self.create_subscription(
            Image, '/camera/color/image_raw', self.color_callback, 10)
        self.sub_cloud = self.create_subscription(
            PointCloud2, '/camera/depth/points', self.cloud_callback, 10)
        self.get_logger().info('CameraVerifier waiting for topics: /camera/depth/image_raw, /camera/color/image_raw, /camera/depth/points ...')

    def depth_callback(self, msg):
        self.depth_count += 1
        if self.depth_count <= 3:
            self.get_logger().info(f'[DEPTH] Frame {self.depth_count}: {msg.width}x{msg.height}, encoding={msg.encoding}')

    def color_callback(self, msg):
        self.color_count += 1
        if self.color_count <= 3:
            self.get_logger().info(f'[COLOR] Frame {self.color_count}: {msg.width}x{msg.height}, encoding={msg.encoding}')

    def cloud_callback(self, msg):
        self.cloud_count += 1
        if self.cloud_count <= 3:
            self.get_logger().info(f'[POINTCLOUD] Frame {self.cloud_count}: {msg.width}x{msg.height} ({msg.width*msg.height} pts), frame_id={msg.header.frame_id}')

    def all_received(self):
        return self.depth_count >= 3 and self.color_count >= 3 and self.cloud_count >= 3

def main():
    rclpy.init()
    node = CameraVerifier()
    while rclpy.ok() and not node.all_received():
        rclpy.spin_once(node, timeout_sec=0.1)
    
    if node.all_received():
        node.get_logger().info('SUCCESS: All streams (Depth, RGB, and PointCloud2) verified!')
        ret = 0
    else:
        node.get_logger().error('Timeout or interrupted before all frames received.')
        ret = 1
        
    node.destroy_node()
    rclpy.shutdown()
    sys.exit(ret)

if __name__ == '__main__':
    main()
