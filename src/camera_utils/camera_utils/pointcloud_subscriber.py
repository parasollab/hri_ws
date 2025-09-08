#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import sensor_msgs.msg
import os
import struct
import numpy as np
import open3d as o3d
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Bool

class PointCloudSubscriber(Node):
    def __init__(self):
        super().__init__('pointcloud_subscriber')

        # Declare parameters
        self.declare_parameter('topic', '/camera/depth/color/points')
        self.declare_parameter('output_dir', '/tmp')
        self.declare_parameter('start_topic', '/record_start')

        # Get parameters
        topic_name = self.get_parameter('topic').get_parameter_value().string_value
        output_dir = self.get_parameter('output_dir').get_parameter_value().string_value
        start_topic = self.get_parameter('start_topic').get_parameter_value().string_value

        self.output_dir = output_dir
        os.makedirs(self.output_dir, exist_ok=True)

        self.subscription = self.create_subscription(
            sensor_msgs.msg.PointCloud2,
            topic_name,
            self.listener_callback,
            10
        )

        self.recording = False
        self.create_subscription(
            Bool,
            start_topic,
            self.start_callback,
            10
        )

        self.counter = 0
        self.get_logger().info(f'Subscribed to {topic_name}, saving to {output_dir}')
        self.get_logger().info(f'Listening for start signal on {start_topic}')

    def listener_callback(self, msg):
        if not self.recording:
            return

        num_points = msg.width * msg.height
        if num_points == 0:
            self.get_logger().warn('Received empty point cloud!')
            return

        cloud_points = np.zeros((num_points, 6), dtype=np.float32)  # x, y, z, r, g, b
        for idx, point in enumerate(point_cloud2.read_points(msg, field_names=("x", "y", "z", "rgb"), skip_nans=True)):
            x, y, z, rgb = point
            packed = struct.pack('f', rgb)
            i = struct.unpack('I', packed)[0]
            r = (i >> 16) & 0x0000ff
            g = (i >> 8) & 0x0000ff
            b = (i) & 0x0000ff
            cloud_points[idx] = [x, y, z, r, g, b]

        cloud_points = cloud_points[:idx + 1]

        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(cloud_points[:, :3])
        pcd.colors = o3d.utility.Vector3dVector(cloud_points[:, 3:] / 255.0)

        filename = os.path.join(self.output_dir, f'pointcloud_{self.counter:04d}.ply')
        o3d.io.write_point_cloud(filename, pcd)
        self.get_logger().info(f'Saved {filename}')

        self.counter += 1

    def start_callback(self, msg):
        self.recording = not self.recording
        self.get_logger().info(f'Recording: {self.recording}')

def main(args=None):
    rclpy.init(args=args)
    node = PointCloudSubscriber()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down...')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
