#!/usr/bin/env python3
import csv
import os
from datetime import datetime

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32


class FPSMonitor(Node):
    def __init__(self):
        super().__init__('fps_monitor')
        self.declare_parameter('output_dir', '/tmp')
        output_dir = self.get_parameter('output_dir').get_parameter_value().string_value

        timestamp = datetime.now().strftime('%Y_%m_%d_%H_%M_%S')
        self._csv_path = os.path.join(output_dir, f'fps_{timestamp}.csv')
        self._initialized = False

        self.create_subscription(Int32, '/fps', self._cb, 10)
        self.get_logger().info(f'FPSMonitor up. Writing to {self._csv_path}')

    def _cb(self, msg: Int32):
        now = self.get_clock().now()
        sec = now.seconds_nanoseconds()[0]
        nanosec = now.seconds_nanoseconds()[1]
        with open(self._csv_path, 'a', newline='') as f:
            writer = csv.writer(f)
            if not self._initialized:
                writer.writerow(['ros_sec', 'ros_nanosec', 'fps'])
                self._initialized = True
            writer.writerow([sec, nanosec, msg.data])
        self.get_logger().info(f'Unity FPS: {msg.data}')


def main():
    rclpy.init()
    node = FPSMonitor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
