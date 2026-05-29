#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from moveit_msgs.msg import CollisionObject


class LatencyMeasurer(Node):
    def __init__(self):
        super().__init__('latency_measurer')
        self.create_subscription(CollisionObject, '/collision_object', self._cb, 10)
        self.get_logger().info('LatencyMeasurer up. Subscribing to /collision_object.')

    def _cb(self, msg: CollisionObject):
        now_ns = self.get_clock().now().nanoseconds
        sent_ns = msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec
        if sent_ns == 0:
            self.get_logger().warn('Received message with no timestamp (stamp is zero)')
            return
        latency_ms = (now_ns - sent_ns) / 1_000_000.0
        self.get_logger().info(f'[{msg.id}] op={msg.operation}  latency={latency_ms:.2f} ms')


def main():
    rclpy.init()
    node = LatencyMeasurer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
