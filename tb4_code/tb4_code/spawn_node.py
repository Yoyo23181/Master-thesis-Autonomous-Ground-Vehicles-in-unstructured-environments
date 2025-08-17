#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped
import math

class WifiAPSpawner(Node):
    def __init__(self):
        super().__init__('wifi_ap_spawner')

        self.declare_parameter('n_anchors', 50)
        self.n_anchors = self.get_parameter('n_anchors').get_parameter_value().integer_value

        # Rectangle boundaries
        self.x_min = -16.0
        self.x_max = 16.0
        self.y_min = -24.0
        self.y_max = 24.0

        self.publisher = self.create_publisher(PointStamped, 'wifi_ap_locations', 10)

        self.ap_locations = self.generate_anchor_positions(
            self.n_anchors, self.x_min, self.x_max, self.y_min, self.y_max
        )
        self.timer = self.create_timer(1.0, self.publish_all_aps)

    def generate_anchor_positions(self, n_anchors, x_min, x_max, y_min, y_max):
        width = x_max - x_min
        height = y_max - y_min

        cols = math.ceil(math.sqrt(n_anchors))
        rows = math.ceil(n_anchors / cols)

        x_spacing = width / (cols - 1) if cols > 1 else 0
        y_spacing = height / (rows - 1) if rows > 1 else 0

        positions = []
        count = 0
        for r in range(rows):
            for c in range(cols):
                if count >= n_anchors:
                    break
                x = x_min + c * x_spacing
                y = y_min + r * y_spacing
                positions.append((x, y))
                count += 1
        return positions

    def publish_all_aps(self):
        now = self.get_clock().now().to_msg()
        for i, (x, y) in enumerate(self.ap_locations):
            msg = PointStamped()
            msg.header.stamp = now
            msg.header.frame_id = 'map'
            msg.point.x = float(x)
            msg.point.y = float(y)
            msg.point.z = 0.0
            self.publisher.publish(msg)
            self.get_logger().info(f"Published AP #{i+1} at ({x:.2f}, {y:.2f})")

        # self.timer.cancel()

def main(args=None):
    rclpy.init(args=args)
    node = WifiAPSpawner()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()