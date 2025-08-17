# #!/usr/bin/env python3
#
# import rclpy
# from rclpy.node import Node
# from nav_msgs.msg import Odometry
# from std_msgs.msg import Float32
# import math
#
# class SimulatedWiFi(Node):
#     def __init__(self):
#         super().__init__('simulated_wifi')
#
#         self.declare_parameter('access_points', [
#             {'name': 'ap1', 'x': 0.0, 'y': 0.0},
#             {'name': 'ap2', 'x': 5.0, 'y': 5.0}
#         ])
#
#         self.aps = self.get_parameter('access_points').get_parameter_value().string_array_value
#         self.access_points = self._parse_ap_param(self.aps)
#         self.publishers = {}
#
#         for ap in self.access_points:
#             topic = f"/wifi/{ap['name']}/signal"
#             self.publishers[ap['name']] = self.create_publisher(Float32, topic, 10)
#             self.get_logger().info(f"WiFi AP '{ap['name']}' at ({ap['x']}, {ap['y']}) -> publishing to {topic}")
#
#         self.subscription = self.create_subscription(
#             Odometry,
#             '/a200_0000/odom',
#             self.odom_callback,
#             10
#         )
#
#     def _parse_ap_param(self, raw_array):
#         # Expecting strings like 'ap1:1.0,2.0'
#         aps = []
#         for entry in raw_array:
#             name, coords = entry.split(':')
#             x, y = map(float, coords.split(','))
#             aps.append({'name': name, 'x': x, 'y': y})
#         return aps
#
#     def odom_callback(self, msg):
#         x = msg.pose.pose.position.x
#         y = msg.pose.pose.position.y
#
#         for ap in self.access_points:
#             dist = math.sqrt((x - ap['x'])**2 + (y - ap['y'])**2)
#             signal = 1.0 / (1.0 + dist)  # simple signal decay model
#             self.publishers[ap['name']].publish(Float32(data=signal))
#
#
# def main(args=None):
#     rclpy.init(args=args)
#     node = SimulatedWiFi()
#     rclpy.spin(node)
#     node.destroy_node()
#     rclpy.shutdown()

#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped
import math

class WifiAPSpawner(Node):
    def __init__(self):
        super().__init__('wifi_ap_spawner')

        self.declare_parameter('n_anchors', 100)
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
