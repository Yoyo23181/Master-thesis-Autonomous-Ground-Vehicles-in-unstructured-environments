import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped
from geometry_msgs.msg import PointStamped
from std_msgs.msg import String
import math

class UWBProximityDetector(Node):
    def __init__(self):
        super().__init__('uwb_proximity_detector')

        self.declare_parameter('uwb_range', 5.0)
        self.uwb_range = self.get_parameter('uwb_range').get_parameter_value().double_value

        self.anchor_positions = []  # List of (x, y)
        self.robot_position = None

        # Subscriptions
        self.create_subscription(PoseWithCovarianceStamped, '/a200_0957/amcl_pose', self.pose_callback, 10)
        self.create_subscription(PointStamped, 'wifi_ap_locations', self.anchor_callback, 10)

        # Publisher
        self.detected_pub = self.create_publisher(String, 'uwb_detected_anchors', 10)

        self.timer = self.create_timer(1.0, self.check_proximity)

    def anchor_callback(self, msg):
        pos = (msg.point.x, msg.point.y)
        if pos not in self.anchor_positions:
            self.anchor_positions.append(pos)
            self.get_logger().info(f"Anchor registered at {pos}")

    def pose_callback(self, msg):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        self.robot_position = (x, y)

    def check_proximity(self):
        if self.robot_position is None or not self.anchor_positions:
            return

        detected = []
        rx, ry = self.robot_position
        for i, (ax, ay) in enumerate(self.anchor_positions):
            distance = math.hypot(ax - rx, ay - ry)
            if distance <= self.uwb_range:
                detected.append(f"AP#{i+1} ({ax:.1f}, {ay:.1f})")

        msg = String()
        if detected:
            msg.data = f"Detected by: {', '.join(detected)}"
        else:
            msg.data = "No anchors in range"

        self.detected_pub.publish(msg)
        self.get_logger().info(msg.data)

def main(args=None):
    rclpy.init(args=args)
    node = UWBProximityDetector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
