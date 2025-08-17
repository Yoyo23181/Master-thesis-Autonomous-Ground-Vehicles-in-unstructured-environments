import rclpy
from rclpy.node import Node
from tf2_msgs.msg import TFMessage
from geometry_msgs.msg import PoseStamped

class GroundTruthPosePublisher(Node):
    def __init__(self):
        super().__init__('ground_truth_pose_publisher')
        self.subscription = self.create_subscription(
            TFMessage,
            '/model/a200_0957/robot/pose',
            self.tf_callback,
            10
        )
        self.publisher = self.create_publisher(
            PoseStamped,
            '/a200_0957/ground_truth_pose',
            10
        )

    def tf_callback(self, msg: TFMessage):
        for transform in msg.transforms:
            if transform.child_frame_id == 'a200_0957/robot':
                pose_msg = PoseStamped()
                pose_msg.header = transform.header

                # Copy translation (Vector3) to Point manually
                pose_msg.pose.position.x = transform.transform.translation.x
                pose_msg.pose.position.y = transform.transform.translation.y
                pose_msg.pose.position.z = transform.transform.translation.z

                # Orientation is already a Quaternion, assign directly
                pose_msg.pose.orientation = transform.transform.rotation

                self.publisher.publish(pose_msg)
                self.get_logger().info(
                    f'Published GT pose: x={pose_msg.pose.position.x:.2f}, y={pose_msg.pose.position.y:.2f}')
                break


def main(args=None):
    rclpy.init(args=args)
    node = GroundTruthPosePublisher()
    rclpy.spin(node)
    rclpy.shutdown()
