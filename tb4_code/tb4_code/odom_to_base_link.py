#!/usr/bin/env python3

import rclpy
# from gymnasium.envs.registration import namespace
from rclpy.node import Node
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped
from rclpy.qos import qos_profile_sensor_data

class OdomToBaseLink(Node):
    def __init__(self):
        super().__init__('odom_to_base_link')
        # super().__init__('odom_to_base_link', namespace='a200_0957')

        self.odom_sub = self.create_subscription(Odometry, '/tb3/odom', self.odom_callback, qos_profile_sensor_data)

        self.tf_broadcaster = TransformBroadcaster(self)

    def odom_callback(self, msg):

        t = TransformStamped()

        # t.header.stamp = self.get_clock().now().to_msg()
        t.header.stamp = msg.header.stamp
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'
        t.transform.translation.x = msg.pose.pose.position.x
        t.transform.translation.y = msg.pose.pose.position.y
        t.transform.translation.z = msg.pose.pose.position.z
        t.transform.rotation.x = msg.pose.pose.orientation.x
        t.transform.rotation.y = msg.pose.pose.orientation.y
        t.transform.rotation.z = msg.pose.pose.orientation.z
        t.transform.rotation.w = msg.pose.pose.orientation.w

        self.tf_broadcaster.sendTransform(t)
        self.get_logger().info('Transform from odom to base_link published.')
def main(args=None):
    rclpy.init(args=args)
    odom_to_tf = OdomToBaseLink()
    rclpy.spin(odom_to_tf)
    odom_to_tf.destroy_node()
    rclpy.shutdown()
if __name__ == '__main__':
    main()

