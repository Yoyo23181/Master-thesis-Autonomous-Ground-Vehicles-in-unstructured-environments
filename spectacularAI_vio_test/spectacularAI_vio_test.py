#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import depthai
import spectacularAI
import time
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import matplotlib
from matplotlib import interactive
interactive(True)

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry


class VIO(Node):
    def __init__(self):
        super().__init__('spectacularAI_vio_node')
        self.pipeline = depthai.Pipeline()
        self.vio_pipeline = spectacularAI.depthai.Pipeline(self.pipeline)
        self.vio_pub = self.create_publisher(Odometry, 'vio', 10)
        self.device = depthai.Device(self.pipeline)
        self.vio_session = self.vio_pipeline.startSession(self.device)
        self.timer = self.create_timer(0, self.processOutput)

    def processOutput(self):
        while self.vio_session.hasOutput():
            out = self.vio_session.getOutput()
            vel = np.array([out.velocity.x, out.velocity.y, out.velocity.z])
            angul_vel = np.array([out.angularVelocity.x, out.angularVelocity.y, out.angularVelocity.z])
            pose = np.array([out.pose.position.x, out.pose.position.y, out.pose.position.z])
            orientation = np.array([out.pose.orientation.x, out.pose.orientation.y, out.pose.orientation.z, out.pose.orientation.w])
            self.publish_vio(vel, angul_vel, pose, orientation)

    def publish_vio(self, vel, angul_vel, pose, orientation):
        msg = Odometry()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "map"
        msg.child_frame_id = "base_link"
        msg.twist.twist.linear.x = vel[0]
        msg.twist.twist.linear.y = vel[1]
        msg.twist.twist.linear.z = vel[2]
        msg.twist.twist.angular.x = angul_vel[0]
        msg.twist.twist.angular.y = angul_vel[1]
        msg.twist.twist.angular.z = angul_vel[2]
        msg.pose.pose.position.x = pose[0]
        msg.pose.pose.position.y = pose[1]
        msg.pose.pose.position.z = pose[2]
        msg.pose.pose.orientation.x = orientation[0]
        msg.pose.pose.orientation.y = orientation[1]
        msg.pose.pose.orientation.z = orientation[2]
        msg.pose.pose.orientation.w = orientation[3]
        self.vio_pub.publish(msg)



def main(args=None):
    rclpy.init(args=args)
    vio = VIO()
    rclpy.spin(vio)
    vio.destroy_node()
    rclpy.shutdown()

