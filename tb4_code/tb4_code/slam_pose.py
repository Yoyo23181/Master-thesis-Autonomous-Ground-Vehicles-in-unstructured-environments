#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from datetime import datetime
import matplotlib.pyplot as plt
import math
import os
import numpy as np
import csv
import math
import numpy as np


class SLAMvsAMCLMonitor(Node):
    def __init__(self):
        super().__init__('slam_vs_amcl_monitor')

        self.t0 = self.get_clock().now()
        self.err_time = []

        # Paths
        self.real_path = []
        self.estimated_path = []

        # Motion detection
        self.prev_pose = None
        self.stationary_time = 0.0
        self.stop_threshold = 0.05  # m/s
        self.stop_duration = 30.0    # seconds
        self.last_time = self.get_clock().now()

        # Subscriptions
        self.create_subscription(PoseWithCovarianceStamped, '/tb3/amcl_pose', self.robot_callback, 10)
        self.create_subscription(Odometry, '/vio', self.slam_callback, qos_profile_sensor_data)

        self.latest_amcl = None
        self.latest_slam = None

        # Timer to evaluate and log
        self.timer = self.create_timer(1.0, self.evaluate)

        self.slam_noise_std = 0.0

        self.declare_parameter('vio_offset_x', 1.8)
        # self.declare_parameter('vio_offset_x', 2.19)
        self.declare_parameter('vio_offset_y', 2.17)
        # self.declare_parameter('vio_offset_y', -1.81)
        self.declare_parameter('vio_deg_yaw', 90.0)
        self.vio_dx = float(self.get_parameter('vio_offset_x').value)
        self.vio_dy = float(self.get_parameter('vio_offset_y').value)
        self.vio_deg_yaw = self.get_parameter('vio_deg_yaw').value
        self.vio_theta = math.radians(self.vio_deg_yaw)
        self.vio_cos = math.cos(self.vio_theta)
        self.vio_sin = math.sin(self.vio_theta)

    def robot_callback(self, msg):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        self.latest_amcl = (x, y)

    def slam_callback(self, msg):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y

        x_slam = self.vio_cos * x - self.vio_sin * y + self.vio_dx
        y_slam = self.vio_sin * x + self.vio_cos * y + self.vio_dy

        # x_noisy = x + np.random.normal(0, self.slam_noise_std)
        # y_noisy = y + np.random.normal(0, self.slam_noise_std)

        # self.latest_slam = (x_noisy, y_noisy)

        self.latest_slam = (x_slam, y_slam)

    def evaluate(self):
        if self.latest_amcl is None or self.latest_slam is None:
            return

        now = self.get_clock().now()
        elapsed = (now - self.last_time).nanoseconds * 1e-9
        self.last_time = now

        rx, ry = self.latest_amcl
        sx, sy = self.latest_slam

        # sx = sx + np.random.gauss(0, self.noise_std)
        # sy = sy + np.random.gauss(0, self.noise_std)

        # Motion detection based on AMCL pose
        if self.prev_pose:
            dx = rx - self.prev_pose[0]
            dy = ry - self.prev_pose[1]
            velocity = math.hypot(dx, dy) / elapsed
        else:
            velocity = 0.0
        self.prev_pose = (rx, ry)

        if velocity >= self.stop_threshold:
            self.real_path.append((rx, ry))
            self.estimated_path.append((sx, sy))
            t = (now - self.t0).nanoseconds * 1e-9
            self.err_time.append(t)
            self.stationary_time = 0.0
        else:
            self.stationary_time += elapsed
            if self.stationary_time >= self.stop_duration:
                self.save_plot()
                self.stationary_time = 0.0

        self.get_logger().info(f"Real: ({rx:.2f}, {ry:.2f}) — SLAM: ({sx:.2f}, {sy:.2f})")

    def save_error_data(self, errors, suffix=""):
        # timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        filepath = f"/sharedDrive/PLOT_GRAPH/Real/{suffix}_real_experiment_slam.csv"

        with open(filepath, mode='w', newline='') as file:
            writer = csv.writer(file)
            writer.writerow(["Step", "Error [m]"])
            for t, e in errors:
                writer.writerow([f"{t:.6f}", f"{e:.6f}"])

        self.get_logger().info(f"Saved error log to {filepath}")

    def save_plot(self):
        if not self.real_path or not self.estimated_path:
            return

        x_real, y_real = zip(*self.real_path)
        x_est, y_est = zip(*self.estimated_path)

        # Error calculation
        errors = [math.hypot(xr - xe, yr - ye) for (xr, yr), (xe, ye) in zip(self.real_path, self.estimated_path)]

        sorted_errors = np.sort(errors)
        cdf = np.arange(1, len(sorted_errors) + 1) / len(sorted_errors)



        rmse_curve = [math.sqrt(sum(e**2 for e in errors[:i+1]) / (i+1)) for i in range(len(errors))]
        total_distance = sum(math.hypot(x_real[i+1] - x_real[i], y_real[i+1] - y_real[i]) for i in range(len(x_real) - 1))
        percent_errors = [(e / total_distance * 100) if total_distance > 0 else 0 for e in errors]

        t_errors = list(zip(self.err_time, errors))
        t_rmse = list(zip(self.err_time, rmse_curve))
        self.save_error_data(t_errors, "error")
        self.save_error_data(t_rmse, "RMSE")

        plt.plot(figsize=(10, 5))
        plt.plot(errors, label='Error [m]', color='green')
        plt.xlabel('Time Step')
        plt.ylabel('Error [m]')
        plt.title('Error Over Time')
        plt.legend()
        plt.grid(True)

        plt.tight_layout()

        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        filepath = f"/sharedDrive/slam/tb4/slam_plot/real_experiment.png"
        plt.savefig(filepath)
        plt.close()

        self.get_logger().info(f"Saved error plot to {filepath}")

        plt.figure(figsize=(8, 5))
        plt.plot(sorted_errors, cdf, marker='o', linestyle='-', color='purple')
        plt.xlabel("Error [m]")
        plt.ylabel("Cumulative Probability")
        plt.title("Cumulative Distribution Function (CDF) of SLAM Error")
        plt.grid(True)

        # Save the CDF plot
        cdf_filepath = f"/sharedDrive/slam/tb4/slam_plot/real_experiment_cdf.png"
        plt.tight_layout()
        plt.savefig(cdf_filepath)
        plt.close()

        self.get_logger().info(f"Saved error plot to {cdf_filepath}")


        self.real_path.clear()
        self.estimated_path.clear()


def main(args=None):
    rclpy.init(args=args)
    node = SLAMvsAMCLMonitor()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()