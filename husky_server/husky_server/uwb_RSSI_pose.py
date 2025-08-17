#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped, PoseWithCovarianceStamped, PoseStamped
from std_msgs.msg import String
import math
import os
from datetime import datetime
import matplotlib.pyplot as plt
import random
import csv
import numpy as np

class UWBRSSILocalizer(Node):
    def __init__(self):
        super().__init__('uwb_rssi_localizer')

        self.t0 = self.get_clock().now()
        self.err_time = []

        # Parameters
        self.declare_parameter('signal_at_1m', -40.0)
        self.declare_parameter('path_loss_exponent', 4.0)

        self.A = self.get_parameter('signal_at_1m').get_parameter_value().double_value
        self.n = self.get_parameter('path_loss_exponent').get_parameter_value().double_value

        # State
        self.anchor_positions = []  # List of (x, y)
        self.robot_position = None
        self.real_path = []
        self.estimated_path = []
        self.prev_pose = None
        self.stationary_time = 0.0
        self.stop_threshold = 0.05
        self.stop_duration = 20.0
        self.last_time = self.get_clock().now()

        # Subscribers
        self.create_subscription(PointStamped, 'wifi_ap_locations', self.anchor_callback, 10)
        self.create_subscription(PoseStamped, '/a200_0957/ground_truth_pose', self.pose_callback, 10)

        # Publishers
        self.position_pub = self.create_publisher(PoseStamped, '/a200_0957/uwb_estimated_pose', 10)
        self.anchor_log_pub = self.create_publisher(String, 'uwb_rssi_report', 10)

        # Timer
        self.timer = self.create_timer(3.0, self.localize)

        self.declare_parameter('distance_noise_std', 0.1)
        self.distance_noise_std = self.get_parameter('distance_noise_std').get_parameter_value().double_value

    def anchor_callback(self, msg):
        pos = (msg.point.x, msg.point.y)
        if pos not in self.anchor_positions:
            self.anchor_positions.append(pos)
            self.get_logger().info(f"Registered anchor at {pos}")

    def pose_callback(self, msg):
        x = msg.pose.position.x
        y = msg.pose.position.y
        self.robot_position = (x, y)

    def compute_rssi(self, distance):
        if distance < 1.0:
            distance = 1.0
        return self.A - 10 * self.n * math.log10(distance) #formula from : A Survey of Indoor Localization Systems and  Technologies by Faheem Zafari

    def rssi_to_weight(self, rssi):
        return 10 ** (rssi / 10.0)  # Convert dBm to linear scale

    def localize(self):
        if self.robot_position is None or not self.anchor_positions:
            return

        now = self.get_clock().now()
        elapsed = (now - self.last_time).nanoseconds * 1e-9
        self.last_time = now

        rx, ry = self.robot_position
        weighted_sum_x = 0.0
        weighted_sum_y = 0.0
        total_weight = 0.0
        rssi_log = {}

        for i, (ax, ay) in enumerate(self.anchor_positions):
            # dist = math.hypot(ax - rx, ay - ry)
            true_dist = math.hypot(ax - rx, ay - ry)
            # noisy_dist = true_dist + random.gauss(0, self.distance_noise_std)
            noisy_dist = true_dist
            dist = max(noisy_dist, 0.1)  # prevent zero or negative distances

            rssi = self.compute_rssi(dist) + random.gauss(0, self.distance_noise_std)
            weight = self.rssi_to_weight(rssi)
            weighted_sum_x += ax * weight
            weighted_sum_y += ay * weight
            total_weight += weight
            rssi_log[f"AP#{i+1}"] = round(rssi, 2)

        # Log RSSI values
        msg = String()
        msg.data = str(rssi_log)
        self.anchor_log_pub.publish(msg)

        if total_weight == 0:
            self.get_logger().info("No valid RSSI data to estimate pose.")
            return

        est_x = weighted_sum_x / total_weight
        est_y = weighted_sum_y / total_weight

        # Motion detection
        if self.prev_pose:
            dx = rx - self.prev_pose[0]
            dy = ry - self.prev_pose[1]
            velocity = math.hypot(dx, dy) / elapsed
        else:
            velocity = 0.0
        self.prev_pose = (rx, ry)

        if velocity >= self.stop_threshold:
            self.real_path.append((rx, ry))
            self.estimated_path.append((est_x, est_y))
            t = (now - self.t0).nanoseconds * 1e-9
            self.err_time.append(t)
            self.stationary_time = 0.0
        else:
            self.stationary_time += elapsed
            if self.stationary_time >= self.stop_duration:
                self.save_plot()
                self.stationary_time = 0.0

        # Publish estimated pose
        pose_msg = PoseStamped()
        pose_msg.header.stamp = now.to_msg()
        pose_msg.header.frame_id = 'map'
        pose_msg.pose.position.x = est_x
        pose_msg.pose.position.y = est_y
        pose_msg.pose.position.z = 0.0
        pose_msg.pose.orientation.w = 1.0
        self.position_pub.publish(pose_msg)

        self.get_logger().info(
            f"Real: ({rx:.2f}, {ry:.2f}) — Estimated: ({est_x:.2f}, {est_y:.2f})"
        )

    def save_error_data(self, errors, suffix = ""):
        # timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        filepath = f"/sharedDrive/PLOT_GRAPH/simu/{suffix}_RSSI_40_simulation.csv"

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

        errors = [math.hypot(xr - xe, yr - ye) for (xr, yr), (xe, ye) in zip(self.real_path, self.estimated_path)]

        sorted_errors = np.sort(errors)
        cdf = np.arange(1, len(sorted_errors) + 1) / len(sorted_errors)



        rmse_curve = [
            math.sqrt(sum(e**2 for e in errors[:i + 1]) / (i + 1))
            for i in range(len(errors))
        ]
        total_distance = sum(
            math.hypot(x_real[i + 1] - x_real[i], y_real[i + 1] - y_real[i])
            for i in range(len(x_real) - 1)
        )
        percent_errors = [(e / total_distance * 100) if total_distance > 0 else 0 for e in errors]

        t_errors = list(zip(self.err_time, errors))
        t_rmse = list(zip(self.err_time, rmse_curve))
        self.save_error_data(t_errors, "error")
        self.save_error_data(t_rmse, "RMSE")

        # # Plot setup
        # fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 10))

        # # Trajectory Plot
        # ax1.plot(x_real, y_real, label='Real Path', color='blue', marker='o')
        # ax1.plot(x_est, y_est, label='Estimated Path', color='red', linestyle='--', marker='x')
        # ax1.set_xlabel('X [m]')
        # ax1.set_ylabel('Y [m]')
        # ax1.set_title('UWB Estimated vs Real Path')
        # ax1.legend()
        # ax1.grid(True)

        # # RMSE and % Error Plot
        # ax2.plot(rmse_curve, label='Cumulative RMSE [m]', color='green')
        # ax2.plot(percent_errors, label='% Error (est/total dist)', color='orange', linestyle='--')
        # ax2.set_xlabel('Time Step')
        # ax2.set_ylabel('Error')
        # ax2.set_title('Error Evolution Over Time')
        # ax2.legend()
        # ax2.grid(True)

        plt.plot(figsize=(10, 5))
        # plt.plot(rmse_curve, label='Cumulative RMSE [m]', color='green')
        plt.plot(errors, label='Error [m]', color='green')
        plt.xlabel('Time Step')
        plt.ylabel('Error [m]')
        plt.title('Error Over Time')
        plt.legend()
        plt.grid(True)

        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        filepath = f"/sharedDrive/rssi/husky/rssi_plot/40_real_noise.png"
        plt.tight_layout()
        plt.savefig(filepath)
        plt.close()

        self.get_logger().info(f"Saved RSSI trajectory plot to {filepath}")

        plt.figure(figsize=(8, 5))
        plt.plot(sorted_errors, cdf, marker='o', linestyle='-', color='purple')
        plt.xlabel("Error [m]")
        plt.ylabel("Cumulative Probability")
        plt.title("Cumulative Distribution Function (CDF) of RSSI Error")
        plt.grid(True)

        cdf_filepath = f"/sharedDrive/rssi/husky/rssi_plot/cdf_40_real_noise.png"
        plt.tight_layout()
        plt.savefig(cdf_filepath)
        plt.close()

        self.get_logger().info(f"Saved CDF plot to {cdf_filepath}")


        self.real_path.clear()
        self.estimated_path.clear()


def main(args=None):
    rclpy.init(args=args)
    node = UWBRSSILocalizer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
