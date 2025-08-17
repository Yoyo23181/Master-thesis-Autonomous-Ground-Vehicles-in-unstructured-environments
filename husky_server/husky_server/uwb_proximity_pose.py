#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped, PoseWithCovarianceStamped, PoseStamped
from std_msgs.msg import String
import math
import os
from datetime import datetime
import matplotlib.pyplot as plt
import csv
import numpy as np

class UWBProximityLocalizer(Node):
    def __init__(self):
        super().__init__('uwb_proximity_localizer')

        self.t0 = self.get_clock().now()
        self.err_time = []

        # Declare parameters
        self.declare_parameter('uwb_range', 30.0)
        self.uwb_range = self.get_parameter('uwb_range').get_parameter_value().double_value

        # Internal state
        self.anchor_positions = []  # List of (x, y)
        self.robot_position = None  # (x, y)

        # Subscribers
        # self.create_subscription(PoseWithCovarianceStamped, '/a200_0957/amcl_pose', self.pose_callback, 10)
        self.create_subscription(PoseStamped, '/a200_0957/ground_truth_pose', self.pose_callback, 10)
        self.create_subscription(PointStamped, 'wifi_ap_locations', self.anchor_callback, 10)

        # Publishers
        self.closest_pub = self.create_publisher(String, 'uwb_detected_anchors', 10)
        self.position_pub = self.create_publisher(PoseStamped, '/a200_0957/uwb_estimated_pose_proximity', 10)

        # Check proximity every second
        self.timer = self.create_timer(1.0, self.check_proximity)

        self.real_path = []
        self.estimated_path = []
        self.prev_pose = None
        self.stationary_time = 0.0
        self.stop_threshold = 0.05  # m/s
        self.stop_duration = 20.0  # seconds
        self.last_time = self.get_clock().now()

    def anchor_callback(self, msg):
        pos = (msg.point.x, msg.point.y)
        if pos not in self.anchor_positions:
            self.anchor_positions.append(pos)
            self.get_logger().info(f"Registered AP at {pos}")

    def pose_callback(self, msg):
        x = msg.pose.position.x
        y = msg.pose.position.y
        self.robot_position = (x, y)

    def check_proximity(self):
        if self.robot_position is None or not self.anchor_positions:
            self.get_logger().info("No anchors or robot detected")
            return

        now = self.get_clock().now()
        elapsed = (now - self.last_time).nanoseconds * 1e-9
        self.last_time = now

        rx, ry = self.robot_position
        detected = []
        for i, (ax, ay) in enumerate(self.anchor_positions):
            distance = math.hypot(ax - rx, ay - ry)
            if distance <= self.uwb_range:
                detected.append((ax, ay))

        if detected:
            # Estimate position as centroid
            avg_x = sum(p[0] for p in detected) / len(detected)
            avg_y = sum(p[1] for p in detected) / len(detected)

            # Velocity check
            if self.prev_pose:
                dx = rx - self.prev_pose[0]
                dy = ry - self.prev_pose[1]
                velocity = math.hypot(dx, dy) / elapsed
            else:
                velocity = 0.0
            self.prev_pose = (rx, ry)

            if velocity >= self.stop_threshold:
                # Only record while robot is moving
                self.real_path.append((rx, ry))
                self.estimated_path.append((avg_x, avg_y))
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
            pose_msg.pose.position.x = avg_x
            pose_msg.pose.position.y = avg_y
            pose_msg.pose.position.z = 0.0
            pose_msg.pose.orientation.w = 1.0
            self.position_pub.publish(pose_msg)

            # Logging
            self.get_logger().info(
                f"Real pose: ({rx:.2f}, {ry:.2f}) — Estimated: ({avg_x:.2f}, {avg_y:.2f})"
            )

            # Anchor report
            msg = String()
            msg.data = f"Detected by {len(detected)} APs"
            self.closest_pub.publish(msg)
        else:
            self.get_logger().info("No APs in range — can't estimate position")

    def save_error_data(self, errors, suffix = ""):
        # timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        filepath = f"/sharedDrive/PLOT_GRAPH/simu/{suffix}_100_proximity_simulation_range_reduced.csv"

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

        # Compute per-step Euclidean errors
        errors = [math.hypot(xr - xe, yr - ye) for (xr, yr), (xe, ye) in zip(self.real_path, self.estimated_path)]

        sorted_errors = np.sort(errors)
        cdf = np.arange(1, len(sorted_errors) + 1) / len(sorted_errors)


        # Compute RMSE curve (cumulative RMSE at each point)
        rmse_curve = []
        for i in range(1, len(errors) + 1):
            cumulative_rmse = math.sqrt(sum(e ** 2 for e in errors[:i]) / i)
            rmse_curve.append(cumulative_rmse)

        # Optional: % error (normalized to path length)
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

        # Save
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        # filepath = f"/sharedDrive/proximity/husky/proximity_plot/{timestamp}.png"
        filepath = f"/sharedDrive/proximity/husky/proximity_plot/100_anchors.png"
        plt.tight_layout()
        plt.savefig(filepath)
        plt.close()

        plt.figure(figsize=(8, 5))
        plt.plot(sorted_errors, cdf, marker='o', linestyle='-', color='purple')
        plt.xlabel("Error [m]")
        plt.ylabel("Cumulative Probability")
        plt.title("Cumulative Distribution Function (CDF) of proximity Error")
        plt.grid(True)

        # cdf_filepath = f"/sharedDrive/proximity/husky/proximity_plot/cdf_{datetime.now().strftime('%Y%m%d_%H%M%S')}.png"
        cdf_filepath = f"/sharedDrive/proximity/husky/proximity_plot/cdf_100_anchors.png"
        plt.tight_layout()
        plt.savefig(cdf_filepath)
        plt.close()

        final_rmse = rmse_curve[-1]
        self.get_logger().info(f"Saved full plot to {filepath}")
        self.get_logger().info(f"Final RMSE: {final_rmse:.2f} m")

        # Clear data
        self.real_path.clear()
        self.estimated_path.clear()


def main(args=None):
    rclpy.init(args=args)
    node = UWBProximityLocalizer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
