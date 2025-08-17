
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped
from std_msgs.msg import String
from datetime import datetime
import matplotlib.pyplot as plt
import math
import os
import numpy as np
import csv
from nav_msgs.msg import Odometry


class ODOMvsAMCLMonitor(Node):
    def __init__(self):
        super().__init__('odom_vs_amcl_monitor')

        self.t0 = self.get_clock().now()
        self.err_time = []

        # Paths
        self.real_path = []
        self.estimated_path = []

        # Motion detection
        self.prev_pose = None
        self.stationary_time = 0.0
        self.stop_threshold = 0.05  # m/s
        self.stop_duration = 20.0    # seconds
        self.last_time = self.get_clock().now()

        # Subscriptions
        self.create_subscription(PoseStamped, '/a200_0957/ground_truth_pose', self.robot_callback, 10)
        self.create_subscription(Odometry, '/a200_0957/platform/odom', self.odom_callback, 10)

        self.latest_amcl = None
        self.latest_odom = None

        # Timer to evaluate and log
        self.timer = self.create_timer(1.0, self.evaluate)

        self.odom_noise_std = 0.00

    def robot_callback(self, msg):
        x = msg.pose.position.x
        y = msg.pose.position.y
        self.latest_amcl = (x, y)

    def odom_callback(self, msg):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y

        if abs(x) > 2 or abs(y) > 2:
            x = x + 1.85
            y = y + 3.95


        self.latest_odom = (x, y)


    def evaluate(self):

        if self.latest_amcl is None or self.latest_odom is None:
            return

        now = self.get_clock().now()
        elapsed = (now - self.last_time).nanoseconds * 1e-9
        self.last_time = now

        rx, ry = self.latest_amcl
        sx, sy = self.latest_odom
        print(self.last_time)
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

        self.get_logger().info(f"Real: ({rx:.2f}, {ry:.2f}) — odom: ({sx:.2f}, {sy:.2f})")

    def save_error_data(self, errors, suffix = ""):
        # timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        filepath = f"/sharedDrive/PLOT_GRAPH/simu/{suffix}_odom_simulation.csv"

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

        plt.tight_layout()

        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        # filepath = f"/sharedDrive/odom/husky/odom_plot/{timestamp}.png"
        filepath = f"/sharedDrive/odom/husky/odom_plot/real_noise.png"
        plt.savefig(filepath)
        plt.close()

        self.get_logger().info(f"Saved error plot to {filepath}")

        plt.figure(figsize=(8, 5))
        plt.plot(sorted_errors, cdf, marker='o', linestyle='-', color='purple')
        plt.xlabel("Error [m]")
        plt.ylabel("Cumulative Probability")
        plt.title("Cumulative Distribution Function (CDF) of odom Error")
        plt.grid(True)

        # Save the CDF plot
        cdf_filepath = f"/sharedDrive/odom/husky/odom_plot/cdf_real_noise.png"
        plt.tight_layout()
        plt.savefig(cdf_filepath)
        plt.close()

        self.get_logger().info(f"Saved CDF plot to {cdf_filepath}")

        self.real_path.clear()
        self.estimated_path.clear()


def main(args=None):
    rclpy.init(args=args)
    node = ODOMvsAMCLMonitor()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
