#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped, PoseWithCovarianceStamped, PoseStamped
from std_msgs.msg import String
import math
import os
from datetime import datetime
import matplotlib.pyplot as plt
import numpy as np
import csv
from nav_msgs.msg import Odometry
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
from std_msgs.msg import Float64MultiArray


class UWBRSSILocalizer(Node):
    def __init__(self):
        super().__init__('uwb_rssi_localizer')

        self.t0 = self.get_clock().now()
        self.err_time = []

        self.ANCHORS_XYZ = {
            0: (2.70, 4.75, 0.50),
            1: (-7.74, -8.44, 0.124),
            2: (-0.36, 6.05, 0.107),
            3: (-3.35, -4.02, 0.195),
            4: (1.70, -6.05, 0.205),
            5: (-4.87, -4.66, 0.749),
            6: (-8.38, -4.18, 0.555),
            7: (-5.50, -7.90, 0.250),
        }

        self.anchor_positions = [(self.ANCHORS_XYZ[i][0], self.ANCHORS_XYZ[i][1]) for i in range(8)]

        # Parameters
        self.declare_parameter('stale_seconds', 5.0)
        self.declare_parameter('range_epsilon', 1e-3)

        self.declare_parameter('signal_at_1m', -40.0)
        self.declare_parameter('path_loss_exponent', 4.0)

        self.stale_seconds = float(self.get_parameter('stale_seconds').value)
        self.range_epsilon = float(self.get_parameter('range_epsilon').value)

        self.A = self.get_parameter('signal_at_1m').get_parameter_value().double_value
        self.n = self.get_parameter('path_loss_exponent').get_parameter_value().double_value

        # State
        self.robot_position = None
        self.real_path = []
        self.estimated_path = []
        self.prev_pose = None
        self.stationary_time = 0.0
        self.stop_threshold = 0.05
        self.stop_duration = 3.0
        self.start_time = self.get_clock().now()
        self.has_saved_plot = False

        qos = QoSProfile(depth=10)
        qos.reliability = QoSReliabilityPolicy.BEST_EFFORT

        # Subscribers
        self.create_subscription(Float64MultiArray, '/anchors_ranges', self.anchor_callback, qos)
        self.create_subscription(PoseWithCovarianceStamped, '/tb3/amcl_pose', self.pose_callback, qos)


        # Publishers
        self.position_pub = self.create_publisher(PoseStamped, 'uwb_estimated_pose', 10)
        self.anchor_log_pub = self.create_publisher(String, 'uwb_rssi_report', 10)

        # Timer
        self.timer = self.create_timer(1.0, self.localize)


        self.pose_received = False

        self.latest_ranges = [math.nan] * 8
        self.last_range = [math.nan] * 8
        self.last_change_time = [None] * 8

    def anchor_callback(self, msg):
        data = list(msg.data)
        # if len(data) < 8:
        #     return
        stamp_sec = float(data[8]) if len(data) >= 9 else (self.get_clock().now().nanoseconds * 1e-9)

        for i in range(8):
            r = float(data[i])

            if not math.isfinite(r) or r <= 0.0:
                self.latest_ranges[i] = math.nan
                continue

            prev = self.last_range[i]

            changed = (not math.isfinite(prev)) or (abs(r - prev) > self.range_epsilon)

            if changed:
                self.last_range[i] = r
                self.last_change_time[i] = stamp_sec
                self.latest_ranges[i] = r

            else:
                t0 = self.last_change_time[i]
                is_stale = (t0 is not None) and ((stamp_sec - t0) > self.stale_seconds)
                self.latest_ranges[i] = (math.nan if is_stale else r)




    def pose_callback(self, msg):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        self.robot_position = (x, y)

        if self.robot_position is None:
            self.get_logger().info("Robot position unavailable")

        if not self.pose_received:
            self.pose_received = True
            self.get_logger().info("Pose received")

    def compute_rssi(self, distance):
        if distance < 1.0:
            distance = 1.0
        return self.A - 10 * self.n * math.log10(distance) #formula from : A Survey of Indoor Localization Systems and  Technologies by Faheem Zafari

    def rssi_to_weight(self, rssi):
        return 10 ** (rssi / 10.0)  # Convert dBm to linear scale

    def localize(self):
        print(self.robot_position)
        if self.robot_position is None:
            return
        print("we have")
        now = self.get_clock().now()
        self.last_time = now

        rx, ry = self.robot_position
        weighted_sum_x = 0.0
        weighted_sum_y = 0.0
        total_weight = 0.0
        rssi_log = {}

        for i, (ax, ay) in enumerate(self.anchor_positions):
            r = self.latest_ranges[i]
            if not math.isfinite(r) or r <= 0.0:
                continue


            dist = self.latest_ranges[i]

            rssi = self.compute_rssi(dist)
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

        self.real_path.append((rx,ry))
        self.estimated_path.append((est_x, est_y))
        t = (now - self.t0).nanoseconds * 1e-9
        self.err_time.append(t)
        total_runtime = (self.get_clock().now() - self.start_time).nanoseconds * 1e-9
        print(total_runtime)

        if total_runtime > 900.0 and not self.has_saved_plot:
            self.save_plot()
            self.has_saved_plot = True



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

    def save_error_data(self, errors, suffix=""):
        # timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        filepath = f"/sharedDrive/PLOT_GRAPH/Real/{suffix}_real_experiment_RSSI.csv"

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
            math.sqrt(sum(e ** 2 for e in errors[:i + 1]) / (i + 1))
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

        plt.plot(figsize=(10, 5))
        plt.plot(errors, label='Error [m]', color='green')
        plt.xlabel('Time Step')
        plt.ylabel('Error [m]')
        plt.title('Error Over Time')
        plt.legend()
        plt.grid(True)

        filepath = f"/sharedDrive/rssi/tb4/rssi_plot/real.png"
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

        cdf_filepath = f"/sharedDrive/rssi/tb4/rssi_plot/cdf_real.png"
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






