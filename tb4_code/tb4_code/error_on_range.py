import numpy as np
from geometry_msgs.msg import PointStamped, PoseWithCovarianceStamped
import rclpy
from rclpy.node import Node
from datetime import datetime
import math
import matplotlib.pyplot as plt
import csv
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
from std_msgs.msg import Float64MultiArray


class UWBSetup():
    def __init__(self, rangeDif=-1):
        self.UWBAnchors = []
        self.rangeDif = rangeDif
        self.ranges = {}
        self.singleValue = True

    def addUWBAnchors(self, anchors):
        for identifier in anchors:
            self.addUWBAnchor(identifier, anchors[identifier])
        self.A = self.calculateSL1_AMatrix()
        self.AtA = np.matmul(np.transpose(self.A), self.A)

    def calculateSL1_AMatrix(self):
        A = np.empty((0, 2))  # 2D version
        for i in range(len(self.UWBAnchors)):
            for j in range(len(self.UWBAnchors) - 1 - i):
                Arow = -2 * (self.UWBAnchors[i].pose - self.UWBAnchors[len(self.UWBAnchors) - 1 - j].pose)
                A = np.append(A, [Arow], axis=0)
        return A

    def printMatrixInformation(self):
        print(f"A = {self.A}")
        print(f"At = {np.transpose(self.A)}")
        print(f"At*A = {self.AtA}")
        print(f"inv(At*A) = {np.linalg.inv(self.AtA)}")

    def addUWBAnchor(self, identifier, pose):
        self.UWBAnchors.append(UWBAnchor(identifier, pose[:2], self.rangeDif))  # Only (x, y)

    def getUWBIdentifier(self, identifier):
        for uwbA in self.UWBAnchors:
            if uwbA.identifier == identifier:
                return uwbA
        return None

    def readUWBInput(self, anchorId, time, distance):
        anchor = self.getUWBIdentifier(anchorId)
        if anchor is not None:
            anchor.addSample(distance)
            self.ranges[anchorId] = [time, distance]
            return self.calculatePoseLS1()

    def calculatePoseLS1(self):
        if self.singleValue and len(self.ranges) == len(self.UWBAnchors):
            B = self.calculateLS1_BVector()
            AtB = np.matmul(np.transpose(self.A), B)
            X = np.linalg.solve(self.AtA, AtB)
            return X
        return None

    def calculateLS1_BVector(self):
        B = np.empty((0,))
        for i in range(len(self.UWBAnchors)):
            for j in range(len(self.UWBAnchors) - 1 - i):
                diff = self.UWBAnchors[i].RangePoses[-1] - self.UWBAnchors[len(self.UWBAnchors) - 1 - j].RangePoses[-1]
                B = np.append(B, [diff], axis=0)
        return B


class UWBAnchor:
    def __init__(self, identifier, pose, rangedif=-1):
        self.identifier = identifier
        self.pose = np.array(pose)  # Only (x, y)
        self.ranges = []
        self.rangedif = rangedif
        self.RangePoses = []
        self.ROrigine = np.square(np.linalg.norm(self.pose))
        self.errors = []

    def addSample(self, ranges):
        if len(self.ranges) == 0 or self.rangedif == -1 or abs(ranges - self.ranges[-1]) < self.rangedif:
            self.ranges.append(ranges)
        else:
            self.ranges.append(self.ranges[-1])
        self.RangePoses.append(np.square(self.ranges[-1]) - self.ROrigine)

class Variance_NLOS_Tracking:
    def __init__(self, history= 100, threshold =0.03):
        self.history = history
        self.threshold = threshold
        self.los = []
        self.measruments = []
    def new_measurement(self, measurement):
        self.measruments.append(measurement)
        if len(self.measruments) > self.history:
            self.measruments.pop(0)

        var = np.var(self.measruments)
        if var < self.threshold:
            self.los.append(True)
            # self.measruments.pop(-1)
        else:
            self.los.append(False)
        return self.los[-1]
class UWBLocalizer(Node):
    def __init__(self):
        super().__init__('uwb_localizer')

        self.robot_position = [None, None, None]
        self.has_saved_plot = False

        self.anchors = {
            "ap_0": (2700.0, 4750.0, 50.0),     #4730
            "ap_1": (-7740.0, -8440.0, 12.4),
            "ap_2": (-360.0, 6050.0, 10.7),
            "ap_3": (-3350.0, -4020.0, 19.5),
            "ap_4": (1700.0, -6050.0, 20.5),
            "ap_5": (-4870.0, -4660.0, 74.9),
            "ap_6": (-8380.0, -4180.0, 55.5),
            "ap_7": (-5500.0, -7900.0, 25.0),
        }

        self.time_series = []
        self.anchor_distances_measured = [[] for _ in range(8)]
        self.anchor_distances_expected = [[] for _ in range(8)]

        qos = QoSProfile(depth=10, reliability=QoSReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(PoseWithCovarianceStamped, '/tb3/amcl_pose', self.pose_callback, qos)
        self.create_subscription(Float64MultiArray, '/anchors_ranges', self.anchors_callback, 10)

        self.uwb = UWBSetup()
        self.start_time = self.get_clock().now()

        self.nlos_trackers = {ap_id: Variance_NLOS_Tracking()
                              for ap_id in self.anchors.keys()}

        self.anchor_times = [[] for _ in range(8)]

        self.last_valid_dist = [None] * 8
        self.min_change = 1e-3

    def pose_callback(self, msg):
        self.robot_position = [msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z]

    def anchors_callback(self, msg):
        dist = list(msg.data[:8])
        time_sec = float(msg.data[8])

        if None in self.robot_position:
            return

        self.time_series.append(time_sec)


        for idx, (anchor_id, anchor_pose) in enumerate(self.anchors.items()):
            current_dist = dist[idx]

            if not math.isfinite(current_dist):
                self.last_valid_dist[idx] = None
                continue

            prev = self.last_valid_dist[idx]
            if prev is not None and math.isclose(current_dist, prev, rel_tol=0.0, abs_tol=self.min_change):
                continue

            self.last_valid_dist[idx] = current_dist


            anchor_pos = np.array(anchor_pose) / 1000.0
            robot_pos = np.array(self.robot_position)

            dz = anchor_pos[2] - robot_pos[2]
            raw_measured = dist[idx]

            if not math.isfinite(raw_measured):
                continue

            if raw_measured > abs(dz):
                measured_2d = math.sqrt(raw_measured**2 - dz**2)
            else:
                measured_2d = raw_measured

            expected_2d = np.linalg.norm(anchor_pos[:2] - robot_pos[:2])

            is_los = self.nlos_trackers[anchor_id].new_measurement(measured_2d)
            if not is_los:
                continue

            self.anchor_distances_measured[idx].append(measured_2d)
            self.anchor_distances_expected[idx].append(expected_2d)
            self.anchor_times[idx].append(time_sec)

        elapsed = (self.get_clock().now() - self.start_time).nanoseconds / 1e9
        print(elapsed)
        if not self.has_saved_plot and elapsed > 800.0:
            self.plot_distances()
            self.has_saved_plot = True

    def plot_distances(self):
        if len(self.time_series) < 2:
            return

        for i, anchor_id in enumerate(self.anchors.keys()):
            plt.figure(figsize=(10, 5))
            plt.plot(self.anchor_times[i], self.anchor_distances_measured[i], label='Measured')
            plt.plot(self.anchor_times[i], self.anchor_distances_expected[i], label='Expected')
            plt.title(f"Anchor {anchor_id} 2D Distance Over Time")
            plt.xlabel("Time [s]")
            plt.ylabel("2D Distance [m]")
            plt.legend()
            plt.grid(True)
            plt.tight_layout()
            plot_path = f"/sharedDrive/ranging/tb4/ranging_plot/distance_vs_expected/result_{anchor_id}.png"
            plt.savefig(plot_path)
            self.get_logger().info(f"Saved histogram to {plot_path}")
            plt.close()

            errors = np.array(self.anchor_distances_measured[i]) - np.array(self.anchor_distances_expected[i])
            plt.figure(figsize=(8, 5))
            plt.hist(errors, bins=50, color='skyblue', edgecolor='black')
            plt.title(f"Error Histogram for {anchor_id}")
            plt.xlabel("Error [m]")
            plt.ylabel("Frequency")
            plt.grid(True)
            plt.tight_layout()
            hist_path = f"/sharedDrive/ranging/tb4/ranging_plot/error_histogram/result_{anchor_id}.png"
            plt.savefig(hist_path)
            self.get_logger().info(f"Saved histogram to {hist_path}")
            plt.close()


def main(args=None):
    rclpy.init(args=args)
    node = UWBLocalizer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()



# import numpy as np
# from geometry_msgs.msg import PointStamped, PoseWithCovarianceStamped, PoseStamped
# import rclpy
# from rclpy.node import Node
# from datetime import datetime
# import math
# import matplotlib.pyplot as plt
# import numpy as np
# import csv
# from nav_msgs.msg import Odometry
# from rclpy.qos import QoSProfile, QoSReliabilityPolicy
# from std_msgs.msg import Float64MultiArray
#
#
# # https://www.researchgate.net/publication/224222701_Static_positioning_using_UWB_range_measurements
# class UWBSetup():
#     def __init__(self, rangeDif=-1):
#         self.UWBAnchors = []
#         self.rangeDif = rangeDif
#
#         self.ranges = {}
#         self.singleValue = True
#
#     def addUWBAnchors(self, anchors):
#
#         for identifier in anchors:
#             self.addUWBAnchor(identifier, anchors[identifier])
#
#             # form https://www.researchgate.net/publication/224222701_Static_positioning_using_UWB_range_measurements/link/0fcfd51071298bfea5000000/download
#         self.A = self.calculateSL1_AMatrix()
#         self.AtA = np.matmul(np.transpose(self.A), self.A)
#
#     def calculateSL1_AMatrix(self):
#         A = np.empty((0, 3))
#         for i in range(len(self.UWBAnchors)):
#             for j in range(len(self.UWBAnchors) - 1 - i):
#                 Arow = -2 * (self.UWBAnchors[i].pose - self.UWBAnchors[len(self.UWBAnchors) - 1 - j].pose)
#                 A = np.append(A, [Arow], axis=0)
#         return A
#
#     def printMatrixInformation(self):
#         print(f"A = {self.A}")
#         print(f"At = {np.transpose(self.A)}")
#         print(f"At*A = {self.AtA}")
#         print(f"inv(At*A) = {np.linalg.inv(self.AtA)}")
#
#     def addUWBAnchor(self, identifier, pose):
#         self.UWBAnchors.append(UWBAnchor(identifier, pose, self.rangeDif))
#
#     def getUWBIdentifier(self, identifier):
#         for uwbA in self.UWBAnchors:
#             if uwbA.identifier == identifier:
#                 return uwbA
#         return None
#
#     def readUWBInput(self, anchorId, time, distance):
#         anchor = self.getUWBIdentifier(anchorId)
#         if anchor != None:
#             anchor.addSample(distance)
#             self.ranges[anchorId] = [time, distance]
#             return self.calculatePoseLS1()
#
#     def calculatePoseLS1(self):
#         if self.singleValue and len(self.ranges) == len(self.UWBAnchors):
#             B = self.calculateLS1_BVector()
#             AtB = np.matmul(np.transpose(self.A), B)
#             X = np.linalg.solve(self.AtA, AtB)
#             return X
#         return None
#
#     def calculateLS1_BVector(self):
#         B = np.empty((0,))
#         for i in range(len(self.UWBAnchors)):
#             for j in range(len(self.UWBAnchors) - 1 - i):
#                 diff = self.UWBAnchors[i].RangePoses[-1] - self.UWBAnchors[len(self.UWBAnchors) - 1 - j].RangePoses[-1]
#                 B = np.append(B, [diff], axis=0)
#         return B
#
#     def estimateUncertainty(self, sigma):
#         if self.AtA is None:
#             return None
#         scale = 8 * (6.7 * 2) * (sigma * 2)
#         Cov_X = scale * np.linalg.inv(self.AtA * 1e-6)  # scale the covariance matrix
#         std_devs = np.sqrt(np.diag(Cov_X))
#
#         # Cov_X = sigma ** 2 * np.linalg.inv(self.AtA*1e-6)
#         std_devs = np.sqrt(np.diag(Cov_X))
#         print(np.linalg.norm(std_devs))
#         print(np.linalg.norm(std_devs[:2]))
#         return std_devs  # returns uncertainties in [x, y, z]
#
#
# class UWBAnchor:
#     def __init__(self, identifier, pose, rangedif=-1):
#         self.identifier = identifier
#         self.pose = np.array(pose)
#         self.ranges = []
#
#         self.rangedif = rangedif
#
#         self.RangePoses = []
#         self.ROrigine = np.square(np.linalg.norm(self.pose))
#         self.errors = []
#
#     def addSample(self, ranges):
#         # limits speed of system to {rangedif} m * f (10 hz)
#         # not sure if this ok -> To much trust on UWB measurements... ?
#         if len(self.ranges) <= 0 or self.rangedif == -1 or abs(ranges - self.ranges[-1]) < self.rangedif:
#             self.ranges.append(ranges)
#         else:
#             self.ranges.append(self.ranges[-1])
#         # self.rangeErrors.append(selfRangeError)
#
#         # self.addMASamples()
#         self.RangePoses.append(np.square(self.ranges[-1]) - self.ROrigine)
#
#     def get_error(self, truePose):
#         if len(self.ranges) <= 0:
#             return
#         error = self.ranges[-1] - np.linalg.norm(self.pose - truePose * 1000)
#         self.errors.append(error / 1000.)
#
#
# class UWBLocalizer(Node):
#     def __init__(self):
#         super().__init__('uwb_localizer')
#
#         self.prev_pose = None
#         self.stationary_time = 0.0
#         self.stop_threshold = 0.05  # m/s
#         self.stop_duration = 3.0  # seconds
#         self.last_time = self.get_clock().now()
#         self.has_saved_plot = False
#         self.robot_position = [None, None, None]
#
#         self.real_path = []
#         self.real_path_1 = []
#         self.real_path_2 = []
#         self.estimated_path = []
#         self.estimated_path_1 = []
#         self.estimated_path_2 = []
#
#         self.anchors = {
#             "ap_0": (2700.0, 4750.0, 50.0),
#             "ap_1": (-7740.0, -8440.0, 12.4),
#             "ap_2": (-360.0, 6050.0, 10.7),
#             "ap_3": (-3350.0, -4020.0, 19.5),
#             "ap_4": (1700.0, -6050.0, 20.5),
#             "ap_5": (-4870.0, -4660.0, 74.9),
#             "ap_6": (-8380.0, -4180.0, 55.5),
#             "ap_7": (-5500.0, -7900.0, 25.0),
#         }
#
#         self.time_series = []
#         self.anchor_distances_measured = [[] for _ in range(8)]
#         self.anchor_distances_expected = [[] for _ in range(8)]
#
#         qos = QoSProfile(depth=10, reliability=QoSReliabilityPolicy.BEST_EFFORT)
#         # qos = QoSProfile(depth=10, reliability=QoSReliabilityPolicy.RELIABLE)
#
#         self.create_subscription(PoseWithCovarianceStamped, '/tb3/amcl_pose', self.pose_callback, qos)
#         self.create_subscription(Float64MultiArray, '/anchors_ranges', self.anchors_callback, 10)
#
#         self.publisher = self.create_publisher(PointStamped, '/uwb_ranging_estimated_pose', 10)
#
#         self.last_valid_dist = [None] * 8
#
#         self.become_inactive = 3.0
#
#         self.start_time = self.get_clock().now()
#
#     def anchors_callback(self, msg):
#         dist = list(msg.data[:8])
#         time_sec = float(msg.data[8])
#
#         if None in self.robot_position:
#             return  # Robot position not yet received
#
#         # Save timestamp
#         self.time_series.append(time_sec)
#
#         # Loop over all anchors and calculate expected distances
#         for idx, (anchor_id, anchor_pose) in enumerate(self.anchors.items()):
#             anchor_pos = np.array(anchor_pose)/1000
#             robot_pos_mm = self.robot_position  # Convert to mm to match anchors
#
#             expected_dist = np.linalg.norm(anchor_pos - robot_pos_mm)
#             measured_dist = dist[idx]
#
#             # Store the values
#             self.anchor_distances_measured[idx].append(measured_dist)
#             self.anchor_distances_expected[idx].append(expected_dist)
#             time_waiting = (self.get_clock().now() - self.start_time).nanoseconds / 1e9
#             print(time_waiting)
#             print(time_waiting > 30.0)
#             print(not self.has_saved_plot)
#         if not self.has_saved_plot and time_waiting > 900.0:
#             self.plot_distances()
#             self.has_saved_plot = True
#
#     def plot_distances(self):
#         if len(self.time_series) < 2:
#             return
#
#         for i, anchor_id in enumerate(self.anchors.keys()):
#             plt.figure(figsize=(10, 5))
#             plt.plot(self.time_series, self.anchor_distances_measured[i], label='Measured')
#             plt.plot(self.time_series, self.anchor_distances_expected[i], label='Expected')
#             plt.title(f"Anchor {anchor_id} Distance Over Time")
#             plt.xlabel("Time [s]")
#             plt.ylabel("Distance [m]")
#             plt.legend()
#             plt.grid(True)
#             plt.tight_layout()
#             plt.savefig(f"/sharedDrive/ranging/tb4/ranging_plot/distance_vs_expected/result_{anchor_id}.png")
#             plt.close()
#             errors = np.array(self.anchor_distances_measured[i]) - np.array(self.anchor_distances_expected[i])
#             plt.figure(figsize=(8, 5))
#             plt.hist(errors, bins=50, color='skyblue', edgecolor='black')
#             plt.title(f"Error Histogram for {anchor_id}")
#             plt.xlabel("Error [m]")
#             plt.ylabel("Frequency")
#             plt.grid(True)
#             hist_path = f"/sharedDrive/ranging/tb4/ranging_plot/error_histogram/result_{anchor_id}.png"
#             plt.tight_layout()
#             plt.savefig(hist_path)
#             self.get_logger().info(f"Saved histogram to {hist_path}")
#             plt.close()
#
#     def pose_callback(self, msg):
#         true_position = np.array([msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z])
#         self.robot_position = true_position
#
#
#
#     def save_plot(self, real_path, estimated_path, suffix=""):
#         if not real_path or not estimated_path:
#             return
#
#         # Compute per-step Euclidean errors
#         errors = [math.hypot(xr - xe, yr - ye) for (xr, yr), (xe, ye) in zip(real_path, estimated_path)]
#
#         sorted_errors = np.sort(errors)
#         cdf = np.arange(1, len(sorted_errors) + 1) / len(sorted_errors)
#
#         self.save_error_data(errors, suffix)
#
#         # Compute RMSE curve (cumulative RMSE at each point)
#         rmse_curve = []
#         for i in range(1, len(errors) + 1):
#             cumulative_rmse = math.sqrt(sum(e ** 2 for e in errors[:i]) / i)
#             rmse_curve.append(cumulative_rmse)
#
#         plt.plot(figsize=(10, 5))
#         plt.plot(rmse_curve, label='Cumulative RMSE [m]', color='green')
#         plt.xlabel('Time Step')
#         plt.ylabel('Error')
#         plt.title('Cummulative RMSE Over Time')
#         plt.legend()
#         plt.grid(True)
#
#         # Save
#         timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
#         filepath = f"/sharedDrive/ranging/tb4/ranging_plot/real{suffix}.png"
#         plt.tight_layout()
#         plt.savefig(filepath)
#         plt.close()
#
#         plt.figure(figsize=(8, 5))
#         plt.plot(sorted_errors, cdf, marker='o', linestyle='-', color='purple')
#         plt.xlabel("Error [m]")
#         plt.ylabel("Cumulative Probability")
#         plt.title("Cumulative Distribution Function (CDF) of ranging Error")
#         plt.grid(True)
#
#         cdf_filepath = f"/sharedDrive/ranging/tb4/ranging_plot/cdf_real{suffix}.png"
#         plt.tight_layout()
#         plt.savefig(cdf_filepath)
#         plt.close()
#
#         final_rmse = rmse_curve[-1]
#         self.get_logger().info(f"Saved full plot to {filepath}")
#         self.get_logger().info(f"Final RMSE: {final_rmse:.2f} m")
#
#         # Clear data
#         real_path.clear()
#         estimated_path.clear()
#
#
# def main(args=None):
#     rclpy.init(args=args)
#     node = UWBLocalizer()
#     rclpy.spin(node)
#     node.destroy_node()
#     rclpy.shutdown()