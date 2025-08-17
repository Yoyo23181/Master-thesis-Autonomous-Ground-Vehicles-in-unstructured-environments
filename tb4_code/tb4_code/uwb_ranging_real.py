import numpy as np
from geometry_msgs.msg import PointStamped, PoseWithCovarianceStamped, PoseStamped
import rclpy
from rclpy.node import Node
from datetime import datetime
import math
import matplotlib.pyplot as plt
import numpy as np
import csv
from nav_msgs.msg import Odometry
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
from std_msgs.msg import Float64MultiArray

# https://www.researchgate.net/publication/224222701_Static_positioning_using_UWB_range_measurements
class UWBSetup():
    def __init__(self, rangeDif=-1):
        self.UWBAnchors = []
        self.rangeDif = rangeDif

        self.ranges = {}
        self.singleValue = True

    def addUWBAnchors(self, anchors):

        for identifier in anchors:
            self.addUWBAnchor(identifier, anchors[identifier])

            # form https://www.researchgate.net/publication/224222701_Static_positioning_using_UWB_range_measurements/link/0fcfd51071298bfea5000000/download
        self.A = self.calculateSL1_AMatrix()
        self.AtA = np.matmul(np.transpose(self.A), self.A)

    def calculateSL1_AMatrix(self):
        A = np.empty((0, 2))
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
        self.UWBAnchors.append(UWBAnchor(identifier, pose, self.rangeDif))

    def getUWBIdentifier(self, identifier):
        for uwbA in self.UWBAnchors:
            if uwbA.identifier == identifier:
                return uwbA
        return None

    def readUWBInput(self, anchorId, time, distance):
        anchor = self.getUWBIdentifier(anchorId)
        if anchor != None:
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

    def estimateUncertainty(self, sigma):
        if self.AtA is None:
            return None
        scale = 8 * (6.7 * 2) * (sigma * 2)
        Cov_X = scale * np.linalg.inv(self.AtA * 1e-6)  # scale the covariance matrix
        std_devs = np.sqrt(np.diag(Cov_X))

        # Cov_X = sigma ** 2 * np.linalg.inv(self.AtA*1e-6)
        std_devs = np.sqrt(np.diag(Cov_X))
        print(np.linalg.norm(std_devs))
        print(np.linalg.norm(std_devs[:2]))
        return std_devs  # returns uncertainties in [x, y, z]


class UWBAnchor:
    def __init__(self, identifier, pose, rangedif=-1):
        self.identifier = identifier
        self.pose = np.array(pose)
        self.ranges = []

        self.rangedif = rangedif

        self.RangePoses = []
        self.ROrigine = np.square(np.linalg.norm(self.pose))
        self.errors = []

    def addSample(self, ranges):
        # limits speed of system to {rangedif} m * f (10 hz)
        # not sure if this ok -> To much trust on UWB measurements... ?
        if len(self.ranges) <= 0 or self.rangedif == -1 or abs(ranges - self.ranges[-1]) < self.rangedif:
            self.ranges.append(ranges)
        else:
            self.ranges.append(self.ranges[-1])
        # self.rangeErrors.append(selfRangeError)

        # self.addMASamples()
        self.RangePoses.append(np.square(self.ranges[-1]) - self.ROrigine)

    def get_error(self, truePose):
        if len(self.ranges) <= 0:
            return
        error = self.ranges[-1] - np.linalg.norm(self.pose - truePose * 1000)
        self.errors.append(error / 1000.)


class Variance_NLOS_Tracking:
    def __init__(self, history = 10, threshold =0.03):
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

        self.t0 = self.get_clock().now()
        self.err_time = []

        self.prev_pose = None
        self.stationary_time = 0.0
        self.stop_threshold = 0.05  # m/s
        self.stop_duration = 3.0  # seconds
        self.last_time = self.get_clock().now()
        self.has_saved_plot = False
        self.robot_position = [None, None, None]

        self.real_path = []
        self.real_path_1 = []
        self.real_path_2 = []
        self.estimated_path = []
        self.estimated_path_1 = []
        self.estimated_path_2 = []



        self.anchors_z = {
            "ap_0": (50.0),
            "ap_1": (12.4),
            "ap_2": (10.7),
            "ap_3": (19.5),
            "ap_4": (20.5),
            "ap_5": (74.9),
            # "ap_6": (55.5),
            "ap_7": (25.0),
        }

        self.anchors_without_z= {
            "ap_0": (2700, 4750),
            "ap_1": (-7740, -8440),
            "ap_2": (-360, 6050),
            "ap_3": (-3350, -4020),
            "ap_4": (1700, -6050),
            "ap_5": (-4870, -4660),
            # "ap_6": (-8380, -4180),
            "ap_7": (-5500, -7900),
        }
        # self.anchors_1_without_z = {
        #     "ap_0": (2700, 4750),
        #     "ap_2": (-360, 6050),
        #     "ap_3": (-3350, -4020),
        #     "ap_4": (1700, -6050),
        # }
        #
        # self.anchors_1_z = {
        #     "ap_0": (50.0),
        #     "ap_2": (10.7),
        #     "ap_3": (19.5),
        #     "ap_4": (20.5),
        # }

        self.anchors_1_without_z = {
            "ap_0": (2700, 4750),
            "ap_2": (-360, 6050),
            "ap_3": (-3350, -4020),
        }

        self.anchors_1_z = {
            "ap_0": (50.0),
            "ap_2": (10.7),
            "ap_3": (19.5),

        }

        # self.anchors_2_without_z = {
        #     "ap_1": (-7740, -8440),
        #     "ap_3": (-3350, -4020),
        #     "ap_4": (1700, -6050),
        #     "ap_5": (-4870, -4660),
        #     "ap_6": (-8380, -4180),
        #     "ap_7": (-5500, -7900),
        # }
        #
        # self.anchors_2_z = {
        #     "ap_1": (12.4),
        #     "ap_3": (19.5),
        #     "ap_4": (20.5),
        #     "ap_5": (74.9),
        #     "ap_6": (55.5),
        #     "ap_7": (25.0),
        # }

        self.anchors_2_without_z = {
            "ap_1": (-7740, -8440),
            "ap_5": (-4870, -4660),
            "ap_7": (-5500, -7900),
        }

        self.anchors_2_z = {
            "ap_1": (12.4),
            "ap_5": (74.9),
            "ap_7": (25.0),
        }

        # self.anchors_2_without_z = {
        #     "ap_3": (-3350, -4020),
        #     "ap_5": (-4870, -4660),
        #     "ap_7": (-5500, -7900),
        # }
        #
        # self.anchors_2_z = {
        #     "ap_3": (19.5),
        #     "ap_5": (74.9),
        #     "ap_7": (25.0),
        # }

        self.group_1 = {"ap_0", "ap_2", "ap_3"}
        # self.group_2 = {"ap_1", "ap_3", "ap_4", "ap_5", "ap_6", "ap_7"}
        # self.group_2 = {"ap_1", "ap_5", "ap_7"}
        # self.group_2 = {"ap_3", "ap_5", "ap_7"}
        self.group_2 = {"ap_5", "ap_7"}

        self.uwb = UWBSetup()

        self.uwb_1 = UWBSetup()
        self.uwb_2 = UWBSetup()


        self.uwb_1.addUWBAnchors(self.anchors_1_without_z)
        # self.uwb_2.addUWBAnchors(self.anchors_2_without_z)

        qos = QoSProfile(depth=10,  reliability=QoSReliabilityPolicy.BEST_EFFORT)
        # qos = QoSProfile(depth=10, reliability=QoSReliabilityPolicy.RELIABLE)

        self.create_subscription(PoseWithCovarianceStamped, '/tb3/amcl_pose', self.pose_callback, qos)
        self.create_subscription(Float64MultiArray, '/anchors_ranges', self.anchors_callback, 10)

        self.publisher = self.create_publisher(PointStamped, '/uwb_ranging_estimated_pose', 10)

        self.last_valid_dist = [None] * 8

        self.become_inactive = 3.0

        self.start_time = self.get_clock().now()

        self.dist_2d_projected = [None] * 8

        # var = Variance_NLOS_Tracking()
        # self.nlos_trackers = {ap_id: var.new_measurement(self.dist_2d_projected)
        #                       for ap_id in self.anchors_without_z.keys()}
        self.nlos_trackers = {ap_id: Variance_NLOS_Tracking()
                              for ap_id in self.anchors_without_z.keys()}




    def project_to_2d(self, dist_3d, z_anchor, z_robot):
        dz = z_anchor - (z_robot * 1000.0)
        return math.sqrt(dist_3d ** 2 - dz ** 2) if dist_3d > abs(dz) else dist_3d


    def anchors_callback(self, msg):
        anchors_active = []
        dist_active = []


        dist = list(msg.data[:8])



        time_sec = float(msg.data[8])

        for i, (ap_id, ap_pos) in enumerate(self.anchors_without_z.items()):
            current_dist = dist[i]

            if not math.isfinite(current_dist):
                self.last_valid_dist[i] = None
                continue
            if self.last_valid_dist[i] is not None and math.isclose(current_dist, self.last_valid_dist[i], rel_tol=1e-9):
                continue

            self.last_valid_dist[i] = current_dist

            if self.robot_position[2] is not None and ap_id in self.anchors_z:
                projected_dist = self.project_to_2d(current_dist, self.anchors_z[ap_id], self.robot_position[2])
            else:
                projected_dist = current_dist

            dist[i] = projected_dist

            is_los = self.nlos_trackers[ap_id].new_measurement(projected_dist)


            if not is_los:
                continue

            anchors_active.append(ap_id)
            dist_active.append(projected_dist)

        anchors_used = {ap_id: self.anchors_without_z[ap_id] for ap_id in anchors_active}


        if len(anchors_active) < 3:
            return
        self.uwb.UWBAnchors.clear()
        self.uwb.ranges.clear()
        self.uwb.addUWBAnchors(anchors_used)
        for j, (ap_id, ap_pos) in enumerate(anchors_used.items()):
            # true_distance = dist[j]
            true_distance = dist[j]

            estimate = self.uwb.readUWBInput(ap_id, time_sec, true_distance)

            if estimate is not None:
                now = self.get_clock().now()
                self.last_time = now

                rx, ry = self.robot_position[0], self.robot_position[1]

                if (rx is not None) and (ry is not None) and (estimate[0] is not None) and (estimate[1] is not None) and (math.isfinite(estimate[0])) and (math.isfinite(estimate[1])):
                    self.real_path.append((rx, ry))
                    self.estimated_path.append((estimate[0]/1000, estimate[1]/1000))
                    t = (now - self.t0).nanoseconds * 1e-9
                    self.err_time.append(t)
                total_runtime = (self.get_clock().now() - self.start_time).nanoseconds * 1e-9

                print(total_runtime)

                if total_runtime > 900.0 and not self.has_saved_plot:
                    self.save_plot(self.real_path, self.estimated_path, "_main")
                    self.save_plot(self.real_path_1, self.estimated_path_1, "_area_1")
                    self.save_plot(self.real_path_2, self.estimated_path_2, "_area_2")
                    self.has_saved_plot = True

                estimated_pose = PointStamped()
                estimated_pose.header.stamp.sec = int(time_sec)
                estimated_pose.header.stamp.nanosec = int((time_sec % 1) * 1e9)
                estimated_pose.point.x = float(estimate[0]/1000)
                estimated_pose.point.y = float(estimate[1]/1000)
                self.publisher.publish(estimated_pose)



        active_set = set(anchors_active)

        # if self.group_1.issubset(active_set):
        # # if self.group_1 & active_set:
        #     # tru_dis = [dist[list(self.anchors_without_z.keys()).index(ap_id)] for ap_id in self.anchors_1_without_z]
        #     distance_map = dict(zip(anchors_active, dist_active))
        #
        #     tru_dis = [distance_map[ap_id] for ap_id in self.anchors_1_without_z if ap_id in distance_map]
        #     for w, (ap_id, ap_pos) in enumerate(self.anchors_1_without_z.items()):
        #         true_distance_1 = tru_dis[w]
        #         estimate_1 = self.uwb_1.readUWBInput(ap_id, time_sec, true_distance_1)
        #         if estimate_1 is not None:
        #             rx, ry = self.robot_position[0], self.robot_position[1]
        #             now = self.get_clock().now()
        #             self.last_time = now
        #             if rx is not None and ry is not None and estimate_1[0] is not None and estimate_1[1] is not None and math.isfinite(estimate_1[0]) and math.isfinite(estimate_1[1]):
        #                 self.real_path_1.append((rx, ry))
        #                 self.estimated_path_1.append((estimate_1[0] / 1000, estimate_1[1] / 1000))
        #
        #
        #
        #
        # if self.group_2.issubset(active_set):
        # # if self.group_2 & active_set:
        #     # tru_dist = [dist[list(self.anchors_without_z.keys()).index(ap_id)] for ap_id in self.anchors_2_without_z]
        #
        #     dist_map = dict(zip(anchors_active, dist_active))
        #
        #     tru_dist = [dist_map[ap_id] for ap_id in anchors_used if ap_id in dist_map]
        #     for p, (ap_id, ap_pos) in enumerate(anchors_used.items()):
        #
        #         true_distance_2 = tru_dist[p]
        #
        #         self.uwb_2.UWBAnchors.clear()
        #         self.uwb_2.ranges.clear()
        #         self.uwb_2.addUWBAnchors(anchors_used)
        #
        #         estimate_2 = self.uwb_2.readUWBInput(ap_id, time_sec, true_distance_2)
        #         if estimate_2 is not None:
        #             rx, ry = self.robot_position[0], self.robot_position[1]
        #             now = self.get_clock().now()
        #             self.last_time = now
        #
        #             if rx is not None and ry is not None and estimate_2[0] is not None and estimate_2[1] is not None and math.isfinite(estimate_2[0]) and math.isfinite(estimate_2[1]):
        #                 print("even further")
        #                 self.real_path_2.append((rx, ry))
        #                 self.estimated_path_2.append((estimate_2[0] / 1000, estimate_2[1] / 1000))



    def pose_callback(self, msg):
        true_position = np.array([msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z])
        self.robot_position = true_position



    def save_error_data(self, errors, suffix = ""):
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        filepath = f"/sharedDrive/ranging/tb4/error_log/{suffix}_2D_real_ranging.csv"

        with open(filepath, mode='w', newline='') as file:
            writer = csv.writer(file)
            writer.writerow(["Step", "Error [m]"])
            for t, e in errors:
                writer.writerow([f"{t:.6f}", f"{e:.6f}"])

        self.get_logger().info(f"Saved error log to {filepath}")


    def save_plot(self, real_path, estimated_path, suffix=""):
        if not real_path or not estimated_path:
            print("there is nothing to plot")
            return


        # Compute per-step Euclidean errors
        errors = [math.hypot(xr - xe, yr - ye) for (xr, yr), (xe, ye) in zip(real_path, estimated_path)]

        sorted_errors = np.sort(errors)
        cdf = np.arange(1, len(sorted_errors) + 1) / len(sorted_errors)


        # Compute RMSE curve (cumulative RMSE at each point)
        rmse_curve = []
        for i in range(1, len(errors) + 1):
            cumulative_rmse = math.sqrt(sum(e ** 2 for e in errors[:i]) / i)
            rmse_curve.append(cumulative_rmse)

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

        # Save
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        filepath = f"/sharedDrive/ranging/tb4/ranging_plot/2D_real{suffix}.png"
        plt.tight_layout()
        plt.savefig(filepath)
        plt.close()

        plt.figure(figsize=(8, 5))
        plt.plot(sorted_errors, cdf, marker='o', linestyle='-', color='purple')
        plt.xlabel("Error [m]")
        plt.ylabel("Cumulative Probability")
        plt.title("Cumulative Distribution Function (CDF) of ranging Error")
        plt.grid(True)

        cdf_filepath = f"/sharedDrive/ranging/tb4/ranging_plot/2D_cdf_real{suffix}.png"
        plt.tight_layout()
        plt.savefig(cdf_filepath)
        plt.close()

        final_rmse = rmse_curve[-1]
        self.get_logger().info(f"Saved full plot to {filepath}")
        self.get_logger().info(f"Final RMSE: {final_rmse:.2f} m")

        # Clear data
        real_path.clear()
        estimated_path.clear()

def main(args=None):
    rclpy.init(args=args)
    node = UWBLocalizer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()