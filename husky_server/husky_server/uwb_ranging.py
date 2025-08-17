import numpy as np
from geometry_msgs.msg import PointStamped, PoseWithCovarianceStamped, PoseStamped
import rclpy
from rclpy.node import Node
from datetime import datetime
import math
import matplotlib.pyplot as plt
import numpy as np
import csv

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
        A = np.empty((0, 3))
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

class UWBLocalizer(Node):
    def __init__(self):
        super().__init__('uwb_localizer')

        self.t0 = self.get_clock().now()



        self.real_path = []
        self.estimated_path = []
        self.prev_pose = None
        self.stationary_time = 0.0
        self.stop_threshold = 0.05  # m/s
        self.stop_duration = 5.5  # seconds
        self.last_time = self.get_clock().now()
        self.has_saved_plot = True
        self.AP_range = 30000.0

        self.anchors = {
            "ap_1": [-16000.0, -24000.0, 1.0],
            "ap_2": [0.0, -24000.0, 1.5],
            "ap_3": [16000.0, -24000.0, 2.0],
            "ap_4": [-16000.0, 0.0, 2.0],
            "ap_5": [0.0, 0.0, 1.0],
            "ap_6": [16000.0, 0.0, 1.5],
            "ap_7": [-16000.0, 24000.0, 1.0],
            "ap_8": [0.0, 24000.0, 2.0],
        }

        # self.anchors = {
        #     "ap_1": [-16000.00, -24000.00, 1.0],
        #     "ap_2": [-8000.00, -24000.00, 2.0],
        #     "ap_3": [0.00, -24000.00, 1.5],
        #     "ap_4": [8000.00, -24000.00, 1.0],
        #     "ap_5": [16000.00, -24000.00, 2.0],
        #     "ap_6": [-16000.00, -8000.00, 1.5],
        #     "ap_7": [-8000.00, -8000.00, 1.0],
        #     "ap_8": [0.00, -800.00, 2.0],
        #     "ap_9": [8000.00, -8000.00, 1.5],
        #     "ap_10": [16000.00, -8000.00, 1.0],
        #     "ap_11": [-16000.00, 8000.00, 2.0],
        #     "ap_12": [-8000.00, 8000.00, 1.5],
        #     "ap_13": [0.00, 8000.00, 1.0],
        #     "ap_14": [8000.00, 8000.00, 2.0],
        #     "ap_15": [16000.00, 8000.00, 1.5],
        #     "ap_16": [-16000.00, 24000.00, 1.0],
        #     "ap_17": [-8000.00, 24000.00, 2.0],
        #     "ap_18": [0.00, 24000.00, 1.5],
        #     "ap_19": [8000.00, 24000.00, 1.0],
        #     "ap_20": [16000.00, 24000.00, 2.0],
        # }

        # self.anchors = {
        #     "ap_1": [-16000.00, -24000.00, 1.0],
        #     "ap_2": [-10670.00, -24000.00, 2.0],
        #     "ap_3": [-5330.00, -24000.00, 1.5],
        #     "ap_4": [0.00, -24000.00, 1.0],
        #     "ap_5": [5330.00, -24000.00, 2.0],
        #     "ap_6": [10670.00, -24000.00, 1.5],
        #     "ap_7": [16000.00, -24000.00, 1.0],
        #     "ap_8": [-16000.00, -14400.00, 1.5],
        #     "ap_9": [-10670.00, -14400.00, 1.0],
        #     "ap_10": [-5330.00, -14400.00, 1.5],
        #     "ap_11": [0.00, -14400.00, 2.0],
        #     "ap_12": [5330.00, -14400.00, 1.0],
        #     "ap_13": [10670.00, -14400.00, 1.5],
        #     "ap_14": [16000.00, -14400.00, 2.0],
        #     "ap_15": [-16000.00, -4800.00, 1.5],
        #     "ap_16": [-10670.00, -4800.00, 1.0],
        #     "ap_17": [-5330.00, -4800.00, 1.5],
        #     "ap_18": [0.00, -4800.00, 2.0],
        #     "ap_19": [5330.00, -4800.00, 1.0],
        #     "ap_20": [10670.00, -4800.00, 1.5],
        #     "ap_21": [16000.00, -4800.00, 2.0],
        #     "ap_22": [-16000.00, 4800.00, 1.5],
        #     "ap_23": [-10670.00, 4800.00, 1.0],
        #     "ap_24": [-5330.00, 4800.00, 1.5],
        #     "ap_25": [0.00, 4800.00, 2.0],
        #     "ap_26": [5330.00, 4800.00, 1.0],
        #     "ap_27": [10670.00, 4800.00, 1.5],
        #     "ap_28": [16000.00, 4800.00, 2.0],
        #     "ap_29": [-16000.00, 14400.00, 1.5],
        #     "ap_30": [-10670.00, 14400.00, 1.0],
        #     "ap_31": [-5330.00, 14400.00, 1.5],
        #     "ap_32": [0.00, 14400.00, 2.0],
        #     "ap_33": [5330.00, 14400.00, 1.0],
        #     "ap_34": [10670.00, 14400.00, 1.5],
        #     "ap_35": [16000.00, 14400.00, 2.0],
        #     "ap_36": [-16000.00, 24000.00, 1.0],
        #     "ap_37": [-10670.00, 24000.00, 2.0],
        #     "ap_38": [-5330.00, 24000.00, 1.5],
        #     "ap_39": [0.00, 24000.00, 1.0],
        #     "ap_40": [5330.00, 24000.00, 2.0],
        # }

        # self.anchors = {
        #     "ap_1": [-16000.00, -24000.00, 1.0],
        #     "ap_2": [-12440.00, -24000.00, 2.0],
        #     "ap_3": [-8890.00, -24000.00, 1.5],
        #     "ap_4": [-5330.00, -24000.00, 1.0],
        #     "ap_5": [-1780.00, -24000.00, 2.0],
        #     "ap_6": [1780.00, -24000.00, 1.5],
        #     "ap_7": [5330.00, -24000.00, 1.0],
        #     "ap_8": [8890.00, -24000.00, 2.0],
        #     "ap_9": [12440.00, -24000.00, 1.5],
        #     "ap_10": [16000.00, -24000.00, 1.0],
        #     "ap_11": [-16000.00, -18670.00, 2.0],
        #     "ap_12": [-12440.00, -18670.00, 1.5],
        #     "ap_13": [-8890.00, -18670.00, 1.0],
        #     "ap_14": [-5330.00, -18670.00, 2.0],
        #     "ap_15": [-1780.00, -18670.00, 1.5],
        #     "ap_16": [1780.00, -18670.00, 1.0],
        #     "ap_17": [5330.00, -18670.00, 2.0],
        #     "ap_18": [8890.00, -18670.00, 1.5],
        #     "ap_19": [12440.00, -18670.00, 1.0],
        #     "ap_20": [16000.00, -18670.00, 2.0],
        #     "ap_21": [-16000.00, -13330.00, 1.5],
        #     "ap_22": [-12440.00, -13330.00, 1.0],
        #     "ap_23": [-8890.00, -13330.00, 1.5],
        #     "ap_24": [-5330.00, -13330.00, 2.0],
        #     "ap_25": [-1780.00, -13330.00, 1.5],
        #     "ap_26": [1780.00, -13330.00, 1.0],
        #     "ap_27": [5330.00, -13330.00, 2.0],
        #     "ap_28": [8890.00, -13330.00, 1.5],
        #     "ap_29": [12440.00, -13330.00, 1.0],
        #     "ap_30": [16000.00, -13330.00, 2.0],
        #     "ap_31": [-16000.00, -8000.00, 1.5],
        #     "ap_32": [-12440.00, -8000.00, 1.0],
        #     "ap_33": [-8890.00, -8000.00, 1.5],
        #     "ap_34": [-5330.00, -8000.00, 2.0],
        #     "ap_35": [-1780.00, -8000.00, 1.5],
        #     "ap_36": [1780.00, -8000.00, 1.0],
        #     "ap_37": [5330.00, -8000.00, 2.0],
        #     "ap_38": [8890.00, -8000.00, 1.5],
        #     "ap_39": [12440.00, -8000.00, 1.0],
        #     "ap_40": [16000.00, -8000.00, 2.0],
        #     "ap_41": [-16000.00, -2670.00, 1.5],
        #     "ap_42": [-12440.00, -2670.00, 1.0],
        #     "ap_43": [-8890.00, -2670.00, 1.5],
        #     "ap_44": [-5330.00, -2670.00, 2.0],
        #     "ap_45": [-1780.00, -2670.00, 1.5],
        #     "ap_46": [1780.00, -2670.00, 1.0],
        #     "ap_47": [5330.00, -2670.00, 2.0],
        #     "ap_48": [8890.00, -2670.00, 1.5],
        #     "ap_49": [12440.00, -2670.00, 1.0],
        #     "ap_50": [16000.00, -2670.00, 2.0],
        #     "ap_51": [-16000.00, 2670.00, 1.5],
        #     "ap_52": [-12440.00, 2670.00, 1.0],
        #     "ap_53": [-8890.00, 2670.00, 1.5],
        #     "ap_54": [-5330.00, 2670.00, 2.0],
        #     "ap_55": [-1780.00, 2670.00, 1.5],
        #     "ap_56": [1780.00, 2670.00, 1.0],
        #     "ap_57": [5330.00, 2670.00, 2.0],
        #     "ap_58": [8890.00, 2670.00, 1.5],
        #     "ap_59": [12440.00, 2670.00, 1.0],
        #     "ap_60": [16000.00, 2670.00, 2.0],
        #     "ap_61": [-16000.00, 8000.00, 1.5],
        #     "ap_62": [-12440.00, 8000.00, 1.0],
        #     "ap_63": [-8890.00, 8000.00, 1.5],
        #     "ap_64": [-5330.00, 8000.00, 2.0],
        #     "ap_65": [-1780.00, 8000.00, 1.5],
        #     "ap_66": [1780.00, 8000.00, 1.0],
        #     "ap_67": [5330.00, 8000.00, 2.0],
        #     "ap_68": [8890.00, 8000.00, 1.5],
        #     "ap_69": [12440.00, 8000.00, 1.0],
        #     "ap_70": [16000.00, 8000.00, 2.0],
        #     "ap_71": [-16000.00, 13330.00, 1.5],
        #     "ap_72": [-12440.00, 13330.00, 1.0],
        #     "ap_73": [-8890.00, 13330.00, 1.5],
        #     "ap_74": [-5330.00, 13330.00, 2.0],
        #     "ap_75": [-1780.00, 13330.00, 1.5],
        #     "ap_76": [1780.00, 13330.00, 1.0],
        #     "ap_77": [5330.00, 13330.00, 2.0],
        #     "ap_78": [8890.00, 13330.00, 1.5],
        #     "ap_79": [12440.00, 13330.00, 1.0],
        #     "ap_80": [16000.00, 13330.00, 2.0],
        #     "ap_81": [-16000.00, 18670.00, 1.5],
        #     "ap_82": [-12440.00, 18670.00, 1.0],
        #     "ap_83": [-8890.00, 18670.00, 1.5],
        #     "ap_84": [-5330.00, 18670.00, 2.0],
        #     "ap_85": [-1780.00, 18670.00, 1.5],
        #     "ap_86": [1780.00, 18670.00, 1.0],
        #     "ap_87": [5330.00, 18670.00, 2.0],
        #     "ap_88": [8890.00, 18670.00, 1.5],
        #     "ap_89": [12440.00, 18670.00, 1.0],
        #     "ap_90": [16000.00, 18670.00, 2.0],
        #     "ap_91": [-16000.00, 24000.00, 1.5],
        #     "ap_92": [-12440.00, 24000.00, 1.0],
        #     "ap_93": [-8890.00, 24000.00, 1.5],
        #     "ap_94": [-5330.00, 24000.00, 2.0],
        #     "ap_95": [-1780.00, 24000.00, 1.5],
        #     "ap_96": [1780.00, 24000.00, 1.0],
        #     "ap_97": [5330.00, 24000.00, 2.0],
        #     "ap_98": [8890.00, 24000.00, 1.5],
        #     "ap_99": [12440.00, 24000.00, 1.0],
        #     "ap_100": [16000.00, 24000.00, 2.0],
        # }

        self.uwb = UWBSetup()
        self.uwb.addUWBAnchors(self.anchors)

        self.create_subscription(PoseStamped, '/a200_0957/ground_truth_pose', self.pose_callback, 10)
        self.publisher = self.create_publisher(PointStamped, '/uwb_estimated_pose', 10)

        # self.noise_std = 0.1
        self.noise_std = 100

    def pose_callback(self, msg):
        true_position = np.array([msg.pose.position.x, msg.pose.position.y, msg.pose.position.z])
        # true_position = np.array([msg.pose.position.x, msg.pose.position.y])
        for ap_id, ap_pos in self.anchors.items():
            # distance = np.linalg.norm(np.array(ap_pos) - true_position)
            # print(ap_pos, true_position, np.array(ap_pos), np.linalg.norm(np.array(ap_pos) - true_position*1000.0))
            true_distance = np.linalg.norm(np.array(ap_pos) - true_position*1000.0)

            noisy_distance = true_distance + np.random.normal(0, self.noise_std)

            estimate = self.uwb.readUWBInput(ap_id, msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9,
                                                 noisy_distance)

            if estimate is not None:
                estimate_m = estimate * 0.001
                now = self.get_clock().now()
                elapsed = (now - self.last_time).nanoseconds * 1e-9
                self.last_time = now

                rx, ry = msg.pose.position.x, msg.pose.position.y

                if self.prev_pose:
                    dx = rx - self.prev_pose[0]
                    dy = ry - self.prev_pose[1]
                    velocity = math.hypot(dx, dy) / elapsed
                else:
                    velocity = 0.0
                self.prev_pose = (rx, ry)

                if velocity >= self.stop_threshold:
                    # self.get_logger().info("I move")
                    self.real_path.append((rx, ry))
                    self.estimated_path.append((estimate_m[0], estimate_m[1]))
                    t = (now - self.t0).nanoseconds * 1e-9
                    self.err_time.append(t)
                    self.stationary_time = 0.0
                    self.has_saved_plot = False
                else:
                    self.stationary_time += elapsed
                    # self.get_logger().info("I dont move")
                    # self.get_logger().info(f"{self.stationary_time:.3f} seconds")

                    if self.stationary_time >= self.stop_duration and not self.has_saved_plot:
                        self.save_plot()
                        self.stationary_time = 0.0
                        self.has_saved_plot = True

                        self.get_logger().info("Stopping node after saving plot.")
                        rclpy.shutdown()
                        exit(0)


                estimated_pose = PointStamped()
                estimated_pose.header = msg.header
                estimated_pose.point.x = float(estimate_m[0])
                estimated_pose.point.y = float(estimate_m[1])
                estimated_pose.point.z = float(estimate_m[2])
                self.publisher.publish(estimated_pose)



                # self.get_logger().info(
                #         f"[UWB Estimate] x={estimate[0]:.2f}, y={estimate[1]:.2f}, z={estimate[2]:.2f}"
                #
                # )
                # self.get_logger().info(
                #         f"[UWB True] x={true_position[0]:.2f}, y={true_position[1]:.2f}, z={true_position[2]:.2f}"
                #
                # )

    def save_error_data(self, errors, suffix = ""):
        # timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        filepath = f"/sharedDrive/PLOT_GRAPH/simu/{suffix}_8_ranging_simulation.csv"

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
        # filepath = f"/sharedDrive/ranging/husky/ranging_plot/{timestamp}.png"
        filepath = f"/sharedDrive/ranging/husky/ranging_plot/8_anchors_with_real_noise.png"
        plt.tight_layout()
        plt.savefig(filepath)
        plt.close()

        plt.figure(figsize=(8, 5))
        plt.plot(sorted_errors, cdf, marker='o', linestyle='-', color='purple')
        plt.xlabel("Error [m]")
        plt.ylabel("Cumulative Probability")
        plt.title("Cumulative Distribution Function (CDF) of ranging Error")
        plt.grid(True)

        cdf_filepath = f"/sharedDrive/ranging/husky/ranging_plot/cdf_8_anchors_with_real_noise.png"
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
    node = UWBLocalizer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()