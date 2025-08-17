import numpy as np
from geometry_msgs.msg import PointStamped, PoseStamped, PoseWithCovarianceStamped
import rclpy
from rclpy.node import Node
from datetime import datetime
import math
import matplotlib.pyplot as plt
from tb4_code.uwb_odom.Fixed2DUPF import Fixed2DUPF
from tb4_code.uwb_odom.LIDAROdom import LIDAROdom
import Code.UtilityCode.Transformation_Matrix_Fucntions as TMF
import csv
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64MultiArray
from rclpy.qos import qos_profile_sensor_data
import traceback





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
class UWB_ODOM_RANGING(Node):
    def __init__(self):
        super().__init__('uwb_odom_ranging')

        self.real_path = []
        self.estimated_path = []

        self.robot_id = "husky"
        self.agv_v1 = "agv1"
        # self.sigma_uwb = 0.1
        self.sigma_uwb = 0.3
        dz =-0.13
        self.t_map_velodyne = np.zeros(4)
        self.t_map_robot = None

        self.lidar_odom = LIDAROdom()
        # q_v = np.eye(4) *0.1
        # q_v = np.eye(4) *0.05**2 # vio
        q_v = np.eye(4) *0.005**2 # wheel odom
        self.lidar_odom.set_q_dt( q_v)
        self.robot_upf = None

        # self.robot_upf = Fixed2DUPF(id=self.robot_id, x_ha_0=np.zeros(4), drift_correction_bool=True, dz=dz)
        # self.robot_upf.set_ukf_parameters(kappa=-1, alpha=1, beta=1)
        # self.robot_upf.set_ukf_parameters(kappa=0, alpha=0.1, beta=2.0)
        # self.robot_upf_likelihood_threshold = 0.5
        self.robot_upf_likelihood_threshold = 0.9
        self.min_range_for_acceptance = 15.0

        self.create_subscription(PoseWithCovarianceStamped, '/tb3/amcl_pose', self.pose_callback, 10)
        self.create_subscription(Odometry, '/tb3/odom', self.odom_callback, qos_profile_sensor_data)
        # self.create_subscription(Odometry, '/vio', self.odom_callback, qos_profile_sensor_data)
        self.create_subscription(Float64MultiArray, '/anchors_ranges', self.anchors_callback, 10)


        self.publisher = self.create_publisher(PointStamped, '/uwb_odom_estimated_pose', 10)


        self.stationnary_time = 0.0
        self.stop_threshold = 0.05
        self.stop_duration = 30.0
        self.prev_pose = None

        self.latest_uwb_distance = None
        self.has_saved_plot = False
        self.last_time = None

        self.anchor_position = np.array([-3.350, -4.020, 0.0195, 0.0], dtype=float)

        self.rel_pose_r_a = []

        self.t_real = None
        self.q_real = None
        self.true_position = None

        self.log_t_real = []
        self.log_q_real = []
        self.log_x_ha = []

        self.noise_std = 0.1

        self.err_time = []
        self.t0 = self.get_clock().now()
        self.robot_uwb = None



        self.anchors= {
            "ap_0": (2.700, 4.750, 0.0500),
            "ap_1": (-7.740, -8.440, 0.0124),
            "ap_2": (-0.360, 6.050, 0.0107),
            "ap_3": (-3.350, -4.020, 0.0195),
            "ap_4": (1.700, -6.050, 0.0205),
            "ap_5": (-4.870, -4.660, 0.0749),
            "ap_6": (-8.380, -4.180, 0.0555),
            "ap_7": (-5.500, -7.900, 0.0250),
        }

        self.last_valid_dist = [None] * 8

        self.become_inactive = 3.0

        self.nlos_trackers = {ap_id: Variance_NLOS_Tracking()
                              for ap_id in self.anchors.keys()}

        self.anchor_id = "ap_3"

        # self.declare_parameter('vio_offset_x', 2.1355)
        self.declare_parameter('odom_offset_x', -0.02)
        # self.declare_parameter('vio_offset_y', -1.8)
        self.declare_parameter('odom_offset_y', +0.12)
        self.declare_parameter('odom_deg_yaw', 0.0)

        # self.declare_parameter('odom_offset_x', 1.79)  # vio
        # self.declare_parameter('odom_offset_y', 2.13)  # vio
        # self.declare_parameter('odom_deg_yaw', 90.0)  # vio

        self.odom_dx = float(self.get_parameter('odom_offset_x').value)
        self.odom_dy = float(self.get_parameter('odom_offset_y').value)
        self.odom_deg_yaw = self.get_parameter('odom_deg_yaw').value
        self.odom_theta = math.radians(self.odom_deg_yaw)
        self.odom_cos = math.cos(self.odom_theta)
        self.odom_sin = math.sin(self.odom_theta)



    def anchors_callback(self, msg):

        anchors_active = []
        dist_active = []

        dist = list(msg.data[:8])

        time_sec = float(msg.data[8])
        for i, (ap_id, ap_pos) in enumerate(self.anchors.items()):
            current_dist = dist[i]

            if not math.isfinite(current_dist):
                self.last_valid_dist[i] = None
                continue
            if self.last_valid_dist[i] is not None and math.isclose(current_dist, self.last_valid_dist[i], rel_tol=1e-9):
                continue

            self.last_valid_dist[i] = current_dist



            dist[i] = current_dist

            is_los = self.nlos_trackers[ap_id].new_measurement(current_dist)


            if not is_los:
                continue

            anchors_active.append(ap_id)
            dist_active.append(current_dist)

        active_dist_by_id = {aid: d for aid, d in zip(anchors_active, dist_active)}
        if self.anchor_id in active_dist_by_id:
            self.robot_uwb = active_dist_by_id[self.anchor_id]


        # anchors_used = {ap_id: self.anchors[ap_id] for ap_id in anchors_active}
        #
        #
        # for i, (ap_id, ap_pos) in enumerate(anchors_used.items()):
        #     if ap_id == self.anchor_id:
        #         self.robot_uwb = dist[i]


    def pose_callback(self, msg):
        x_pose = msg.pose.pose.position.x
        y_pose = msg.pose.pose.position.y
        z_pose = msg.pose.pose.position.z
        w_orientation = msg.pose.pose.orientation.w
        x_orientation = msg.pose.pose.orientation.x
        y_orientation = msg.pose.pose.orientation.y
        z_orientation = msg.pose.pose.orientation.z

        self.t_real =  np.array([
            x_pose, y_pose, z_pose,
        ])
        self.q_real = np.array([
            w_orientation, x_orientation, y_orientation, z_orientation,
        ])


        self.true_position = (x_pose,y_pose,z_pose)



    def odom_callback(self, msg):
        if self.t_real is None or self.q_real is None:
            self.get_logger().warn("Waiting for AMCL pose before processing odom.")
            return
            # Require at least one UWB update
        if self.robot_uwb is None:
            self.get_logger().warn("Waiting for UWB range (/anchors_ranges).")
            return
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        z_odom = msg.pose.pose.position.z
        w_orientation = msg.pose.pose.orientation.w
        x_orientation = msg.pose.pose.orientation.x
        y_orientation = msg.pose.pose.orientation.y
        z_orientation = msg.pose.pose.orientation.z

        x_odom = self.odom_cos * x - self.odom_sin * y + self.odom_dx
        y_odom = self.odom_sin * x + self.odom_cos * y + self.odom_dy
        # z_odom = z + 0.78

        p_gt = np.array([
            x_odom, y_odom, z_odom,
        ])
        print(p_gt, self.true_position)
        q_gt = np.array([
            w_orientation, x_orientation, y_orientation, z_orientation,
        ])
        t_sec = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        self.lidar_odom.odom_callback(p_gt, q_gt, t_sec)
        # print(f"dt: {self.lidar_odom.d_t_odom}")
        # print(f"t_robot: {self.lidar_odom.t_map_agv}")
        dt = self.lidar_odom.d_t_odom
        p = self.lidar_odom.t_map_agv
        q = self.lidar_odom.q
        dtime = self.lidar_odom.dt
        try:
            # self.get_logger().info(f"Calling predict with dt={dt}, q.shape={q.shape}")
            # self.get_logger().info(f"d_t_odom: {self.lidar_odom.d_t_odom}, shape: {self.lidar_odom.d_t_odom.shape}")
            # self.get_logger().info(f"q:\n{q}")
            q_ha = q
            # print(q)

              # Simulated anchor at origin



            # update the attribute for future use
            # print(self.t_real, self.q_real, dt)

            if self.robot_upf is None:
                dz = -0.195
                self.robot_upf = Fixed2DUPF(id=self.robot_id, x_ha_0=p, drift_correction_bool=True, dz=dz)
                self.robot_upf.set_ukf_parameters(kappa=-1, alpha=1, beta=1)
                self.robot_upf.set_initalisation_parameters(0.5, 12, 1, dz, 0.1)
            else:
                # if np.linalg.norm(dt) > 0.000001:
                self.robot_upf.ha.predict(dt, q_ha)
                self.robot_upf.ha.update(p, q_ha)
                # robot_uwb = np.linalg.norm(self.true_position - self.anchor_position[:3]) + np.random.normal(0, self.noise_std)

                if self.robot_uwb is None or self.robot_uwb == 0:
                    raise Exception("UWB distance is 0 or None")
                self.latest_uwb_distance = self.robot_uwb
                self.robot_upf.run_2d_model(self.robot_uwb)

                self.robot_upf.ha.reset_integration()

                self.t_si_sj = self.robot_upf.best_particle.t_si_sj

                if self.t_si_sj.size != 4:
                    self.get_logger().warn(f"t_si_sj not length 4 (got {self.t_si_sj}). Skipping this step.")
                    return

                T_si_sj = TMF.transformation_matrix_from_4D_t(self.t_si_sj)

                T_o_r = TMF.transformation_matrix_from_q(self.q_real, self.t_real)
                t_or = TMF.get_4D_t_from_matrix(T_o_r)
                # print(t_or)
                T_o_a = TMF.transformation_matrix_from_4D_t(self.anchor_position)

                T_o_r_inversed = TMF.inv_transformation_matrix(T_o_r)

                T_r_a_est = T_o_r_inversed @ T_o_a

                self.rel_pose_r_a = TMF.get_translation(T_r_a_est)

                rel_pose_est = np.array([self.t_si_sj[0], self.t_si_sj[1]])
                rel_pose_gt = np.array([self.rel_pose_r_a[0], self.rel_pose_r_a[1]])
                # rel_pose_gt = np.array(self.rel_pose_r_a[0], self.rel_pose_r_a[1])

                error = rel_pose_gt - rel_pose_est
                error = self.t_si_sj[:2] - self.rel_pose_r_a[:2]

                err = np.linalg.norm(error)

                rmse_step = np.sqrt(np.mean(error ** 2))
                # self.get_logger().info(f" error :  {err} ")

                # dX = self.anchor_position[0] - self.true_position[0]
                # dY = self.anchor_position[1] - self.true_position[1]
                #
                # rel_x = math.cos(yaw) * dX + math.sin(yaw) * dY
                # rel_y = - math.sin(yaw) * dX + math.cos(yaw) * dY
                #
                # print(f"UWB : {self.robot_uwb}, {p}")
                # print(f"x_ha: {self.robot_upf.ha.x_ha}")
                # print(f"t_true: {self.true_position}")
                # print(f"A_pos: {self.anchor_position[:3]}")
                # print(f"T_o_a: {T_o_a}")
                # print(f"T_OR: {T_o_r}")
                # print(f"T_OR_inversed: {T_o_r_inversed}")
                # print(f"t_est: {self.t_si_sj}")
                # print(f"t_R_a_est: {self.rel_pose_r_a}")
                # print(f"error: {error}, err: {err}")
                # for particle in self.robot_upf.particles:
                #     print(particle.weight, particle.t_si_sj)
                # print("------------------------------------------")

                self.log_t_real.append(self.t_real.copy())
                self.log_q_real.append(self.q_real.copy())
                self.log_x_ha.append(self.robot_upf.ha.x_ha.copy())

                self.publish_estimate(msg)







        except Exception as e:
            self.get_logger().error(f"RPE update failed: {e}")
            self.get_logger().error("RPE update failed:\n" + traceback.format_exc())

    def save_logs(self):
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        filepath = f"/sharedDrive/dlio/husky/logs/simulation_log_{timestamp}.csv"

        with open(filepath, mode="w", newline="") as f:
            writer = csv.writer(f)
            writer.writerow(["t_real_x", "t_real_y", "t_real_z",
                             "q_real_w", "q_real_x", "q_real_y", "q_real_z",
                             "x_ha_0", "x_ha_1", "x_ha_2", "x_ha_3"])
            for t, q, xha in zip(self.log_t_real, self.log_q_real, self.log_x_ha):
                writer.writerow([*t, *q, *xha])

        self.get_logger().info(f"Saved log to {filepath}")

    def save_error_data(self, errors, suffix=""):
        # timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        filepath = f"/sharedDrive/PLOT_GRAPH/Real/{suffix}_uwb_ranging_simulation.csv"

        with open(filepath, mode='w', newline='') as file:
            writer = csv.writer(file)
            writer.writerow(["Step", "Error [m]"])
            for t, e in errors:
                writer.writerow([f"{t:.6f}", f"{e:.6f}"])

        self.get_logger().info(f"Saved error log to {filepath}")

    def publish_estimate(self, msg):

        if self.true_position is None:
            self.get_logger().warn("No true_position yet; skipping velocity & plotting logic.")
            return

        point = PointStamped()
        point.header = msg.header
        point.header.frame_id = "map"
        point.point.x = float(self.rel_pose_r_a[0])
        point.point.y = float(self.rel_pose_r_a[1])
        # point.point.z = float(estimate[2])
        self.publisher.publish(point)

        # self.real_path.append((p_gt[0], p_gt[1]))
        # t = (now - self.t0).nanoseconds * 1e-9
        # self.err_time.append(t)
        # self.real_path.append( (float(self.rel_pose_r_a[0]), float(self.rel_pose_r_a[1])) )
        # self.estimated_path.append((float(self.t_si_sj[0]), float(self.t_si_sj[1]) ) )



        now = self.get_clock().now()
        if self.last_time is None:
            elapsed = 0.0
        else:
            elapsed = (now - self.last_time).nanoseconds * 1e-9
        self.last_time = now


        if self.prev_pose is not None:
            dx = self.true_position[0] - self.prev_pose[0]
            dy = self.true_position[1] - self.prev_pose[1]
            velocity = math.hypot(dx, dy) / elapsed

        else:
            velocity = 0.0
        self.prev_pose = self.true_position


        if velocity < self.stop_threshold:
            self.stationnary_time += elapsed
            # print(self.stationnary_time)

            if self.stationnary_time >= self.stop_duration and not self.has_saved_plot:
                self.save_plot()
                self.save_logs()
                self.robot_upf.reset()

                self.stationnary_time = 0.0
                self.has_saved_plot = True

        else:
            t = (now - self.t0).nanoseconds * 1e-9
            self.err_time.append(t)
            self.real_path.append((float(self.rel_pose_r_a[0]), float(self.rel_pose_r_a[1])))
            self.estimated_path.append((float(self.t_si_sj[0]), float(self.t_si_sj[1])))
            self.stationnary_time = 0.0
            self.has_saved_plot = False

    def save_plot(self):
        if not self.real_path or not self.estimated_path:
            return

        x_real, y_real = zip(*self.real_path)
        x_est, y_est = zip(*self.estimated_path)

        errors = [math.hypot(xr - xe, yr - ye) for (xr, yr), (xe, ye) in zip(self.real_path, self.estimated_path)]

        sorted_errors = np.sort(errors)
        cdf = np.arange(1, len(sorted_errors) + 1) / len(sorted_errors)

        rmse_curves = []
        for i in range(1, len(errors) + 1):
            cumulative_rmse = math.sqrt(sum(e**2 for e in errors[:i]) / i)
            rmse_curves.append(cumulative_rmse)

        total_distance = sum(
            math.hypot(x_real[i+1] - x_real[i], y_real[i+1] - y_real[i]) for i in range(len(x_real) - 1)
        )
        percent_error = [(e / total_distance * 100) if total_distance > 0 else 0 for e in errors]

        t_errors = list(zip(self.err_time, errors))
        t_rmse = list(zip(self.err_time, rmse_curves))
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
        filepath = f"/sharedDrive/dlio/tb4/dlio_plot/real_noise.png"
        plt.tight_layout()
        plt.savefig(filepath)
        plt.close()

        plt.figure(figsize=(8, 5))
        plt.plot(sorted_errors, cdf, marker='o', linestyle='-', color='purple')
        plt.xlabel("Error [m]")
        plt.ylabel("Cumulative Probability")
        plt.title("Cumulative Distribution Function (CDF) of odometry with uwb range Error")
        plt.grid(True)

        cdf_filepath = f"/sharedDrive/dlio/tb4/dlio_plot/cdf_real_noise.png"
        plt.tight_layout()
        plt.savefig(cdf_filepath)
        plt.close()

        final_rmse = rmse_curves[-1]
        self.get_logger().info(f"Saved full plot to {filepath}")
        self.get_logger().info(f"Final RMSE: {final_rmse:.2f} m")

        # Clear data
        self.real_path.clear()
        self.estimated_path.clear()

def main(args=None):
    rclpy.init(args=args)
    node = UWB_ODOM_RANGING()
    rclpy.spin(node)
    rclpy.destroy_node()
    rclpy.shutdown()
