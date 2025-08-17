import numpy as np
from geometry_msgs.msg import PointStamped, PoseStamped
import rclpy
from rclpy.node import Node
from datetime import datetime
import math
import matplotlib.pyplot as plt
from husky_server.uwb_odom.Fixed2DUPF import Fixed2DUPF
from husky_server.uwb_odom.LidarOdom import LIDAROdom
import Code.UtilityCode.Transformation_Matrix_Fucntions as TMF
import csv
from nav_msgs.msg import Odometry



class UWB_ODOM_RANGING(Node):
    def __init__(self):
        super().__init__('uwb_odom_ranging')

        self.real_path = []
        self.estimated_path = []

        self.robot_id = "husky"
        self.agv_v1 = "agv1"
        # self.sigma_uwb = 0.1
        self.sigma_uwb = 1e-1
        dz =-0.13
        self.t_map_velodyne = np.zeros(4)
        self.t_map_robot = None

        self.lidar_odom = LIDAROdom()
        q_v = np.eye(4) *0.1**2
        self.lidar_odom.set_q_dt( q_v)
        self.robot_upf = None

        # self.robot_upf = Fixed2DUPF(id=self.robot_id, x_ha_0=np.zeros(4), drift_correction_bool=True, dz=dz)
        # self.robot_upf.set_ukf_parameters(kappa=-1, alpha=1, beta=1)
        # self.robot_upf.set_ukf_parameters(kappa=0, alpha=0.1, beta=2.0)
        # self.robot_upf_likelihood_threshold = 0.5
        self.robot_upf_likelihood_threshold = 0.9
        self.min_range_for_acceptance = 15.0

        self.create_subscription(PoseStamped, '/a200_0957/ground_truth_pose', self.pose_callback, 10)
        self.create_subscription(Odometry, '/a200_0957/platform/odom', self.odom_callback, 10)
        self.publisher = self.create_publisher(PointStamped, '/uwb_odom_estimated_pose', 10)

        self.stationnary_time = 0.0
        self.stop_threshold = 0.05
        self.stop_duration = 5.0
        self.prev_pose = None

        self.latest_uwb_distance = None
        self.has_saved_plot = False
        self.last_time = None

        self.anchor_position = np.array([0.0, 0.0, 0.0, 0.0], dtype=float)

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
    def pose_callback(self, msg):
        x_pose = msg.pose.position.x
        y_pose = msg.pose.position.y
        z_pose = msg.pose.position.z
        w_orientation = msg.pose.orientation.w
        x_orientation = msg.pose.orientation.x
        y_orientation = msg.pose.orientation.y
        z_orientation = msg.pose.orientation.z

        self.t_real =  np.array([
            x_pose, y_pose, z_pose,
        ])
        self.q_real = np.array([
            w_orientation, x_orientation, y_orientation, z_orientation,
        ])


        self.true_position = (x_pose,y_pose,z_pose)



    def odom_callback(self, msg):
        x_odom = msg.pose.pose.position.x
        y_odom = msg.pose.pose.position.y
        z_odom = msg.pose.pose.position.z
        w_orientation = msg.pose.pose.orientation.w
        x_orientation = msg.pose.pose.orientation.x
        y_orientation = msg.pose.pose.orientation.y
        z_orientation = msg.pose.pose.orientation.z



        p_gt = np.array([
            x_odom, y_odom, z_odom,
        ])
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
                dz = -0.13
                self.robot_upf = Fixed2DUPF(id=self.robot_id, x_ha_0=p, drift_correction_bool=True, dz=dz)
                self.robot_upf.set_ukf_parameters(kappa=-1, alpha=1, beta=1)
                self.robot_upf.set_initalisation_parameters(0.1, 12, 1, dz, 1e-2)
            else:
                # if np.linalg.norm(dt) > 0.000001:
                self.robot_upf.ha.predict(dt, q_ha)
                self.robot_upf.ha.update(p, q_ha)
                robot_uwb = np.linalg.norm(self.true_position - self.anchor_position[:3]) + np.random.normal(0, self.noise_std)

                if robot_uwb is None or robot_uwb == 0:
                    raise Exception("UWB distance is 0 or None")
                self.latest_uwb_distance = robot_uwb
                self.robot_upf.run_2d_model(robot_uwb)

                self.robot_upf.ha.reset_integration()

                self.t_si_sj = self.robot_upf.best_particle.t_si_sj

                T_si_sj = TMF.transformation_matrix_from_4D_t(self.t_si_sj)

                T_o_r = TMF.transformation_matrix_from_q(self.q_real, self.t_real)
                t_or = TMF.get_4D_t_from_matrix(T_o_r)
                # print(t_or)
                T_o_a = TMF.transformation_matrix_from_4D_t(self.anchor_position)

                T_o_r_inversed = TMF.inv_transformation_matrix(T_o_r)

                T_r_a_est = T_o_r_inversed @ T_o_a

                self.rel_pose_r_a = TMF.get_translation(T_r_a_est)

                rel_pose_est = np.array([self.t_si_sj[0], self.t_si_sj[1]])
                rel_pose_gt = np.array(self.rel_pose_r_a[0], self.rel_pose_r_a[1])

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
                # print(f"UWB : {robot_uwb}, {p}")
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




            # self.get_logger().info(f"I'm there")
            # self.get_logger().info(f" weight : {self.robot_upf.best_particle.weight > 0.8}")
            # self.get_logger().info(f" likelihood : {self.robot_upf.best_particle.kf.likelihood > self.robot_upf_likelihood_threshold}")
            # self.get_logger().info(f"{self.robot_upf.best_particle.weight}")
            # self.get_logger().info(f"{self.robot_upf.best_particle.kf.likelihood}")
            # self.get_logger().info(f"{self.robot_upf_likelihood_threshold}")
            # self.get_logger().info(f"Best particle weight: {self.robot_upf.best_particle.weight:.3f}")
            # self.get_logger().info(f"Best particle KF likelihood: {self.robot_upf.best_particle.kf.likelihood:.3f}")

            # print(t_sec)
            # print(self.robot_upf.best_particle.weight > 0.8 and self.robot_upf.best_particle.kf.likelihood > self.robot_upf_likelihood_threshold)
            # if (self.robot_upf.best_particle.weight > 0.8 and self.robot_upf.best_particle.kf.likelihood > self.robot_upf_likelihood_threshold):
            # # if True:
            #     t_si_sj = self.robot_upf.best_particle.t_si_sj
            #     if not isinstance(t_si_sj, np.ndarray) or t_si_sj.shape != (4,):
            #         raise ValueError(f"Expected t_si_sj to be shape (4,), got {t_si_sj.shape} with value: {t_si_sj}")
            #
            #
            #     if not isinstance(self.t_map_velodyne, np.ndarray) or self.t_map_velodyne.shape != (4,):
            #         raise ValueError(f"t_map_velodyne is invalid: {self.t_map_velodyne}")
            #     T_map_vel = TMF.transformation_matrix_from_4D_t(self.t_map_velodyne)
            #
            #     T_map_sj = T_map_vel @ T_si_sj
            #
            #     t_map_sj = TMF.get_4D_t_from_matrix(T_map_sj)
            #     t_map_sj[-1] = 0
            #     dis = np.linalg.norm(t_map_sj[:2] - self.t_map_robot[:2])
            #     if dis > self.min_range_for_acceptance:
            #         self.get_logger().warn(f"Distance too large: {dis:.2f} m")
            #
            #     self.publish_estimate(msg, p_gt, t_map_sj, dtime)


        except Exception as e:
            self.get_logger().error(f"RPE update failed: {e}")

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
        filepath = f"/sharedDrive/PLOT_GRAPH/simu/{suffix}_uwb_ranging_simulation.csv"

        with open(filepath, mode='w', newline='') as file:
            writer = csv.writer(file)
            writer.writerow(["Step", "Error [m]"])
            for t, e in errors:
                writer.writerow([f"{t:.6f}", f"{e:.6f}"])

        self.get_logger().info(f"Saved error log to {filepath}")

    def publish_estimate(self, msg):
        point = PointStamped()
        point.header = msg.header
        point.header.frame_id = "map"
        # point.point.x = float(estimate[0])
        # point.point.y = float(estimate[1])
        # point.point.z = float(estimate[2])
        point.point.x = float(self.rel_pose_r_a[0])
        point.point.y = float(self.rel_pose_r_a[1])
        # point.point.z = float(estimate[2])
        self.publisher.publish(point)
        now = self.get_clock().now()

        # self.real_path.append((p_gt[0], p_gt[1]))
        t = (now - self.t0).nanoseconds * 1e-9
        self.err_time.append(t)
        self.real_path.append( (float(self.rel_pose_r_a[0]), float(self.rel_pose_r_a[1])) )
        self.estimated_path.append((float(self.t_si_sj[0]), float(self.t_si_sj[1]) ) )



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
            print(self.stationnary_time)

            if self.stationnary_time >= self.stop_duration and not self.has_saved_plot:
                self.save_plot()
                self.save_logs()
                self.robot_upf.reset()

                self.stationnary_time = 0.0
                self.has_saved_plot = True

        else:
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
        filepath = f"/sharedDrive/dlio/husky/dlio_plot/real_noise.png"
        plt.tight_layout()
        plt.savefig(filepath)
        plt.close()

        plt.figure(figsize=(8, 5))
        plt.plot(sorted_errors, cdf, marker='o', linestyle='-', color='purple')
        plt.xlabel("Error [m]")
        plt.ylabel("Cumulative Probability")
        plt.title("Cumulative Distribution Function (CDF) of odometry with uwb range Error")
        plt.grid(True)

        cdf_filepath = f"/sharedDrive/dlio/husky/dlio_plot/cdf_real_noise.png"
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


# self.robot_upf.ha.predict(dt, q)
#             self.robot_upf.ha.update(t, q)
#             robot_uwb = self.uwb.pairs[self.agv_id+"_"+self.robot_id].get_distance()
#             if robot_uwb == 0. or robot_uwb is None:
#                 raise Exception("UWB distance is 0 or None")
#             self.robot_upf.run_2d_model(robot_uwb)
#             self.robot_upf.ha.reset_integration()
#             # print(self.robot_upf.best_particle.weight, self.robot_upf.best_particle.kf.likelihood)
#             if (self.robot_upf.best_particle.weight > 0.8 and self.robot_upf.best_particle.kf.likelihood > self.robot_upf_likelihood_threshold):
#                 T_si_sj = TMF.transformation_matrix_from_4D_t(self.robot_upf.best_particle.t_si_sj)
#                 T_map_sj = TMF.transformation_matrix_from_4D_t(self.t_map_velodyne) @ T_si_sj
#                 t_map_sj = TMF.get_4D_t_from_matrix(T_map_sj)
#                 t_map_sj[-1] = 0 # set the angle to 0 since we now the translation from the beacon to the place in the map frame.
#                 dis = np.linalg.norm(t_map_sj[:2] - self.t_map_robot[:2])
#                 #TODO: check this function (additional safety by using the map.)
#                 if dis > self.min_range_for_acceptance:
#                     print("Distance between RPE and measured is too large: ", dis)
#                     print("Possibly RPE did not converged or map drift is to big... please check map.")
#                     print("RPE: ", t_map_sj, "Measured: ", self.t_map_robot)
#                     print("t_map_vel: ", self.t_map_velodyne, "t_si_sj: ", self.robot_upf.best_particle.t_si_sj)
#                     # self.robot_upf.reset()
#                 self.rpe_robot = t_map_sj
# self.robot_upf = Fixed2DUPF(id=self.robot_id, x_ha_0=np.zeros(4), drift_correction_bool=True, dz=dz)
# self.robot_upf.set_ukf_parameters(kappa=-1, alpha=1, beta=2)
# self.robot_upf.set_initalisation_parameters(2 * self.sigma_uwb, 12, 1, dz, 0.1)