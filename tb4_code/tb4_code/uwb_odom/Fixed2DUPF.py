import matplotlib.pyplot as plt
from Code.ParticleFilter.ConnectedAgentClass import UPFConnectedAgent
from Code.ParticleFilter.TargetTrackingUKF import TargetTrackingUKF
from Code.DataLoggers.ConnectedAgent_DataLogger import UPFConnectedAgentDataLogger
from Code.DataLoggers.TargetTrackingUKF_DataLogger import UKFDatalogger
from Code.UtilityCode.utility_fuctions import (transform_matrix, sphericalToCartesian, inv_transformation_matrix,
                                               get_states_of_transform, get_4d_rot_matrix, limit_angle,
                                               cartesianToSpherical, sphericalToCartesian)
import numpy as np
import math

class Fixed2DUPF(UPFConnectedAgent):
    def __init__(self, id="0x000", x_ha_0=np.zeros(4), drift_correction_bool=True, dz=0.):
        super().__init__(id=id, x_ha_0=x_ha_0, drift_correction_bool=drift_correction_bool)
        #TODO fix this in the main UPF class
        self.uwb_measurement = 0.
        # self.set_regeneration_parameters(max_number_of_particles=5)
        self.dz = dz

    def reset(self):
        self.particles = []
        self.best_particle = None

    def set_initalisation_parameters(self, sigma_uwb: float, n_azimuth: int, n_heading: int, dz: float, sigma_dz: float):
        self.n_altitude = 1
        self.n_azimuth = n_azimuth
        self.n_heading = n_heading
        self.sigma_uwb = sigma_uwb
        self.sigma_dz = sigma_dz
        self.dz = dz

    def initialise_particles(self, r :float):
        altitude = np.arcsin(self.dz / r)
        sigma_altitude = np.abs(np.arcsin(self.dz + self.sigma_dz / r)) - np.abs(altitude)
        azimuths = [2 * np.pi / self.n_azimuth * j for j in range(self.n_azimuth)]
        sigma_azimuth = (2 * np.pi / self.n_azimuth) / np.sqrt(-8 * np.log(0.5))
        heading = 0
        sigma_heading = np.pi

        sigma_s = [2*self.sigma_uwb, sigma_azimuth, sigma_altitude]
        for azimuth in azimuths:
            s = np.array([r, azimuth, altitude], dtype=float)
            particle = self.create_particle()
            particle.set_initial_state(s, sigma_s, heading, sigma_heading, self.sigma_uwb)
            self.particles.append(particle)

        self.set_best_particle(self.particles[0])


    def create_particle(self) -> TargetTrackingUKF:
        weight = 1. / self.n_azimuth / self.n_heading / self.n_altitude
        particle = Fixed2DUKF(x_ha_0=self.ha.x_ha_0, weight=weight, drift_correction_bool=self.drift_correction_bool)
        particle.set_ukf_properties(self.kappa, self.alpha, self.beta)
        particle.set_uwb_extrinsicity(self.t_si_uwb, self.t_sj_uwb)
        return particle


    def split_sphere_in_equal_areas(self, r: float, sigma_uwb: float, n_azimuth: int, n_heading: int, dz: float, dz_sigma: float):
        """
        Function to split the area of a sphere in almost equal areas (= weights)
        Starting from the n_altitude that has to be uneven (will be made uneven).
        For each altitude
        """
        self.n_altitude = 1
        self.n_azimuth = n_azimuth
        self.n_heading = n_heading
        self.sigma_uwb = sigma_uwb

        altitude = np.arcsin(dz/r)
        sigma_altitude = np.abs(np.arcsin(dz+dz_sigma/r)) - np.abs(altitude)
        azimuths = [2 * np.pi / n_azimuth * j for j in range(n_azimuth)]
        sigma_azimuth = (2 * np.pi / n_azimuth) / np.sqrt(-8 * np.log(0.5))
        heading = 0
        sigma_heading = np.pi

        # for i, altitude in enumerate(altitudes):
        #     azimuths = [-np.pi + (2 * np.pi / azimuth_bins[i]) * j for j in range(azimuth_bins[i])]
        sigma_s = [sigma_uwb, sigma_azimuth, sigma_altitude]
        for azimuth in azimuths:
            s = np.array([r, azimuth, altitude], dtype=float)
            particle = self.create_particle()
            particle.set_initial_state(s, sigma_s, heading, sigma_heading, sigma_uwb)
            self.particles.append(particle)
        # for alt in [-np.pi/2, np.pi/2]:
        #     for az in [-np.pi, np.pi]:
        #         for heading in headings:
        #             particle = TargetTrackingUKF(x_ha_0=self.ha.x_ha_0, weight=1)
        #             particle.set_ukf_properties(self.kappa, self.alpha, self.beta)
        #             particle.set_initial_state(np.array([r, az, alt]), np.array([sigma_uwb, 2*np.pi/3, sigma_altitude]),
        #                                        heading, sigma_heading, sigma_uwb)
        #             self.particles.append(particle)

        self.set_best_particle(self.particles[0])

    def run_2d_model(self, measurement, time_i=None):
        #TODO: Maybe not bad to take this structure in the parent class.
        if self.particles == []:
            self.initialise_particles(measurement)
        else:
            # print(measurement, self.ha.x_ha_0, self.ha.x_ha)
            # print(self.ha.p_dx_ha)
            super().run_model(dx_ca = np.zeros(4), measurement = measurement, q_ca = np.zeros((4,4)), time_i=time_i)


    def generate_new_particle(self):
        azimuth = np.random.uniform(-np.pi, np.pi)
        # altitude = np.random.uniform(-np.pi / 2, np.pi / 2)
        altitude = np.arcsin(self.dz / self.uwb_measurement)
        heading = 0
        # TODO should make something that is parameterized.
        sigma_azimuth = (2 * np.pi / 8) / np.sqrt(-8 * np.log(0.5))
        sigma_altitude = (np.pi / 5) / np.sqrt(-8 * np.log(0.5))
        sigma_heading = (2 * np.pi / 8) / np.sqrt(-8 * np.log(0.5))
        particle = self.create_particle()
        particle.weight = 0.2
        s = np.array([self.uwb_measurement, azimuth, altitude], dtype=float)
        sigma_s = [2 * self.sigma_uwb, sigma_azimuth, sigma_altitude]
        particle.set_initial_state(s, sigma_s, heading, sigma_heading, self.sigma_uwb)
        self.particles.append(particle)


class Fixed2DUKF(TargetTrackingUKF):
    def __init__(self, x_ha_0=np.zeros(4), weight =1,  drift_correction_bool=True):
        super().__init__(x_ha_0=x_ha_0,weight=weight, drift_correction_bool=drift_correction_bool)


    def calculate_P_x_ca(self):
        f = get_4d_rot_matrix(self.kf.x[3])
        stds = np.sqrt(np.diag(self.kf.P))
        ca_0 = sphericalToCartesian(self.kf.x[:3])  # + self.x_ha_0[:3]
        ca_0_max = sphericalToCartesian(self.kf.x[:3] + stds[:3])  # + self.x_ha_0[:3]
        dis_0 = np.linalg.norm(ca_0_max[:2] - ca_0[:2])

        self.sigma_x_ca_0 = dis_0  # / self.kf.x[0]
        self.sigma_x_ca = np.sqrt(dis_0 ** 2
                                  + np.linalg.norm(self.kf.P[4:-3, 4:-3].astype(np.float64))
                                  + np.linalg.norm(self.q_ca[:2, :2].astype(np.float64)))
        self.sigma_h_ca = np.sqrt(self.kf.P[3, 3] + self.kf.P[-2, -2] + self.q_ca[-1, -1])



class Fixed2DUKFDataLogger(UKFDatalogger):
    def log_spherical_data(self, i):
        ca_p_real = self.connected_agent.x_real[i]
        ca_h_real = self.connected_agent.h_real[i]
        ha_p_real = self.host_agent.x_real[i]
        ha_h_real = self.host_agent.h_real[i]

        # relative_transformation = (ca_p_real - ha_p_real)
        # spherical_relative_transformation = cartesianToSpherical(relative_transformation)
        # spherical_relative_transformation[1] = limit_angle(spherical_relative_transformation[1] - ha_h_real)
        # relative_transformation_r = sphericalToCartesian(spherical_relative_transformation)
        #
        # error_relative_transformation_est = np.linalg.norm(
        #     relative_transformation_r - sphericalToCartesian(self.ukf.s_ca_r))
        t_G_si = np.append(ha_p_real, np.array([ha_h_real]))
        T_G_si = transform_matrix(t_G_si)
        T_si_G = inv_transformation_matrix(np.append(ha_p_real, np.array([ha_h_real])))
        T_G_sj = transform_matrix(np.append(ca_p_real, np.array([ca_h_real])))
        T_si_sj = T_si_G @ T_G_sj
        t_si_sj = get_states_of_transform(T_si_sj)
        e_t_si_sj = t_si_sj - self.ukf.t_si_sj

        error_relative_transformation_est = np.linalg.norm(e_t_si_sj[:2])
        self.error_relative_transformation_est.append(error_relative_transformation_est)

        spherical_relative_transformation = cartesianToSpherical(t_si_sj[:3])
        self.spherical_relative_transformation = np.append(self.spherical_relative_transformation,
                                                           spherical_relative_transformation.reshape(1, 3), axis=0)

        spherical_relative_transformation_estimation = np.reshape(cartesianToSpherical(self.ukf.t_si_sj[:3]), (1, 3))
        self.spherical_estimated_relative_transformation = np.append(self.spherical_estimated_relative_transformation,
                                                                     spherical_relative_transformation_estimation,
                                                                     axis=0)

        self.spherical_estimated_relative_start_position = np.append(self.spherical_estimated_relative_start_position,
                                                                     np.reshape(self.ukf.kf.x[:4], (1, 4)), axis=0)

        error_s = spherical_relative_transformation - spherical_relative_transformation_estimation
        error_s[0, 1] = limit_angle(error_s[0, 1])
        error_s[0, 2] = limit_angle(error_s[0, 2])
        error_s = np.reshape(np.abs(error_s), (1, 3))
        self.error_spherical_relative_transformation_estimation = np.append(
            self.error_spherical_relative_transformation_estimation, error_s, axis=0)

        est_T_G_sj = T_G_si @ transform_matrix(self.ukf.t_si_sj)
        est_t_G_sj = get_states_of_transform(est_T_G_sj)
        self.estimated_ca_position = np.append(self.estimated_ca_position, est_t_G_sj[:3].reshape(1, 3), axis=0)

    def plot_2D_drift(self):
        plt.figure()
        # plot drift of the position
        plt.ylabel("Position [m]")
        # ax[0].plot(self.error_ca_position, linestyle=self.relative_linestyle, color="darkgreen",
        #            label="ca position estimation error")
        plt.plot(self.error_relative_transformation_est, linestyle=self.relative_linestyle,
                   color="tab:blue", label="Error on 2D position estimation", linewidth=2)
        # ax[0].plot(self.error_relative_transformation_slam, linestyle=self.relative_linestyle, color=self.slam_color,
        #            label="relative transformation slam")
        # ax[0].plot(self.sigma_x_ca_0, color=self.estimation_color, linestyle=self.stds_linestyle, label="std on the start position estimation")
        plt.plot(self.sigma_x_ca, color="red", linestyle=self.stds_linestyle,
                   label="Convidence on the 2D position estimation", linewidth=2)
        plt.grid(True)
        plt.legend()

class Fixed2DUPFDataLogger(UPFConnectedAgentDataLogger):
    def add_particle(self, particle):
        # particle.set_datalogger(self.host_agent, self.connected_agent, name="Particle " + str(self.particle_count))
        particle_log = Fixed2DUKFDataLogger(self.host_agent, self.connected_agent, particle, name="Particle " + str(self.particle_count))
        self.particle_count += 1
        self.particle_logs.append(particle_log)
