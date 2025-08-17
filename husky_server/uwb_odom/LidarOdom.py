import numpy as np
from Code.UtilityCode import Transformation_Matrix_Fucntions as TMF

class LIDAROdom():
    '''
    For now we will take the AMCL_pose as the ground truth and calculate form that the odometry
    Hopefully it will be good enough.
    '''
    def __init__(self):
        self.d_t_odom = np.zeros(4)
        self.T_map_agv = np.eye(4)
        self.t_map_agv = np.zeros(4)
        self.t_odom = np.zeros(4)
        self.previous_T_map = np.eye(4)
        self.prev_time = None
        self.dt = 0
        self.q_v = np.eye(4)
        self.q = np.eye(4)

    def set_q_dt(self, q_v):
        self.q_v = q_v # q_v is the variance of the odometry speed and angular velocity.
        self.q = q_v # Assume dt = 1s. This will be updated in the callback

    def odom_callback(self, p, q, time):
        self.T_map_agv = TMF.transformation_matrix_from_q(q, p)
        if self.prev_time is not None:
            self.dt = time - self.prev_time

            # print(self.T_map)
            T_odom = TMF.inv_transformation_matrix(self.previous_T_map) @ self.T_map_agv
            self.d_t_odom = TMF.get_4D_t_from_matrix(T_odom)
            self.q = self.q_v * self.dt ** 2 # Needs to be sqaure to be correct -> Otherwise dim self.q = m^2/s iso m^2 since self.q_v = m^2/s^2
        self.previous_T_map = self.T_map_agv
        self.t_odom = TMF.get_4D_t_from_matrix(self.T_map_agv)
        self.t_map_agv = TMF.get_4D_t_from_matrix(self.T_map_agv)
        self.prev_time = time
