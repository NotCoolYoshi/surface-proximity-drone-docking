# controller.py
import numpy as np

class CascadedPD:

    def __init__(self, x_des, z_des):
        self.x_des = x_des
        self.z_des = z_des

    def update_target(self, x_des, z_des):
        self.x_des = x_des
        self.z_des = z_des

    def compute(self, state, params):
        x, z, vx, vz, theta, theta_dot = state
        m = params['m']
        g = params['g']
        I = params['I']

        # Outer loop: vertical
        ez = self.z_des - z
        az_des = params['kp_z'] * ez + params['kd_z'] * (0.0 - vz)
        T = m * (g + az_des)
        T = float(np.clip(T, 0.0, params['T_max']))

        # Outer loop: horizontal
        ex = self.x_des - x
        ax_des = params['kp_x'] * ex + params['kd_x'] * (0.0 - vx)

        if T > 0.1:
            theta_des = -(m * ax_des) / T
        else:
            theta_des = 0.0
        theta_des = float(np.clip(theta_des,
                                  -params['theta_max'],
                                   params['theta_max']))

        # Inner loop: attitude
        e_theta = theta_des - theta
        tau = I * (params['kp_th'] * e_theta
                 + params['kd_th'] * (0.0 - theta_dot))

        return T, tau