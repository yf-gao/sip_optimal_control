# Copyright (C) 2025 Robert Bosch GmbH

# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU Affero General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.

# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU Affero General Public License for more details.

# You should have received a copy of the GNU Affero General Public License
# along with this program.  If not, see <http://www.gnu.org/licenses/>.
#
# -------------------------------------
# Author: Yunfan Gao
#


import casadi as ca
from scipy.linalg import logm, expm
import numpy as np


class DifferentialDriveRobot():
    # Consider a discrete-time linear system for the robot dynamics:
    # \nu_{k+1} = disc_A \nu_k + disc_B (linVCmd_k, angVCmd_k)
    # (linVMeas_k, angVMeas_k) = dist_C \nu_k + dist_D (linVCmd_k, angVCmd_k),
    # where $\nu_k \in R^{4}$ represents the dynamic state.
    # The values of the disc_A, disc_B, dist_C, and dist_D matrices in the following are specific for the Neobotix MP-500 robot (https://www.neobotix-robots.com/products/mobile-robots/mobile-robot-mp-500) with a maximum forward velocity of 3m/s and angular velocity of 5rad/s,
    # and are obtained by using the N4SID method (https://nfoursid.readthedocs.io/en/latest/) based on the data collected from the robot.
    disc_A = np.array([[0.73698832, 0.01597382, 0.06026152, 0.07303846],
                       [-0.0078049, 0.69396595, -0.11718393, 0.01482203],
                       [-0.77785672, 0.81754223, 0.92222123, -0.01603535,],
                       [-1.2356698, -0.81602845, -0.02435645, 0.86768727]])
    disc_B = np.array([[-4.16130160e-02, 3.29138983e-02,],
                       [-4.30305613e-02, -5.23138700e-02,],
                       [-1.93680236e-02, 2.28163012e-01,],
                       [-3.16698394e-01, -1.01792606e-04]])
    dist_C = np.array([[-3.70147307, -2.65703515, -0.16787712, -0.21704129,],
                       [2.50575244, -3.52984477, 0.24767055, -0.28634375],])
    dist_D = np.array([[ 0.02128009, -0.03711049,],
                       [-0.06290439, 0.09915334],])


    def d2c(self, delta_t:float) -> None:
        self.cont_A = logm(self.disc_A) / delta_t
        self.cont_B = self.cont_A @ np.linalg.inv(self.disc_A - np.eye(self.disc_A.shape[0])) @ self.disc_B
        self.cont_C = self.dist_C.copy()
        self.cont_D = self.dist_D.copy()


    def __init__(self, delta_t:float):
        self._model_name = "diff_drive_robot"
        self._nx = 9
        self._nu = 2
        self.d2c(delta_t)


    def export_robot_sys_dyn_xdot(self, x:ca.SX, u:ca.SX, flag_ode_with_sys_dyn: bool) -> ca.SX:
        if x.shape[0] != self._nx and u.shape[0] != self._nu:
            raise Exception("The shape of x and u is incorrect.")
        v_ref = x[3:5]
        x_sid = x[5:]
        if flag_ode_with_sys_dyn:
            tmp = ca.DM(self.cont_C) @ x_sid + ca.DM(self.cont_D) @ v_ref
            x_kin_dot = ca.vertcat(tmp[0]*ca.cos(x[2]),
                                tmp[0]*ca.sin(x[2]),
                                tmp[1])
        else:
            x_kin_dot = ca.vertcat(v_ref[0]*ca.cos(x[2]),
                                v_ref[0]*ca.sin(x[2]),
                                v_ref[1])
        v_ref_dot = u
        x_sid_dot = ca.DM(self.cont_A) @ x_sid + ca.DM(self.cont_B) @ v_ref
        return ca.vertcat(x_kin_dot, v_ref_dot, x_sid_dot)


    def export_robot_ode_fun(self, flag_ode_with_sys_dyn: bool) -> ca.Function:
        x = ca.SX.sym('x', self._nx)
        u = ca.SX.sym('u', self._nu)
        x_dot = self.export_robot_sys_dyn_xdot(x, u, flag_ode_with_sys_dyn)
        return ca.Function('ode', [x, u], [x_dot], ['x', 'u'], ['x_dot'])


    def export_simulator(self, delta_t:float, flag_ode_with_sys_dyn: bool) -> ca.Function:
        x = ca.SX.sym('x_sim', self.nx)
        u = ca.SX.sym('u_sim', self.nu)
        f_x = self.export_robot_sys_dyn_xdot(x, u, flag_ode_with_sys_dyn)
        dae = {'x': x, 'p': u, 'ode': f_x}
        opts = {'tf': delta_t}
        I = ca.integrator('I', 'rk', dae, opts)
        return I
