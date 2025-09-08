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

from acados_template import AcadosOcp, AcadosOcpSolver
from acados_template import AcadosModel
from dataclasses import dataclass
import numpy as np
import casadi as ca
import scipy.linalg


@dataclass
class SolverConfig:
    n_horizon: int = 10
    tf: float = 3.0
    num_max_constr: int = 30


def main():
    # create ocp object to formulate the OCP
    ocp = AcadosOcp()
    config = SolverConfig()
    num_joints = 8

    def export_system_model():

        q = ca.SX.sym('q', num_joints)
        q_dot = ca.SX.sym('qdot', num_joints)

        model = AcadosModel()
        model.x = ca.vertcat(q)
        model.u = ca.vertcat(q_dot)
        model.f_expl_expr = q_dot
        model.name = 'robot_joints'
        return model

    # set model
    model = export_system_model()
    ocp.model = model

    # set prediction horizon
    ocp.solver_options.N_horizon = config.n_horizon
    ocp.solver_options.tf = config.tf

    # cost matrices
    Q_mat = 1e-4 * np.eye(num_joints)
    R_mat = np.eye(num_joints)

    # middle stage cost
    cost_QR = scipy.linalg.block_diag(Q_mat, R_mat)
    ocp.cost.cost_type = 'LINEAR_LS'
    ocp.cost.W = cost_QR.copy()
    ocp.cost.Vx = np.zeros((2 * num_joints, num_joints))
    ocp.cost.Vx[:num_joints, :] = np.eye(num_joints)
    ocp.cost.Vu = np.zeros((2 * num_joints, num_joints))
    ocp.cost.Vu[num_joints:, :] = np.eye(num_joints)
    ocp.cost.yref  = np.zeros((2 * num_joints, ))
    # terminal stage cost
    ocp.cost.cost_type_e = 'LINEAR_LS'
    ocp.cost.W_e    = np.eye(num_joints)
    ocp.cost.Vx_e   = np.eye(num_joints)
    ocp.cost.yref_e = np.zeros((num_joints, ))

    # set constraints
    C = np.ones((config.num_max_constr, num_joints))
    D = np.zeros((config.num_max_constr, num_joints))
    lg = -1e4 * np.ones((config.num_max_constr, ))
    ug =  1e4 * np.ones((config.num_max_constr, ))
    ocp.constraints.C = C
    ocp.constraints.D = D
    ocp.constraints.lg = lg
    ocp.constraints.ug = ug
    ocp.constraints.C_e = C
    ocp.constraints.lg_e = lg
    ocp.constraints.ug_e = ug

    # NOTE: The bounds will NOT be overwritten by the sipoc_ra_solver
    max_joint_vel = 2.0
    max_lin_vel = 1.0
    ocp.constraints.idxbu = np.arange(0, num_joints)
    ocp.constraints.lbu = np.array([-max_lin_vel] + [-max_joint_vel] * (num_joints - 1))
    ocp.constraints.ubu = np.array([max_lin_vel] + [max_joint_vel] * (num_joints - 1))

    ocp.constraints.idxsg = np.arange(0, config.num_max_constr)
    ocp.constraints.idxsg_e = np.arange(0, config.num_max_constr)
    l1_penalty = 1e3
    l2_penalty = 1e2
    ocp.cost.Zl = l2_penalty * np.ones((config.num_max_constr, ))
    ocp.cost.Zu = l2_penalty * np.ones((config.num_max_constr, ))
    ocp.cost.zl = l1_penalty * np.ones((config.num_max_constr, ))
    ocp.cost.zu = l1_penalty * np.ones((config.num_max_constr, ))
    ocp.cost.Zl_e = l2_penalty * np.ones((config.num_max_constr, ))
    ocp.cost.Zu_e = l2_penalty * np.ones((config.num_max_constr, ))
    ocp.cost.zl_e = l1_penalty * np.ones((config.num_max_constr, ))
    ocp.cost.zu_e = l1_penalty * np.ones((config.num_max_constr, ))

    ocp.constraints.x0 = np.ones((num_joints, )) * 1e-2

    ocp.dims.nbxe_e = num_joints
    ocp.constraints.idxbxe_e = np.arange(0, num_joints)
    ocp.constraints.idxbx_e = np.arange(0, num_joints)
    ocp.constraints.lbx_e = -np.ones((num_joints, )) * 1e-2
    ocp.constraints.ubx_e = -np.ones((num_joints, )) * 1e-2

    # set options
    ocp.solver_options.qp_solver = 'PARTIAL_CONDENSING_HPIPM'
    ocp.solver_options.hessian_approx = 'GAUSS_NEWTON' # 'GAUSS_NEWTON', 'EXACT'
    ocp.solver_options.integrator_type = 'ERK'
    ocp.solver_options.nlp_solver_type = 'SQP' # SQP_RTI, SQP
    ocp.solver_options.nlp_solver_max_iter = 2
    ocp.solver_options.sim_method_num_stages = 1
    ocp.solver_options.levenberg_marquardt = 1e-7
    ocp.solver_options.globalization = 'FIXED_STEP' # turns on globalization
    ocp.solver_options.print_level = 0
    ocp.code_export_directory = "c_generated_code_sipoc_ra_planning"

    ocp_solver = AcadosOcpSolver(ocp, json_file = 'acados_sipoc_ra_planning.json')




if __name__ == '__main__':
    main()
