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
import numpy as np
import scipy.linalg

from acados_template import AcadosModel, AcadosOcp, AcadosOcpSolver

from sipoc_mr_support.configs import SipocSolverConfig, RobotConfig
from sipoc_mr_support.robot_dynamics_model import DifferentialDriveRobot


class SipocMRNLPSubproblem():
    def __init__(self, robot_cfg: RobotConfig, solver_cfg:SipocSolverConfig) -> None:
        self._robot_cfg  = robot_cfg
        self._solver_cfg = solver_cfg


    def _setup_linear_state_constraints(self, ocp:AcadosOcp) -> None:
        ocp.constraints.x0 = np.zeros((self._solver_cfg.nx, ))
        ocp.constraints.idxbx = np.array([3, 4])
        ocp.constraints.lbx = np.array([self._robot_cfg.min_forward_velocity, -self._robot_cfg.max_angular_velocity])
        ocp.constraints.ubx = np.array([self._robot_cfg.max_forward_velocity,  self._robot_cfg.max_angular_velocity])

        # terminal state constraints
        ocp.constraints.idxbx_e = np.array([3, 4])
        ocp.constraints.lbx_e = np.array([-self._robot_cfg.max_forward_velocity_e, -self._robot_cfg.max_angular_velocity_e])
        ocp.constraints.ubx_e = np.array([ self._robot_cfg.max_forward_velocity_e, self._robot_cfg.max_angular_velocity_e])


    def _setup_nominal_obj_func(self, ocp:AcadosOcp) -> None:
        nx = self._solver_cfg.nx
        nu = self._solver_cfg.nu
        # middle stage cost
        ocp.cost.cost_type = 'LINEAR_LS'
        cost_QR = scipy.linalg.block_diag(self._solver_cfg.Q, self._solver_cfg.R)
        ocp.cost.W = cost_QR
        ocp.cost.Vx = np.zeros((nu+5, nx))
        ocp.cost.Vx[:5, :5] = np.eye(5)
        ocp.cost.Vu = np.zeros((nu+5, nu))
        ocp.cost.Vu[5:, :] = np.eye(nu)
        ocp.cost.yref  = np.zeros((nu+5, ))
        # terminal stage cost = ||x_e - x_ref_e||_{Q_e}^2
        ocp.cost.cost_type_e = 'LINEAR_LS'
        ocp.cost.W_e    = self._solver_cfg.Q * self._solver_cfg.delta_t  # NOTE: multiplied by delta_t
        ocp.cost.Vx_e   = np.zeros((5, nx))
        ocp.cost.Vx_e[:5, :5] = np.eye(5)
        ocp.cost.yref_e = np.zeros((5, ))


    def _setup_input_constraints(self, ocp: AcadosOcp) -> None:
        nu = self._solver_cfg.nu
        ocp.constraints.idxbu = np.arange(nu)
        lbu = np.array([self._robot_cfg.min_forward_acceleration, -self._robot_cfg.max_angular_acceleration])
        ubu = np.array([self._robot_cfg.max_forward_acceleration,  self._robot_cfg.max_angular_acceleration])
        ocp.constraints.lbu = lbu
        ocp.constraints.ubu = ubu


    def _define_robot_model_and_h_constr(self, flag_ode_with_sys_dyn) -> None:
        x = ca.MX.sym('x', self._solver_cfg.nx)
        u = ca.MX.sym('u', self._solver_cfg.nu)

        model = DifferentialDriveRobot(self._solver_cfg.delta_t)
        print("delta_t in robot model", self._solver_cfg.delta_t)
        f_expl = model.export_robot_sys_dyn_xdot(x, u, flag_ode_with_sys_dyn)
        xdot = ca.MX.sym('xdot', self._solver_cfg.nx)
        f_impl = xdot - f_expl

        model = AcadosModel()
        model.f_expl_expr = f_expl
        model.f_impl_expr = f_impl
        model.x = x
        model.xdot = xdot
        model.u = u
        model.name = "diff_drive_robot"

        return model


    def setup_OCP(self, flag_ode_with_sys_dyn: bool = True) -> None:
        # create ocp object to formulate the OCP
        ocp = AcadosOcp()
        ocp.model = self._define_robot_model_and_h_constr(flag_ode_with_sys_dyn)

        ocp.solver_options.N_horizon = self._solver_cfg.n_hrzn
        if self._solver_cfg.shooting_nodes.size == self._solver_cfg.n_hrzn+1:
            ocp.solver_options.shooting_nodes = self._solver_cfg.shooting_nodes
            ocp.solver_options.tf = self._solver_cfg.shooting_nodes[-1]
            sim_num_steps = np.zeros((self._solver_cfg.n_hrzn, ), dtype=np.int32)
            for idx_hrzn in range(self._solver_cfg.n_hrzn):
                if (self._solver_cfg.shooting_nodes[idx_hrzn+1] - self._solver_cfg.shooting_nodes[idx_hrzn] <= self._solver_cfg.delta_t * 0.99):
                    raise Exception(f"Time step {self._solver_cfg.shooting_nodes[idx_hrzn+1] - self._solver_cfg.shooting_nodes[idx_hrzn]} should be no smaller than config.delta_t = {self._solver_cfg.delta_t}")
                sim_num_steps[idx_hrzn] = int(round((self._solver_cfg.shooting_nodes[idx_hrzn+1] - self._solver_cfg.shooting_nodes[idx_hrzn]) / self._solver_cfg.delta_t))
            print(sim_num_steps)
            ocp.solver_options.sim_method_num_steps = sim_num_steps
        else:
            ocp.solver_options.tf = self._solver_cfg.n_hrzn * self._solver_cfg.delta_t

        self._setup_nominal_obj_func(ocp)
        self._setup_linear_state_constraints(ocp)
        self._setup_input_constraints(ocp)

        # set constraints
        C = np.ones((self._solver_cfg.constr_max_num, self._solver_cfg.nx))
        D = np.zeros((self._solver_cfg.constr_max_num, self._solver_cfg.nu))
        lg = -1e4 * np.ones((self._solver_cfg.constr_max_num, ))
        ug =  1e4 * np.ones((self._solver_cfg.constr_max_num, ))
        ocp.constraints.C = C
        ocp.constraints.D = D
        ocp.constraints.lg = lg
        ocp.constraints.ug = ug
        ocp.constraints.C_e = C
        ocp.constraints.lg_e = lg
        ocp.constraints.ug_e = ug
        ocp.constraints.idxsg = np.arange(0, self._solver_cfg.constr_max_num)
        ocp.constraints.idxsg_e = np.arange(0, self._solver_cfg.constr_max_num)

        ocp.cost.Zl   = self._solver_cfg.slack_l2_penalty * np.ones((self._solver_cfg.constr_max_num, ))
        ocp.cost.Zu   = self._solver_cfg.slack_l2_penalty * np.ones((self._solver_cfg.constr_max_num, ))
        ocp.cost.zl   = self._solver_cfg.slack_l1_penalty * np.ones((self._solver_cfg.constr_max_num, ))
        ocp.cost.zu   = self._solver_cfg.slack_l1_penalty * np.ones((self._solver_cfg.constr_max_num, ))
        ocp.cost.Zl_e = self._solver_cfg.slack_l2_penalty * np.ones((self._solver_cfg.constr_max_num, ))
        ocp.cost.Zu_e = self._solver_cfg.slack_l2_penalty * np.ones((self._solver_cfg.constr_max_num, ))
        ocp.cost.zl_e = self._solver_cfg.slack_l1_penalty * np.ones((self._solver_cfg.constr_max_num, ))
        ocp.cost.zu_e = self._solver_cfg.slack_l1_penalty * np.ones((self._solver_cfg.constr_max_num, ))

        # set options
        ocp.solver_options.qp_solver = 'PARTIAL_CONDENSING_HPIPM'
        ocp.solver_options.hessian_approx = 'GAUSS_NEWTON' # 'GAUSS_NEWTON', 'EXACT'
        ocp.solver_options.integrator_type = 'ERK'
        ocp.solver_options.nlp_solver_type = 'SQP'         # SQP_RTI, SQP
        ocp.solver_options.print_level = 0
        ocp.solver_options.levenberg_marquardt = 1e-7
        ocp.solver_options.nlp_solver_max_iter = 4
        ocp.solver_options.sim_method_num_stages = 4
        ocp.solver_options.sim_method_jac_reuse = 1
        ocp.code_export_directory = 'c_generated_code_sipoc_mr'

        self.ocp_solver = AcadosOcpSolver(ocp, json_file = 'acados_sipoc_mr.json')


if __name__ == "__main__":
    import argparse
    parser = argparse.ArgumentParser()
    parser.add_argument("--ode_with_sys_dyn", help="include system dynamics in the ODE.", action='store_true')
    parser.add_argument("--no_ode_with_sys_dyn", help="exclude system dynamics in the ODE.", dest='ode_with_sys_dyn', action='store_false')
    parser.add_argument("--numerical_eval", help="Generate code for numerical evaluation.", action='store_true')
    parser.add_argument("--real_time_control", help="Generate code for real-time control.", action='store_true')
    parser.set_defaults(ode_with_sys_dyn=True, numerical_eval=False, real_time_control=False)
    args = parser.parse_args()
    if not args.numerical_eval and not args.real_time_control:
        raise ValueError("Please specify either --numerical_eval or --real_time_control.")
    if args.numerical_eval and args.real_time_control:
        raise ValueError("Please specify either --numerical_eval or --real_time_control, not both.")
    if args.numerical_eval:
        print(f"Generating code for numerical evaluation with ode_with_sys_dyn = {args.ode_with_sys_dyn}.")
    if args.real_time_control:
        print(f"Generating code for real-time control with ode_with_sys_dyn = {args.ode_with_sys_dyn}.")

    solver_cfg = SipocSolverConfig()
    if args.numerical_eval:
        # NOTE: Uniform time interval for the numerical tests
        solver_cfg.delta_t = 0.2
        solver_cfg.n_hrzn  = 30
        solver_cfg.constr_max_num = 25
        solver_cfg.nx = 9
    else:
        time_steps = np.array([0.] + [0.05]*4 + [0.1]*10 + [0.2]*6)
        solver_cfg.shooting_nodes = np.cumsum(time_steps)
        solver_cfg.delta_t = 0.05  # NOTE: used for sys-id discretization and computation of num_steps
        solver_cfg.n_hrzn  = 20
        solver_cfg.constr_max_num = 8
        solver_cfg.nx = 9

    robot_cfg = RobotConfig()
    robot_cfg.robot_capsule_half_length = 0.5
    robot_cfg.robot_capsule_radius = 0.45
    robot_cfg.robot_polygon_radius = 0.2

    solver = SipocMRNLPSubproblem(robot_cfg=robot_cfg, solver_cfg=solver_cfg)
    solver.setup_OCP(args.ode_with_sys_dyn)
