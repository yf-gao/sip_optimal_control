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


import os
import sys
from ament_index_python.packages import get_package_prefix, get_package_share_directory
utils_prefix_path = get_package_prefix("sipoc_mr_utils")
lib_path = os.path.join(utils_prefix_path, 'lib')
sys.path.append(lib_path)
solver_prefix_path = get_package_prefix("sipoc_mr_solver")
lib_path = os.path.join(solver_prefix_path, 'lib')
sys.path.append(lib_path)
support_share_path = get_package_share_directory("sipoc_mr_support")

from dataclasses import dataclass
from enum import Enum
import matplotlib.pyplot as plt
import numpy as np

from sipoc_mr_support.configs import WorldConfig, RobotConfig, SipocSolverConfig
from sipoc_mr_support.generate_map import WorldMap
from sipoc_plot_utils.plot_utils import draw_capsule_arch_line_on_ax

from py_sipoc_mr_utils import py_vel_profile_optimizer, py_sd_map
from py_sipoc_mr_solver import py_capsule_nominal_controller, py_solver_status

@dataclass
class SIPResult:
    x_sol: np.ndarray
    u_sol: np.ndarray
    obs_sol: np.ndarray
    plane_sol: np.ndarray
    gamma_sol: np.ndarray
    sl_sol: np.ndarray
    num_constr: int


class WarmStartStrategy(Enum):
    ENDPOINTS    = 1
    COLD_START   = 2
    SOLN_LAST_TS = 3


def test(world_type: str):
    world_dist = {
        "L_corridor":
        {
            "sdf_file_str": os.path.join(support_share_path, "maps", "L_corridor_occupancy_map.csv"),
            "lb_px_grid": -2.99,
            "lb_py_grid": -2.99,
            "grid_size": 0.02,
            "map_height": 300,
            "map_width": 300,
            "current_state": np.array([-1.2, 1.13, 0., 0.05, -np.pi/10]),
            "waypoints": np.array([[-1.2, 1.13], [0.3, 0.9], [0.8, 0.6], [0.9, -1.5]]),
        },
        "docking_station":
        {
            "sdf_file_str": os.path.join(support_share_path, "maps", "docking_station_occupancy_map.csv"),
            "lb_px_grid": -2.99,
            "lb_py_grid": -2.99,
            "grid_size": 0.02,
            "map_height": 300,
            "map_width": 300,
            "current_state": np.array([-1.2, 1.13, -np.pi/6, 0.05, -np.pi/10]),
            "waypoints": np.array([[-1.2, 1.13], [0.0, 0.3], [1.5, 0.05]]),
        },
        "s_corridor": {
            "sdf_file_str": os.path.join(support_share_path, "maps", "s_corridor_occupancy_map.csv"),
            "lb_px_grid": -7.99,
            "lb_py_grid": -1.99,
            "grid_size": 0.02,
            "map_height": 200,
            "map_width": 800,
            "current_state": np.array([-6.0, 0.62, -np.pi/6, 0.05, -np.pi/10]),
            "waypoints": np.array([[-6.0, 0.62], [-1., -1.0], [ 2.0,  0.3], [ 2.9,  0.6], [ 5.8,  0.6]]),
        },
        "walkway": {
            "sdf_file_str": os.path.join(support_share_path, "maps", "walkway_occupancy_map.csv"),
            "lb_px_grid": -7.99,
            "lb_py_grid": -1.99,
            "grid_size": 0.02,
            "map_height": 200,
            "map_width": 800,
            "current_state": np.array([-6.0, 0.62, -np.pi/6, 0.05, -np.pi/10]),
            "waypoints": np.array([[-6.0, 0.62], [-4.6, -0.36], [-0.26, -0.4], [ 1.0, -0.0], [ 5.0,  0.2]]),
        }
    }
    sdf_file_str = world_dist[world_type]["sdf_file_str"]
    sdf_param = py_sd_map.SDFParam()
    sdf_param.lb_px_grid = world_dist[world_type]["lb_px_grid"]
    sdf_param.lb_py_grid = world_dist[world_type]["lb_py_grid"]
    sdf_param.grid_size = world_dist[world_type]["grid_size"]
    sdf_param.map_height = world_dist[world_type]["map_height"]
    sdf_param.map_width = world_dist[world_type]["map_width"]
    sdf_param.map_occ_threshold = 90

    world_cfg = WorldConfig()
    world_cfg.world_type = world_type
    world_map = WorldMap(world_cfg)
    robot_cfg = RobotConfig()
    robot_cfg.robot_capsule_half_length = 0.5
    robot_cfg.robot_capsule_radius = 0.4
    solver_cfg = SipocSolverConfig()
    solver_cfg.delta_t = 0.2
    solver_cfg.n_hrzn  = 30
    solver_cfg.constr_max_num = 25
    solver_cfg.nx = 9

    ref_gen = py_vel_profile_optimizer.VelocityProfileOptimizer()
    solver_cfg.shooting_nodes = np.cumsum(np.array([0.0] + [solver_cfg.delta_t] * solver_cfg.n_hrzn))
    ref_gen_time_scaling = 1.0 if world_type == "L_corridor" or world_type == "docking_station" else 1.25 # Scale by 1.25 such that the reference can reach the end
    ref_gen.configureOptimizer(solver_cfg.shooting_nodes * ref_gen_time_scaling)
    ref_gen.setRefPath(world_dist[world_type]["waypoints"].flatten())
    robot_cur_state = world_dist[world_type]["current_state"]
    ref_traj_pybind_vector = ref_gen.computeReferenceTraj(robot_cur_state)
    ref_traj = np.array(ref_traj_pybind_vector, dtype=np.float64).reshape((solver_cfg.n_hrzn+1, -1), order='C')

    color_cycle_list = ["tab:green", "tab:red", "tab:purple", "tab:brown", "tab:pink", "tab:gray", "tab:cyan"]
    def animate_solution(result: SIPResult, num_hrzn: int):
        if world_type == "L_corridor" or world_type == "docking_station":
            fig = plt.figure(17, figsize=(12, 12))
        else:
            fig = plt.figure(17, figsize=(24, 6))
        ax = fig.subplots(1, 1)
        ax.set_xlim(world_map.px_lsp[0], world_map.px_lsp[-1])
        ax.set_ylim(world_map.py_lsp[0], world_map.py_lsp[-1])
        for idx_hrzn in range(num_hrzn + 1):
            ax.clear()
            ax.plot(ref_traj[:, 0], ref_traj[:, 1], linewidth=2.0, color="tab:brown", alpha=0.3, linestyle="dashed", label = "Ref. Traj.")
            ax.imshow(world_map.occupancy_map.T, cmap='gray', vmin=0, vmax=1, origin='lower', extent=(world_map.px_lsp[0], world_map.px_lsp[-1], world_map.py_lsp[0], world_map.py_lsp[-1]), aspect=1)
            num_considered_obs = 0
            while (num_considered_obs < result.num_constr[idx_hrzn]):
                if np.allclose(result.obs_sol[idx_hrzn, 2*num_considered_obs:], 0.) and np.allclose(result.plane_sol[idx_hrzn, 2*num_considered_obs:], 0.) and np.allclose(result.gamma_sol[idx_hrzn, num_considered_obs:], 0.):
                    break
                num_considered_obs += 1
            ax.scatter(x=result.obs_sol[idx_hrzn, 0:2*num_considered_obs:2], y=result.obs_sol[idx_hrzn, 1:2*num_considered_obs:2], c="tab:orange", marker='x', label='Critical Obs.', linewidths=5, s=100)
            for ii in range(num_considered_obs):
                ax.quiver(result.obs_sol[idx_hrzn, 2*ii], result.obs_sol[idx_hrzn, 2*ii+1], result.plane_sol[idx_hrzn, 2*ii], result.plane_sol[idx_hrzn, 2*ii+1], color=color_cycle_list[ii%len(color_cycle_list)], scale=20)
                ax.scatter(result.x_sol[idx_hrzn, 0]+result.gamma_sol[idx_hrzn, ii]*np.cos(result.x_sol[idx_hrzn, 2]), result.x_sol[idx_hrzn, 1]+result.gamma_sol[idx_hrzn, ii]*np.sin(result.x_sol[idx_hrzn, 2]), color=color_cycle_list[ii%len(color_cycle_list)], marker='v', s=50)
            draw_capsule_arch_line_on_ax(ax, result.x_sol[[idx_hrzn], :], (robot_cfg.robot_capsule_radius, robot_cfg.robot_capsule_half_length), color='tab:blue', plot_capsule_center_line=True)
            ax.legend(fontsize=16)
            title = f"idx_hrzn={idx_hrzn}"
            ax.set_title(title, fontsize=16)
            ax.set_xticklabels([])
            ax.set_yticklabels([])
            plt.draw()
            plt.pause(0.6)
            # plt.savefig(f"fig_capsule_{world_type}_{idx_hrzn:03d}.png", dpi=200)
        plt.close(fig)

    ws_strategy = WarmStartStrategy.ENDPOINTS
    foo = py_capsule_nominal_controller.CapsuleNominalController(solver_cfg.n_hrzn, solver_cfg.delta_t)
    def solve_cpp():
        yref = np.hstack((ref_traj[:-1, :], np.zeros((solver_cfg.n_hrzn, 2))))
        yref_e = ref_traj[-1, :].copy()
        foo.setupOcpSolver()
        foo.loadOccAndUpdateSDF(sdf_file_str, sdf_param)
        foo.updateInitialState(robot_cur_state)
        foo.updateReferenceTrajectory(np.hstack((yref.flatten(), yref_e)))
        foo.initializeOCPByReference(False)
        if ws_strategy == WarmStartStrategy.COLD_START:
            status = foo.solveOCP(if_reset_num_constr=True)
            print(f"Solver status={status}")
        elif ws_strategy == WarmStartStrategy.ENDPOINTS:
            foo.solveEndpointGammaConstrainedSubproblem()
            status = foo.solveOCP(if_reset_num_constr=False)
            print(f"Solver status={status}")
        else:
            raise NotImplementedError
    def get_cpp_solution():
        x_sol_cpp = foo.getXSol().reshape((solver_cfg.n_hrzn+1, -1))
        u_sol_cpp = foo.getUSol().reshape((solver_cfg.n_hrzn, -1))
        u_sol_cpp = np.vstack((u_sol_cpp, np.zeros((1, 2))))
        obs_sol_cpp = foo.getObsSol().reshape((solver_cfg.n_hrzn+1, -1))
        plane_sol_cpp = foo.getPlaneSol().reshape((solver_cfg.n_hrzn+1, -1))
        gamma_sol_cpp = foo.getGammaSol().reshape((solver_cfg.n_hrzn+1, -1))
        assert gamma_sol_cpp.shape[1] == solver_cfg.constr_max_num, f"gamma_sol_cpp shape={gamma_sol_cpp.shape}, solver_cfg.constr_max_num={solver_cfg.constr_max_num}"
        sl_sol_cpp = foo.getSlackSol().reshape((solver_cfg.n_hrzn+1, -1))
        solution_over_horizon_cpp = SIPResult(x_sol_cpp, u_sol_cpp, obs_sol_cpp, plane_sol_cpp, gamma_sol_cpp, sl_sol_cpp, num_constr=solver_cfg.constr_max_num * np.ones((solver_cfg.n_hrzn+1)).astype(np.int32))
        return solution_over_horizon_cpp
    solve_cpp()
    solution_over_horizon_cpp = get_cpp_solution()
    animate_solution(solution_over_horizon_cpp, solver_cfg.n_hrzn)


if __name__ == "__main__":
    import argparse
    parser = argparse.ArgumentParser()
    parser.add_argument('--world', type=str, default='walkway', help='Type of the world to use')
    args = parser.parse_args()
    if args.world in ["L_corridor", "docking_station", "s_corridor", "walkway"]:
        world_type = args.world
    else:
        raise ValueError(f"Unsupported world type {args.world}, supported types are: L_corridor, docking_station, s_corridor, walkway")
    test(world_type)
