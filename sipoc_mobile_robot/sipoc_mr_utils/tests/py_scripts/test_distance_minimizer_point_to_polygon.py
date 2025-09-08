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


from ament_index_python.packages import get_package_prefix
import os
import sys
prefix_path = get_package_prefix("sipoc_mr_utils")
lib_path = os.path.join(prefix_path, 'lib')
sys.path.append(lib_path)

import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as patches
import casadi as ca

from py_sipoc_mr_utils import py_distance_point_polygon


def debug_plot(vertices, robot_yaw, p_obs, casadi_sol, cpp_sol):
    rot_mat = np.array([[np.cos(robot_yaw), -np.sin(robot_yaw)], [np.sin(robot_yaw), np.cos(robot_yaw)]])
    vertices_rotated = rot_mat @ vertices.T
    casadi_sol_rotated = rot_mat @ casadi_sol[:, np.newaxis]
    cpp_sol_rotated = rot_mat @ cpp_sol[:, np.newaxis]
    print(f"p_obs:{p_obs}")
    print(f"vertices_rotated:{vertices_rotated}")
    print(f"casadi_sol:{casadi_sol}")
    print(f"cpp_sol:{cpp_sol}")
    fig, ax = plt.subplots()
    ax.add_patch(patches.Polygon(vertices_rotated.T, closed=True, fill=False))
    ax.scatter(p_obs[0], p_obs[1], color="tab:red", marker="o")
    ax.scatter(casadi_sol_rotated[0], casadi_sol_rotated[1], color="tab:blue", marker="D", linewidth=2, s=40)
    ax.scatter(cpp_sol_rotated[0], cpp_sol_rotated[1], color="tab:green", marker="v", s=50)
    ax.set_aspect('equal')
    plt.show()


def test_correctness():
    vertices = np.array([[-0.18, -0.11], [0.45, -0.11], [0.45, 0.11], [-0.18, 0.11], [-0.33, 0.]])
    num_vertices = vertices.shape[0]
    foo = py_distance_point_polygon.DistanceMinimizerPointToPolygon(vertices.flatten())
    # NOTE: A is flattened (column-major)
    half_space_A = foo.halfSpaceA()
    half_space_b = foo.halfSpaceb()

    def export_casadi_solver():
        gamma = ca.MX.sym("gamma", 2)
        param_A = ca.MX.sym("param_A", num_vertices, 2)
        param_b = ca.MX.sym("param_b", num_vertices)
        param_obs = ca.MX.sym("param_obs", 2)
        robot_yaw = ca.MX.sym("robot_yaw")
        rotation_matrix = ca.MX(2, 2)
        rotation_matrix[0, 0] = ca.cos(robot_yaw)
        rotation_matrix[0, 1] = -ca.sin(robot_yaw)
        rotation_matrix[1, 0] = ca.sin(robot_yaw)
        rotation_matrix[1, 1] = ca.cos(robot_yaw)
        vec_r2o = param_obs - rotation_matrix @ gamma
        f = ca.sqrt(ca.dot(vec_r2o, vec_r2o))
        g = param_A @ gamma - param_b
        ll_nlp = {
            'f': f,
            'x': ca.veccat(gamma),
            'p': ca.veccat(robot_yaw, param_obs, param_A, param_b),
            'g': g,
        }
        options = {
            "print_time": False,
            "ipopt.check_derivatives_for_naninf": "yes",
            "ipopt.print_level": 1,
            "error_on_fail": False,
        }
        casadi_solver = ca.nlpsol("solver", "ipopt", ll_nlp, options)
        return casadi_solver
    casadi_solver = export_casadi_solver()

    num_test = 1000
    np.random.seed(0)
    random_p_obs = np.random.uniform(-2.5, 2.5, (num_test, 2))
    random_robot_yaw = np.random.uniform(-np.pi, np.pi, (num_test,))
    skipped = 0
    test_passed = 0
    non_zero_dist = 0
    diff_list = []

    for idx_test in range(num_test):
        cpp_solver_param = np.zeros((3,))
        cpp_solver_param[0] = random_p_obs[idx_test, 0]
        cpp_solver_param[1] = random_p_obs[idx_test, 1]
        cpp_solver_param[2] = random_robot_yaw[idx_test]

        res = casadi_solver(x0=ca.DM([0.0, 0.0]), p=ca.veccat(random_robot_yaw[idx_test], random_p_obs[idx_test, :], half_space_A, half_space_b), lbg=[-np.inf]*num_vertices, ubg=[0.0]*num_vertices)
        if casadi_solver.stats()["return_status"] != "Solve_Succeeded":
            skipped += 1
            inside_by_cpp = foo.ifInsidePolygon(np.hstack((random_robot_yaw[idx_test], random_p_obs[idx_test, :])))
            print(f"casadi solver failed. inside_by_cpp:{inside_by_cpp}")
            continue
        if res["f"].full().flatten() <= 1e-6:
            if foo.ifInsidePolygon(np.hstack((random_robot_yaw[idx_test], random_p_obs[idx_test, :]))):
                test_passed += 1
            else:
                print(f"idx_test:{idx_test}: casadi solver returned zero distance, but point is outside polygon.")
                debug_plot(vertices, random_robot_yaw[idx_test], random_p_obs[idx_test, :], res["x"].full().flatten(), np.zeros((2,)))
        else:
            non_zero_dist += 1
            casadi_sol = res["x"].full().flatten()
            cpp_sol = foo.argMinDistance(cpp_solver_param)
            diff = np.linalg.norm(casadi_sol - cpp_sol, ord=np.inf)
            diff_list.append(diff)
            if np.isclose(casadi_sol, cpp_sol, atol=1e-6).all():
                test_passed += 1
            else:
                print(f"idx_test:{idx_test}: casadi solver and cpp solver returned different solutions, diff:{diff}")
                debug_plot(vertices, random_robot_yaw[idx_test], random_p_obs[idx_test, :], casadi_sol, cpp_sol)

    print(f"non_zero_dist:{non_zero_dist}, test_passed:{test_passed}, skipped:{skipped}")
    print(f"max diff:{np.max(diff_list)}")


if __name__ == "__main__":
    test_correctness()
