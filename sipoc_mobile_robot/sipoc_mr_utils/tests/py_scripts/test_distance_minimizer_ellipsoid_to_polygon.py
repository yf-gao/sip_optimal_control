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

from py_sipoc_mr_utils import py_distance_ellip_polygon
from sipoc_plot_utils.plot_utils import mat2ellip


def debug_plot(vertices, robot_yaw, p_obs, Sigma_p, casadi_sol, cpp_sol):
    rot_mat = np.array([[np.cos(robot_yaw), -np.sin(robot_yaw)], [np.sin(robot_yaw), np.cos(robot_yaw)]])
    vertices_rotated = rot_mat @ vertices.T
    casadi_gamma_p_rotated = rot_mat @ casadi_sol[:2, np.newaxis]
    cpp_gamma_p_rotated = rot_mat @ cpp_sol[:2, np.newaxis]
    print(f"p_obs:{p_obs}")
    print(f"casadi_sol:{casadi_sol}")
    print(f"cpp_sol:{cpp_sol}")
    fig, ax = plt.subplots()
    ax.add_patch(patches.Polygon(vertices_rotated.T, closed=True, fill=False))
    width, height, angle = mat2ellip(Sigma_p)
    ellipsoid = patches.Ellipse((p_obs[0], p_obs[1]), width*2, height*2, angle=angle, fill=False, zorder=20, linewidth=2, color="tab:cyan")
    ax.add_patch(ellipsoid)
    ax.scatter(p_obs[0], p_obs[1], color="tab:red", marker="o")
    ax.scatter(casadi_gamma_p_rotated[0], casadi_gamma_p_rotated[1], color="tab:blue", marker="D", linewidth=2, s=40, label="casadi polygon")
    ax.scatter(cpp_gamma_p_rotated[0], cpp_gamma_p_rotated[1], color="tab:purple", marker="D", linewidth=2, s=40, label="cpp polygon")
    ax.scatter(-casadi_sol[2] + p_obs[0], -casadi_sol[3] + p_obs[1], color="tab:blue", marker="v", linewidth=2, s=50, label="casadi ellipsoid")
    ax.scatter(-cpp_sol[2] + p_obs[0], -cpp_sol[3] + p_obs[1], color="tab:purple", marker="v", linewidth=2, s=50, label="cpp ellipsoid")
    ax.set_aspect('equal')
    ax.legend()
    plt.show()


def test_correctness(vertices: np.ndarray):
    print(vertices.flatten())
    num_vertices = vertices.shape[0]
    if num_vertices < 3 or vertices.shape[1] != 2:
        raise ValueError("Vertices must be a 2D array with at least 3 points and 2 coordinates each.")
    foo = py_distance_ellip_polygon.DistanceMinimizerEllipsoidToPolygon(vertices.flatten())

    def export_casadi_solver():
        weights = ca.MX.sym("gamma", num_vertices)
        gamma_ellipsoid = ca.MX.sym("gamma_ellipsoid", 2)
        param_obs = ca.MX.sym("param_obs", 2)
        robot_yaw = ca.MX.sym("robot_yaw")
        rotation_matrix = ca.MX(2, 2)
        rotation_matrix[0, 0] = ca.cos(robot_yaw)
        rotation_matrix[0, 1] = -ca.sin(robot_yaw)
        rotation_matrix[1, 0] = ca.sin(robot_yaw)
        rotation_matrix[1, 1] = ca.cos(robot_yaw)
        param_Sigma = ca.MX.sym('param_Sigma', 2, 2)
        vec = rotation_matrix @ (weights.T @ vertices).T + gamma_ellipsoid - param_obs
        f = ca.dot(vec, vec)
        g = ca.vertcat(weights, ca.sum1(weights), gamma_ellipsoid.T @ ca.inv(param_Sigma) @ gamma_ellipsoid - 1.0)
        ll_nlp = {
            'f': f,
            'x': ca.veccat(weights, gamma_ellipsoid),
            'p': ca.veccat(robot_yaw, param_obs, param_Sigma),
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

    num_test = 2000
    np.random.seed(0)
    random_p_obs = np.random.uniform(-2.5, 2.5, (num_test, 2))
    random_robot_yaw = np.random.uniform(-np.pi, np.pi, (num_test,))

    Sigma_p_axis_aligned = np.diag([0.4, 0.1])
    random_Sigma_theta_rotation = np.random.uniform(0.0, np.pi, (num_test,))

    skipped = 0
    test_passed = 0
    non_zero_dist = 0
    diff_list = []

    for idx_test in range(num_test):
        rr = random_Sigma_theta_rotation[idx_test]
        Sigma_theta_rot_matrix = np.array([[np.cos(rr), -np.sin(rr)],
                                             [np.sin(rr), np.cos(rr)]])
        Sigma_p = Sigma_theta_rot_matrix @ Sigma_p_axis_aligned @ Sigma_theta_rot_matrix.T

        cpp_solver_param = np.zeros((7,))
        cpp_solver_param[0] = Sigma_p[0, 0]
        cpp_solver_param[1] = Sigma_p[0, 1]
        cpp_solver_param[2] = Sigma_p[1, 0]
        cpp_solver_param[3] = Sigma_p[1, 1]
        cpp_solver_param[4] = random_p_obs[idx_test, 0]
        cpp_solver_param[5] = random_p_obs[idx_test, 1]
        cpp_solver_param[6] = random_robot_yaw[idx_test]


        res = casadi_solver(x0=ca.DM([1.0/num_vertices]*num_vertices + [0., 0.]),
                            p=ca.veccat(random_robot_yaw[idx_test], random_p_obs[idx_test, :], Sigma_p), 
                            lbg=[0.] * num_vertices + [1., -np.inf], ubg=[np.inf]*num_vertices + [1., 0.])
        if casadi_solver.stats()["return_status"] != "Solve_Succeeded":
            skipped += 1
            inside_by_cpp = foo.ifInsidePolygon(np.hstack((random_robot_yaw[idx_test], random_p_obs[idx_test, :])))
            print(f"casadi solver failed. inside_by_cpp:{inside_by_cpp}")
            continue
        if res["f"].full().flatten() > 1e-6:
            non_zero_dist += 1
            casadi_sol_raw = res["x"].full().flatten()
            casadi_sol_gamma_p = casadi_sol_raw[np.newaxis, :num_vertices] @ vertices
            casadi_sol = np.concatenate((casadi_sol_gamma_p.flatten(), casadi_sol_raw[num_vertices:]))
            cpp_sol = foo.argMinDistance(cpp_solver_param)
            cpp_sol_dist = cpp_sol[-1]
            cpp_sol = cpp_sol[:-1]
            casadi_sol_dist = res["f"].full().flatten()[0]
            diff_dist = np.abs(casadi_sol_dist - cpp_sol_dist)
            diff = np.linalg.norm(casadi_sol - cpp_sol, ord=np.inf)
            diff_list.append(diff)
            if diff < 5e-5 and diff_dist < 5e-5:
                test_passed += 1
            else:
                print(f"Test #{idx_test} failed. Diff={diff}. casadi_dist:{casadi_sol_dist}, cpp_dist:{cpp_sol_dist}")
                debug_plot(vertices, random_robot_yaw[idx_test], random_p_obs[idx_test, :], Sigma_p, casadi_sol, cpp_sol)

    print(f"non_zero_dist:{non_zero_dist}, test_passed:{test_passed}, skipped:{skipped}")
    print(f"max diff:{np.max(diff_list)}")


if __name__ == "__main__":
    vertices = np.array([[-0.18, -0.11], [0.45, -0.11], [0.45, 0.11], [-0.18, 0.11], [-0.33, 0.]])
    test_correctness(vertices)
