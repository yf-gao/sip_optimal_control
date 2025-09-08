// Copyright (C) 2025 Robert Bosch GmbH
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU Affero General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU Affero General Public License for more details.
//
// You should have received a copy of the GNU Affero General Public License
// along with this program.  If not, see <http://www.gnu.org/licenses/>.
//
// -------------------------------------
// Author: Yunfan Gao
//

#include "sipoc_mr_solver/dilated_polygon_nominal_controller.hpp"
#include "sipoc_mr_utils/common_structs.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <manif/SE2.h>

#include <chrono>
#include <string>
#include <iostream>
#include <filesystem>

int main()
{
    using std::chrono::duration;
    using std::chrono::duration_cast;
    using std::chrono::high_resolution_clock;
    using std::chrono::microseconds;
    using std::chrono::milliseconds;

    sipoc_mr_utils::OCPParameters params;
    params.ocp_dims.n_hrzn = 30;
    params.ocp_iters.max_constr_num = 25;
    params.ocp_dims.nx = 9;
    params.robot.vertices_flattened = {-0.1, 0.2, -0.12, -0.05, 0.12, -0.05, 0.1, 0.2, 0., 0.35};

    sipoc_mr_solver::DilatedPolygonNominalController solver(params);
    solver.setupOcpSolver();

    std::string package_name = "sipoc_mr_support";
    std::filesystem::path map_csv_path = ament_index_cpp::get_package_share_directory(package_name);
    map_csv_path += "/maps/walkway_occupancy_map.csv";
    sipoc_mr_utils::SDFParam sdf_param;
    sdf_param.lb_px_grid = -7.99;
    sdf_param.lb_py_grid = -1.99;
    sdf_param.grid_size = 0.02;
    sdf_param.map_height = 200;
    sdf_param.map_width = 800;
    sdf_param.map_occ_threshold = 90;
    solver.loadOccAndUpdateSDF(map_csv_path.string(), sdf_param);
    manif::SE2<double> current_pose(0., 0., 0.1);
    manif::SE2Tangent<double> current_ref_vel(1., 0., 0.5);
    Eigen::Vector<double, 9> full_state;
    full_state << current_pose.x(), current_pose.y(), current_pose.angle(),
        current_ref_vel.x(), current_ref_vel.angle(), 0., 0., 0., 0.;

    int n_hrzn = params.ocp_dims.n_hrzn;
    int ny = 7;
    int nyn = 5;
    double *yref = (double *)malloc(n_hrzn * ny * sizeof(double));
    double *yref_e = (double *)malloc(nyn * sizeof(double));
    std::fill_n(yref, n_hrzn * ny, 0.1);
    std::fill_n(yref_e, nyn, 0.2);
    solver.updateReferenceTrajectory(yref, yref_e);
    free(yref);
    free(yref_e);

    int num_tests = 2;
    double total_t_initialization = 0.;
    double total_t_warmstart = 0.;
    double total_t_solveOCP = 0.;
    double total_t_lower_level = 0.;
    double total_t_upper_level = 0.;
    int num_iter = 0;
    for (int idx_test = 0; idx_test < num_tests; ++idx_test)
    {
        auto t1 = high_resolution_clock::now();
        solver.updateInitialState(full_state);
        auto t2 = high_resolution_clock::now();
        solver.initializeOCPByReference();
        auto t3 = high_resolution_clock::now();
        solver.resetElapsedTime();
        solver.solveEndpointGammaConstrainedSubproblem();
        solver.solveOCP((idx_test % 2) == 0);
        auto t4 = high_resolution_clock::now();

        auto Duration1 = duration_cast<microseconds>(t2 - t1);
        auto Duration2 = duration_cast<microseconds>(t3 - t2);
        auto Duration3 = duration_cast<microseconds>(t4 - t3);

        total_t_initialization += Duration1.count();
        total_t_warmstart += Duration2.count();
        total_t_solveOCP += Duration3.count();
        total_t_lower_level += solver.lower_level_elapsed_time();
        total_t_upper_level += solver.upper_level_elapsed_time();
        std::cout << "converged at " << solver.iterConverg() << std::endl;
        num_iter += solver.iterConverg();

        solver.shiftInputStateTrajectory(idx_test);
    }

    std::cout << "time_initialization:" << 1e-3 * total_t_initialization / num_tests << "ms" << std::endl;
    std::cout << "time_warmstart:" << 1e-3 * total_t_warmstart / num_tests << "ms" << std::endl;
    std::cout << "time_solveOCP:" << 1e-3 * total_t_solveOCP / num_tests << "ms" << std::endl;
    std::cout << "time_lower_level:" << 1e-3 * total_t_lower_level / num_tests << "ms" << std::endl;
    std::cout << "time_upper_level:" << 1e-3 * total_t_upper_level / num_tests << "ms" << std::endl;
    std::cout << "number iteration:" << num_iter << "." << std::endl;
}
