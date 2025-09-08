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

#include "sipoc_mr_solver/pybind/dilated_polygon_robust_controller_pybind.hpp"
#include <stdexcept>
#include <pybind11/stl.h>

namespace py = pybind11;

namespace sipoc_mr_solver
{
    py::array_t<double> DilatedPolygonRobustControllerPybind::getXSolPybind()
    {
        return py::array_t<double>(
            py::buffer_info(
                x_sol_,
                sizeof(double), // itemsize
                py::format_descriptor<double>::format(),
                1,                                                                        // ndim
                std::vector<size_t>{(params_.ocp_dims.n_hrzn + 1) * params_.ocp_dims.nx}, // shape
                std::vector<size_t>{sizeof(double)}                                       // strides
                ));
    }

    py::array_t<double> DilatedPolygonRobustControllerPybind::getUSolPybind()
    {
        return py::array_t<double>(
            py::buffer_info(
                u_sol_,
                sizeof(double), // itemsize
                py::format_descriptor<double>::format(),
                1,                                                                    // ndim
                std::vector<size_t>{(params_.ocp_dims.n_hrzn) * params_.ocp_dims.nu}, // shape
                std::vector<size_t>{sizeof(double)}                                   // strides
                ));
    }

    py::array_t<double> DilatedPolygonRobustControllerPybind::getSlackSolPybind()
    {
        return py::array_t<double>(
            py::buffer_info(
                sl_sol_,
                sizeof(double), // itemsize
                py::format_descriptor<double>::format(),
                1,                                                                                                                // ndim
                std::vector<size_t>{static_cast<unsigned int>((params_.ocp_dims.n_hrzn + 1) * params_.ocp_iters.max_constr_num)}, // shape
                std::vector<size_t>{sizeof(double)}                                                                               // strides
                ));
    };

    py::array_t<double> DilatedPolygonRobustControllerPybind::getObsSolPybind()
    {
        std::vector<double> obs_sol((params_.ocp_dims.n_hrzn + 1) * params_.ocp_iters.max_constr_num * 2);
        for (int idx_hrzn = 0; idx_hrzn <= params_.ocp_dims.n_hrzn; ++idx_hrzn)
        {
            for (unsigned int idx_constr = 0; idx_constr < params_.ocp_iters.max_constr_num; ++idx_constr)
            {
                obs_sol[2 * (idx_hrzn * params_.ocp_iters.max_constr_num + idx_constr)] = obs_gamma_star_sol_[idx_hrzn][idx_constr].obs_coords.x();
                obs_sol[2 * (idx_hrzn * params_.ocp_iters.max_constr_num + idx_constr) + 1] = obs_gamma_star_sol_[idx_hrzn][idx_constr].obs_coords.y();
            }
        }
        return py::array_t<double>(
            py::buffer_info(
                obs_sol.data(),
                sizeof(double), // itemsize
                py::format_descriptor<double>::format(),
                1,                                                                                                                    // ndim
                std::vector<size_t>{static_cast<unsigned int>((params_.ocp_dims.n_hrzn + 1) * params_.ocp_iters.max_constr_num) * 2}, // shape
                std::vector<size_t>{sizeof(double)}                                                                                   // strides
                ));
    };

    py::array_t<double> DilatedPolygonRobustControllerPybind::getPlaneSolPybind()
    {
        std::vector<double> plane_sol((params_.ocp_dims.n_hrzn + 1) * params_.ocp_iters.max_constr_num * 2);
        for (int idx_hrzn = 0; idx_hrzn <= params_.ocp_dims.n_hrzn; ++idx_hrzn)
        {
            for (unsigned int idx_constr = 0; idx_constr < params_.ocp_iters.max_constr_num; ++idx_constr)
            {
                plane_sol[2 * (idx_hrzn * params_.ocp_iters.max_constr_num + idx_constr)] = obs_gamma_star_sol_[idx_hrzn][idx_constr].sep_plane.x();
                plane_sol[2 * (idx_hrzn * params_.ocp_iters.max_constr_num + idx_constr) + 1] = obs_gamma_star_sol_[idx_hrzn][idx_constr].sep_plane.y();
            }
        }
        return py::array_t<double>(
            py::buffer_info(
                plane_sol.data(),
                sizeof(double), // itemsize
                py::format_descriptor<double>::format(),
                1,                                                                                                                    // ndim
                std::vector<size_t>{static_cast<unsigned int>((params_.ocp_dims.n_hrzn + 1) * params_.ocp_iters.max_constr_num) * 2}, // shape
                std::vector<size_t>{sizeof(double)}                                                                                   // strides
                ));
    };

    py::array_t<double> DilatedPolygonRobustControllerPybind::getGammaSolPybind()
    {
        std::vector<double> gamma_sol((params_.ocp_dims.n_hrzn + 1) * params_.ocp_iters.max_constr_num * 2);
        for (int idx_hrzn = 0; idx_hrzn <= params_.ocp_dims.n_hrzn; ++idx_hrzn)
        {
            for (unsigned int idx_constr = 0; idx_constr < params_.ocp_iters.max_constr_num; ++idx_constr)
            {
                gamma_sol[2 * (idx_hrzn * params_.ocp_iters.max_constr_num + idx_constr)] = obs_gamma_star_sol_[idx_hrzn][idx_constr].gamma_polygon.x();
                gamma_sol[2 * (idx_hrzn * params_.ocp_iters.max_constr_num + idx_constr) + 1] = obs_gamma_star_sol_[idx_hrzn][idx_constr].gamma_polygon.y();
            }
        }
        return py::array_t<double>(
            py::buffer_info(
                gamma_sol.data(),
                sizeof(double), // itemsize
                py::format_descriptor<double>::format(),
                1,                                                                                                                    // ndim
                std::vector<size_t>{static_cast<unsigned int>((params_.ocp_dims.n_hrzn + 1) * params_.ocp_iters.max_constr_num) * 2}, // shape
                std::vector<size_t>{sizeof(double)}                                                                                   // strides
                ));
    }

    py::array_t<double> DilatedPolygonRobustControllerPybind::getCovSolPybind()
    {
        for (unsigned int ii = 0; ii <= static_cast<unsigned int>(params_.ocp_dims.n_hrzn); ++ii)
        {
            unsigned int idx_start = params_.ocp_dims.nx * params_.ocp_dims.nx * ii;
            for (unsigned int idx_row = 0; idx_row < params_.ocp_dims.nx; ++idx_row)
            {
                for (unsigned int idx_col = 0; idx_col < params_.ocp_dims.nx; ++idx_col)
                    pybind_cov_sol_[idx_start + idx_row * params_.ocp_dims.nx + idx_col] = ptr_unc_propagation_->unc_matrices[ii].coeff(idx_row, idx_col);
            }
        }
        return py::array_t<double>(
            py::buffer_info(
                pybind_cov_sol_,
                sizeof(double), // itemsize
                py::format_descriptor<double>::format(),
                1,                                                                                                                         // ndim
                std::vector<size_t>{static_cast<unsigned int>((params_.ocp_dims.n_hrzn + 1) * params_.ocp_dims.nx * params_.ocp_dims.nx)}, // shape
                std::vector<size_t>{sizeof(double)}                                                                                        // strides
                ));
    }

    py::array_t<double> DilatedPolygonRobustControllerPybind::getNumConstrPybind()
    {
        return py::array_t<double>(
            py::buffer_info(
                num_constr_each_time_step_,
                sizeof(int), // itemsize
                py::format_descriptor<int>::format(),
                1,                                                                           // ndim
                std::vector<size_t>{static_cast<unsigned int>(params_.ocp_dims.n_hrzn + 1)}, // shape
                std::vector<size_t>{sizeof(int)}                                             // strides
                ));
    }

    void DilatedPolygonRobustControllerPybind::updateInitialStatePybind(py::array_t<double> initial_state)
    {
        if (initial_state.shape(0) != 5)
        {
            throw std::invalid_argument("The shape of initial state is NOT equal to (5,).");
        }
        auto ptr = initial_state.mutable_data();

        manif::SE2<double> current_pose(ptr[0], ptr[1], ptr[2]);
        manif::SE2Tangent<double> current_ref_vel(ptr[3], 0., ptr[4]);
        updateInitialState(current_pose, current_ref_vel);
    };

    void DilatedPolygonRobustControllerPybind::updateReferenceTrajectoryPybind(py::array_t<double> ref_traj)
    {
        if (ref_traj.shape(0) != (params_.ocp_dims.n_hrzn * params_.ocp_dims.ny + params_.ocp_dims.nyn))
        {
            throw std::invalid_argument("The shape of ref_traj does not match solver dimensions.");
        }
        auto ptr = ref_traj.mutable_data();
        updateReferenceTrajectory(ptr, ptr + params_.ocp_dims.n_hrzn * params_.ocp_dims.ny);
        return;
    }

    bool DilatedPolygonRobustControllerPybind::solveOCPSubproblemPybind(int idx_iter, bool if_reset_num_constr)
    {
        if (if_reset_num_constr)
        {
            std::fill_n(num_constr_each_time_step_, params_.ocp_dims.n_hrzn + 1, 0);
        }
        bool status_subproblem = solveOCPSubproblem(idx_iter);
        std::memcpy(x_sol_, x_sol_new_iter_, (params_.ocp_dims.n_hrzn + 1) * params_.ocp_dims.nx * sizeof(double));
        std::memcpy(u_sol_, u_sol_new_iter_, params_.ocp_dims.n_hrzn * params_.ocp_dims.nu * sizeof(double));
        return status_subproblem;
    }

    void DilatedPolygonRobustControllerPybind::initializeOCPByReferencePybind(bool if_reset_acados)
    {
        if (if_reset_acados)
        {
            diff_drive_robot_acados_reset(acados_ocp_capsule_, 1);
        }
        initializeOCPByReference();
    }

    sipoc_mr_utils::SIPSolverStatus DilatedPolygonRobustControllerPybind::reInitializeAndSolveMultipleTimesPybind(int num_solver_call, bool if_reset_num_constr)
    {
        min_elapsed_time_.lower_level = std::numeric_limits<double>::max();
        min_elapsed_time_.upper_level = std::numeric_limits<double>::max();
        min_elapsed_time_.prop_disturb = std::numeric_limits<double>::max();
        min_elapsed_time_.update_obs_subset = std::numeric_limits<double>::max();
        // double debug_x_sol[30];
        sipoc_mr_utils::SIPSolverStatus status = sipoc_mr_utils::SIPSolverStatus::acadosFail;
        for (int idx = 0; idx < num_solver_call; ++idx)
        {
            resetElapsedTime();
            diff_drive_robot_acados_reset(acados_ocp_capsule_, 1);
            initializeOCPByReference();
            solveEndpointGammaConstrainedSubproblem();
            status = solveOCP(if_reset_num_constr);
            min_elapsed_time_.lower_level = std::min(min_elapsed_time_.lower_level, elapsed_time_.lower_level);
            min_elapsed_time_.upper_level = std::min(min_elapsed_time_.upper_level, elapsed_time_.upper_level);
            min_elapsed_time_.prop_disturb = std::min(min_elapsed_time_.prop_disturb, elapsed_time_.prop_disturb);
            min_elapsed_time_.update_obs_subset = std::min(min_elapsed_time_.update_obs_subset, elapsed_time_.update_obs_subset);
        }
        return status;
    }
}

PYBIND11_MODULE(py_dilated_polygon_robust_controller, m)
{
    py::class_<sipoc_mr_solver::DilatedPolygonRobustControllerPybind>(m, "DilatedPolygonRobustController")
        .def(py::init<>())
        .def(py::init<int, double>(), py::arg("n_hrzn"), py::arg("delta_t"))
        .def(py::init<int, double, double>(), py::arg("n_hrzn"), py::arg("delta_t"), py::arg("polygon_dilation_radius"))
        .def("setPolygonVertices", &sipoc_mr_solver::DilatedPolygonRobustControllerPybind::setPolygonVerticesPybind)
        .def("setupOcpSolver", &sipoc_mr_solver::DilatedPolygonRobustControllerPybind::setupOcpSolver)
        .def("loadOccAndUpdateSDF", &sipoc_mr_solver::DilatedPolygonRobustControllerPybind::loadOccAndUpdateSDF)
        .def("updateInitialState", &sipoc_mr_solver::DilatedPolygonRobustControllerPybind::updateInitialStatePybind)
        .def("updateReferenceTrajectory", &sipoc_mr_solver::DilatedPolygonRobustControllerPybind::updateReferenceTrajectoryPybind)
        .def("initializeOCPByReference", &sipoc_mr_solver::DilatedPolygonRobustControllerPybind::initializeOCPByReferencePybind, py::arg("if_reset_acados"))
        .def("solveEndpointGammaConstrainedSubproblem", &sipoc_mr_solver::DilatedPolygonRobustControllerPybind::solveEndpointGammaConstrainedSubproblem)
        .def("shiftInputStateTrajectory", &sipoc_mr_solver::DilatedPolygonRobustControllerPybind::shiftInputStateTrajectory)
        .def("gatherActiveConstraintsAndShiftOneStep", &sipoc_mr_solver::DilatedPolygonRobustControllerPybind::gatherActiveConstraintsAndShiftOneStep)
        .def("solveOCPSubproblem", &sipoc_mr_solver::DilatedPolygonRobustControllerPybind::solveOCPSubproblemPybind, py::arg("idx_iter"), py::arg("if_reset_num_constr"))
        .def("solveOCP", &sipoc_mr_solver::DilatedPolygonRobustControllerPybind::solveOCP, py::arg("if_reset_num_constr"))
        .def("reInitializeAndSolveMultipleTimes", &sipoc_mr_solver::DilatedPolygonRobustControllerPybind::reInitializeAndSolveMultipleTimesPybind, py::arg("num_solver_call"), py::arg("if_reset_num_constr"))
        .def("getXSol", &sipoc_mr_solver::DilatedPolygonRobustControllerPybind::getXSolPybind)
        .def("getUSol", &sipoc_mr_solver::DilatedPolygonRobustControllerPybind::getUSolPybind)
        .def("getSlackSol", &sipoc_mr_solver::DilatedPolygonRobustControllerPybind::getSlackSolPybind)
        .def("getObsSol", &sipoc_mr_solver::DilatedPolygonRobustControllerPybind::getObsSolPybind)
        .def("getPlaneSol", &sipoc_mr_solver::DilatedPolygonRobustControllerPybind::getPlaneSolPybind)
        .def("getGammaSol", &sipoc_mr_solver::DilatedPolygonRobustControllerPybind::getGammaSolPybind)
        .def("getCovSol", &sipoc_mr_solver::DilatedPolygonRobustControllerPybind::getCovSolPybind)
        .def("getNumConstr", &sipoc_mr_solver::DilatedPolygonRobustControllerPybind::getNumConstrPybind)
        .def("iterConverg", &sipoc_mr_solver::DilatedPolygonRobustControllerPybind::iterConverg)
        .def("lower_level_elapsed_time", &sipoc_mr_solver::DilatedPolygonRobustControllerPybind::lower_level_elapsed_time)
        .def("upper_level_elapsed_time", &sipoc_mr_solver::DilatedPolygonRobustControllerPybind::upper_level_elapsed_time)
        .def("min_lower_level_elapsed_time", &sipoc_mr_solver::DilatedPolygonRobustControllerPybind::min_lower_level_elapsed_timePybind)
        .def("min_upper_level_elapsed_time", &sipoc_mr_solver::DilatedPolygonRobustControllerPybind::min_upper_level_elapsed_timePybind)
        .def("min_update_obs_subset_elapsed_time", &sipoc_mr_solver::DilatedPolygonRobustControllerPybind::min_update_obs_subset_elapsed_timePybind)
        .def("min_prop_disturb_elapsed_time", &sipoc_mr_solver::DilatedPolygonRobustControllerPybind::min_prop_disturb_elapsed_timePybind)
        .def("__del__", [](sipoc_mr_solver::DilatedPolygonRobustControllerPybind *instance)
             {
            // Custom destructor is invoked explicitly
            delete instance; }, py::keep_alive<0, 1>());
}
