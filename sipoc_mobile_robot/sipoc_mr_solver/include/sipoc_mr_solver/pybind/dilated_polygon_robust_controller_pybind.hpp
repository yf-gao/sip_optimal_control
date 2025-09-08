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

#ifndef SIPOC_MR_CONTROLLER__DILATED_POLYGON_ROBUST_CONTROLLER_PYBIND_HPP_
#define SIPOC_MR_CONTROLLER__DILATED_POLYGON_ROBUST_CONTROLLER_PYBIND_HPP_

#include "sipoc_mr_solver/dilated_polygon_robust_controller.hpp"

#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>

namespace py = pybind11;

namespace sipoc_mr_solver
{
    class DilatedPolygonRobustControllerPybind : public DilatedPolygonRobustController
    {
    public:
        DilatedPolygonRobustControllerPybind() : DilatedPolygonRobustController()
        {
            pybind_cov_sol_ = (double *)malloc(sizeof(double) * (params_.ocp_dims.nx * params_.ocp_dims.nx * (static_cast<unsigned int>(params_.ocp_dims.n_hrzn) + 1)));
        }

        DilatedPolygonRobustControllerPybind(int n_hrzn, double delta_t) : DilatedPolygonRobustController(n_hrzn, delta_t)
        {
            pybind_cov_sol_ = (double *)malloc(sizeof(double) * (params_.ocp_dims.nx * params_.ocp_dims.nx * (static_cast<unsigned int>(params_.ocp_dims.n_hrzn) + 1)));
        }

        DilatedPolygonRobustControllerPybind(int n_hrzn, double delta_t, double robot_polygon_dilation_radius) : DilatedPolygonRobustController(n_hrzn, delta_t)
        {
            params_.robot.robot_polygon_dilation_radius = robot_polygon_dilation_radius;
            pybind_cov_sol_ = (double *)malloc(sizeof(double) * (params_.ocp_dims.nx * params_.ocp_dims.nx * (static_cast<unsigned int>(params_.ocp_dims.n_hrzn) + 1)));
        }

        ~DilatedPolygonRobustControllerPybind()
        {
            free(pybind_cov_sol_);
        }

        void updateInitialStatePybind(py::array_t<double> param);
        void updateReferenceTrajectoryPybind(py::array_t<double> param);
        void setConstraintParamsFromExternalPybind(py::array_t<double> constraint_params);
        bool solveOCPSubproblemPybind(int idx_iter, bool if_reset_num_constr);
        void initializeOCPByReferencePybind(bool if_reset_acados);
        sipoc_mr_utils::SIPSolverStatus reInitializeAndSolveMultipleTimesPybind(int num_solver_call, bool if_reset_num_constr);
        py::array_t<double> getXSolPybind();
        py::array_t<double> getUSolPybind();
        py::array_t<double> getSlackSolPybind();
        py::array_t<double> getObsSolPybind();
        py::array_t<double> getGammaSolPybind();
        py::array_t<double> getPlaneSolPybind();
        py::array_t<double> getCovSolPybind();
        py::array_t<double> getNumConstrPybind();
        inline double min_lower_level_elapsed_timePybind()
        {
            return min_elapsed_time_.lower_level;
        }
        inline double min_upper_level_elapsed_timePybind()
        {
            return min_elapsed_time_.upper_level;
        }
        inline double min_prop_disturb_elapsed_timePybind()
        {
            return min_elapsed_time_.prop_disturb;
        }
        inline double min_update_obs_subset_elapsed_timePybind()
        {
            return min_elapsed_time_.update_obs_subset;
        }
        void setPolygonVerticesPybind(std::vector<double> vertices)
        {
            params_.robot.vertices_flattened = vertices;
            this->computeMaxGammaPolygonNorm();
        }

    private:
        struct ComputationTime min_elapsed_time_;
        double *pybind_cov_sol_;
    };
} // namespace sipoc_mr_solver

#endif // SIPOC_MR_CONTROLLER__DILATED_POLYGON_ROBUST_CONTROLLER_PYBIND_HPP_
