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

#ifndef SIPOC_MR_CONTROLLER__CAPSULE_NOMINAL_CONTROLLER_PYBIND_HPP_
#define SIPOC_MR_CONTROLLER__CAPSULE_NOMINAL_CONTROLLER_PYBIND_HPP_

#include "sipoc_mr_solver/capsule_nominal_controller.hpp"

#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>

namespace py = pybind11;

namespace sipoc_mr_solver
{
    class CapsuleNominalControllerPybind : public CapsuleNominalController
    {
    public:
        CapsuleNominalControllerPybind() : CapsuleNominalController()
        {
        }

        CapsuleNominalControllerPybind(int n_hrzn, double delta_t) : CapsuleNominalController(n_hrzn, delta_t)
        {
        }

        void updateInitialStatePybind(py::array_t<double> initial_state);
        void updateReferenceTrajectoryPybind(py::array_t<double> ref_traj);
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
        py::array_t<double> getNumConstrPybind();
        inline double min_lower_level_elapsed_timePybind()
        {
            return min_elapsed_time_.lower_level;
        }
        inline double min_upper_level_elapsed_timePybind()
        {
            return min_elapsed_time_.upper_level;
        }
        inline double min_update_obs_subset_elapsed_timePybind()
        {
            return min_elapsed_time_.update_obs_subset;
        }

    private:
        struct ComputationTime min_elapsed_time_;
    };
} // namespace sipoc_mr_solver
#endif // SIPOC_MR_CONTROLLER__CAPSULE_NOMINAL_CONTROLLER_PYBIND_HPP_
