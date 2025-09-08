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

#ifndef SIPOC_MR_CONTROLLER__DILATED_POLYGON_NOMINAL_CONTROLLER_HPP_
#define SIPOC_MR_CONTROLLER__DILATED_POLYGON_NOMINAL_CONTROLLER_HPP_

#include "sipoc_mr_solver/base_controller.hpp"
#include "sipoc_mr_utils/distance_minimizer_point_to_polygon.hpp"
#include "acados_solver_diff_drive_robot.h"

namespace sipoc_mr_solver
{
    class DilatedPolygonNominalController : public BaseController
    {
    public:
        DilatedPolygonNominalController() : BaseController()
        {
            flag_constr_tightening_ = 0.0;
            checkOCPSameDimsAsExported();
        }

        DilatedPolygonNominalController(int n_hrzn, double delta_t) : BaseController(n_hrzn, delta_t)
        {
            flag_constr_tightening_ = 0.0;
            checkOCPSameDimsAsExported();
        }

        DilatedPolygonNominalController(sipoc_mr_utils::OCPParameters &param) : BaseController(param)
        {
            flag_constr_tightening_ = 0.0;
            checkOCPSameDimsAsExported();
        }
        ~DilatedPolygonNominalController()
        {
            diff_drive_robot_acados_free(acados_ocp_capsule_);
            diff_drive_robot_acados_free_capsule(acados_ocp_capsule_);
        }
        bool solveEndpointGammaConstrainedSubproblem();

    protected:
        diff_drive_robot_solver_capsule *acados_ocp_capsule_ = NULL;
        std::unique_ptr<sipoc_mr_utils::DistanceMinimizerPointToPolygon> ptr_ll_solver_;

        sipoc_mr_utils::DistanceMinimizerSolution<double> lower_level_result_ = {};
        sipoc_mr_utils::DistanceMinimizerParameter<double> lower_level_param_ = {};
        sipoc_mr_utils::MinimizerResult<double> gamma_gs_result_ = {};
        Eigen::Vector2d temp_vector2d_;

        void checkOCPSameDimsAsExported();
        void createAcadosSolver();
        bool solveOCPSubproblem(int idx_iter);
        void configureLowerLevelSolver();
        void findClosestObsGivenGamma(const Eigen::Vector2d &gamma_val, unsigned int idx_constr, unsigned int idx_hrzn);
        void updateGammaAndSepPlane(int idx_hrzn, int num_imposed_constr);
        int getObsWithMaximumGVal(int idx_hrzn);
    };
}

#endif // SIPOC_MR_CONTROLLER__DILATED_POLYGON_NOMINAL_CONTROLLER_HPP_
