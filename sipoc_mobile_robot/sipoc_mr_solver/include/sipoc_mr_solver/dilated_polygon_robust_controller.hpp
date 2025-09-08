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

#ifndef SIPOC_MR_CONTROLLER__DILATED_POLYGON_ROBUST_CONTROLLER_HPP_
#define SIPOC_MR_CONTROLLER__DILATED_POLYGON_ROBUST_CONTROLLER_HPP_

#include "sipoc_mr_solver/base_controller.hpp"
#include "sipoc_mr_utils/ellipsoidal_uncertainty_propagation.hpp"
#include "sipoc_mr_utils/distance_minimizer_point_to_polygon.hpp"
#include "sipoc_mr_utils/distance_minimizer_ellipsoid_to_polygon.hpp"
#include "acados_solver_diff_drive_robot.h"

namespace sipoc_mr_solver
{
    class DilatedPolygonRobustController : public BaseController
    {
    protected:
        inline void computeMaxGammaPolygonNorm()
        {
            int num_vertices = params_.robot.vertices_flattened.size() / 2;
            max_gamma_polygon_norm_ = 0.0;
            Eigen::Vector2d vertex;
            for (int idx = 0; idx < num_vertices; ++idx)
            {
                vertex << params_.robot.vertices_flattened[2 * idx], params_.robot.vertices_flattened[2 * idx + 1];
                max_gamma_polygon_norm_ = std::max(max_gamma_polygon_norm_, vertex.norm());
            }
        };

    public:
        bool grads_valid4prop = false;
        DilatedPolygonRobustController() : BaseController()
        {
            flag_constr_tightening_ = 0.0;
            checkOCPSameDimsAsExported();
            computeMaxGammaPolygonNorm();
        }

        DilatedPolygonRobustController(int n_hrzn, double delta_t) : BaseController(n_hrzn, delta_t)
        {
            flag_constr_tightening_ = 0.0;
            checkOCPSameDimsAsExported();
            computeMaxGammaPolygonNorm();
        }

        DilatedPolygonRobustController(sipoc_mr_utils::OCPParameters &param) : BaseController(param)
        {
            flag_constr_tightening_ = 0.0;
            checkOCPSameDimsAsExported();
            computeMaxGammaPolygonNorm();
        }

        ~DilatedPolygonRobustController()
        {
            diff_drive_robot_acados_free(acados_ocp_capsule_);
            diff_drive_robot_acados_free_capsule(acados_ocp_capsule_);
        }
        bool solveEndpointGammaConstrainedSubproblem();
        void inline uncMatrixAtTk(unsigned int k, Eigen::Matrix<double, 9, 9> &unc_matrix)
        {
            unc_matrix = ptr_unc_propagation_->unc_matrices[k];
        }

        void inline updateUncMatricesT0(const Eigen::Matrix<double, 4, 4> &Sigma0)
        {
            ptr_unc_propagation_->unc_matrices[0].setZero();
            ptr_unc_propagation_->unc_matrices[0](0, 0) = 3e-5; // Covariance of Localization on overage
            ptr_unc_propagation_->unc_matrices[0](1, 1) = 3e-5;
            ptr_unc_propagation_->unc_matrices[0](2, 2) = 2e-6;
            ptr_unc_propagation_->unc_matrices[0].bottomRightCorner<4, 4>() = Sigma0;
        }

    protected:
        diff_drive_robot_solver_capsule *acados_ocp_capsule_ = NULL;
        std::unique_ptr<sipoc_mr_utils::DistanceMinimizerEllipsoidToPolygon> ptr_ll_robust_solver_;
        std::unique_ptr<sipoc_mr_utils::DistanceMinimizerPointToPolygon> ptr_ll_nominal_solver_;
        std::unique_ptr<sipoc_mr_utils::EllipsoidalUncertaintyPropagation> ptr_unc_propagation_;

        sipoc_mr_utils::DistanceMinimizerSolution<double> lower_level_result_ = {};
        sipoc_mr_utils::DistanceMinimizerParameter<double> lower_level_param_ = {};
        sipoc_mr_utils::MinimizerResult<double> gamma_gs_result_ = {};
        Eigen::Vector2d temp_vector2d_;
        double max_gamma_polygon_norm_;

        void checkOCPSameDimsAsExported();
        void createAcadosSolver();
        bool solveOCPSubproblem(int idx_iter);
        void configureLowerLevelSolver();
        void findClosestObsGivenGamma(const Eigen::Vector2d &gamma_val, unsigned int idx_constr, unsigned int idx_hrzn);
        void updateGammaAndSepPlane(int idx_hrzn, int num_imposed_constr);
        void updateRobustOCPLinearConstraints(int idx_hrzn);
        int getObsWithMaximumGVal(int idx_hrzn);
    };
}

#endif // SIPOC_MR_CONTROLLER__DILATED_POLYGON_ROBUST_CONTROLLER_HPP_
