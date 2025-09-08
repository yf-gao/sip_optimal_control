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

#include "sipoc_mr_utils/ellipsoidal_uncertainty_propagation.hpp"

#include "acados/utils/mem.h"

#include <stdexcept>
#include <iostream>

namespace sipoc_mr_utils
{
    void EllipsoidalUncertaintyPropagation::allocateMemory()
    {
        d_A_mat_ = (double *)malloc(ocp_dims_.nx * ocp_dims_.nx * sizeof(double));
        d_B_mat_ = (double *)malloc(ocp_dims_.nx * ocp_dims_.nu * sizeof(double));
        unc_matrices.resize(ocp_dims_.n_hrzn + 1);
    }

    void EllipsoidalUncertaintyPropagation::deallocateMemory()
    {
        free(d_A_mat_);
        free(d_B_mat_);
    }

    void EllipsoidalUncertaintyPropagation::setMatricesValues()
    {
        // Zero uncertainties at the start of the prediction horizon
        unc_matrices[0].setZero();
        eigen_additive_W_mat_.setZero();
        eigen_K_mat_.setZero();

        // NOTE: the matrices are measured at the control interval
        // Consider a discrete-time linear system for the robot dynamics:
        // \nu_{k+1} = disc_A \nu_k + disc_B (linVCmd_k, angVCmd_k)
        // where $\nu_k \in R^{4}$ represents the dynamic state.
        // The system is subject to additive disturbance.
        // The covariance matrices of the additive disturbance in the following are specific for the Neobotix MP-500 robot (https://www.neobotix-robots.com/products/mobile-robots/mobile-robot-mp-500) with a maximum forward velocity of 3m/s and angular velocity of 5rad/s,
        // and are obtained by using the N4SID method (https://nfoursid.readthedocs.io/en/latest/) based on the data collected from the robot.
        eigen_additive_W_mat_.bottomRightCorner<4, 4>() = Eigen::Matrix<double, 4, 4>{
            {1.66671625e-05, -2.00701988e-06, 2.51139125e-05, -3.08985736e-05},
            {-2.00701988e-06, 2.89403774e-05, -5.06132829e-06, 2.27151396e-05},
            {2.51139125e-05, -5.06132829e-06, 1.45129030e-04, -4.10594701e-05},
            {-3.08985736e-05, 2.27151396e-05, -4.10594701e-05, 2.27623707e-04}};
    }

    void EllipsoidalUncertaintyPropagation::configureAcadosInterface(ocp_nlp_solver *solver)
    {
        nlp_config_ = solver->config;
        nlp_dims_ = (ocp_nlp_dims *)solver->dims;
        nlp_solver_ = solver;
        if (nlp_dims_->N != ocp_dims_.n_hrzn)
        {
            throw std::logic_error("NOT same number of shooting nodes");
        }
        if ((nlp_dims_->nx[0] != static_cast<int>(ocp_dims_.nx)) || (nlp_dims_->nx[0] != 9))
        {
            throw std::logic_error("nx is NOT equal to 9");
        }
        if ((nlp_dims_->nu[0] != static_cast<int>(ocp_dims_.nu)) || (nlp_dims_->nu[0] != 2))
        {
            throw std::logic_error("nu is NOT equal to 2");
        }
    }

    void EllipsoidalUncertaintyPropagation::propagateUncertainty(const std::vector<double> &shooting_nodes)
    {
        double control_interval = shooting_nodes[1] - shooting_nodes[0];
        for (int ii = 0; ii < ocp_dims_.n_hrzn; ++ii)
        {
            // get and pack: A, B
            ocp_nlp_get_at_stage(nlp_solver_, ii, "A", d_A_mat_);
            ocp_nlp_get_at_stage(nlp_solver_, ii, "B", d_B_mat_);
            castGradAToEigenMatrix(d_A_mat_, eigen_A_mat_);
            castGradBToEigenMatrix(d_B_mat_, eigen_B_mat_);

            eigen_ABK_mat_ = eigen_A_mat_ + eigen_B_mat_ * eigen_K_mat_;
            unc_matrices[ii + 1] = eigen_ABK_mat_ * unc_matrices[ii] * eigen_ABK_mat_.transpose() + eigen_additive_W_mat_ * (shooting_nodes[ii + 1] - shooting_nodes[ii]) / control_interval;
        }
    }
};
