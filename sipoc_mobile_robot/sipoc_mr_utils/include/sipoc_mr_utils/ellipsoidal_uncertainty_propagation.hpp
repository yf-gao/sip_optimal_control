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

#ifndef SIPOC_MR_UTILS__ELLIPSOIDAL_UNCERTAINTY_PROPAGATION_HPP_
#define SIPOC_MR_UTILS__ELLIPSOIDAL_UNCERTAINTY_PROPAGATION_HPP_

#include "sipoc_mr_utils/common_structs.hpp"

#include "acados_c/ocp_nlp_interface.h"

#include <Eigen/Dense>

#include <vector>

// NOTE: here the nx and nu are hard-coded

namespace sipoc_mr_utils
{
    class EllipsoidalUncertaintyPropagation
    {
    private:
        double *d_A_mat_; // shape = (nx, nx)
        double *d_B_mat_; // shape = (nx, nu)
        Eigen::Matrix<double, 9, 9> eigen_A_mat_;
        Eigen::Matrix<double, 9, 2> eigen_B_mat_;
        Eigen::Matrix<double, 2, 9> eigen_K_mat_;
        Eigen::Matrix<double, 9, 9> eigen_additive_W_mat_;
        Eigen::Matrix<double, 9, 9> eigen_ABK_mat_;
        struct OCPDimensions ocp_dims_;
        struct AdditiveDisturbance disturb_scales_;
        ocp_nlp_config *nlp_config_ = nullptr;
        ocp_nlp_dims *nlp_dims_ = nullptr;
        ocp_nlp_solver *nlp_solver_ = nullptr;

        void allocateMemory();
        void setMatricesValues();
        void deallocateMemory();

        void inline castGradAToEigenMatrix(const double *data_A, Eigen::Matrix<double, 9, 9> &mat)
        {
            // NOTE: the blasfeo matrices are column major
            mat << data_A[0], data_A[9], data_A[18], data_A[27], data_A[36], data_A[45], data_A[54], data_A[63], data_A[72],
                data_A[1], data_A[10], data_A[19], data_A[28], data_A[37], data_A[46], data_A[55], data_A[64], data_A[73],
                data_A[2], data_A[11], data_A[20], data_A[29], data_A[38], data_A[47], data_A[56], data_A[65], data_A[74],
                data_A[3], data_A[12], data_A[21], data_A[30], data_A[39], data_A[48], data_A[57], data_A[66], data_A[75],
                data_A[4], data_A[13], data_A[22], data_A[31], data_A[40], data_A[49], data_A[58], data_A[67], data_A[76],
                data_A[5], data_A[14], data_A[23], data_A[32], data_A[41], data_A[50], data_A[59], data_A[68], data_A[77],
                data_A[6], data_A[15], data_A[24], data_A[33], data_A[42], data_A[51], data_A[60], data_A[69], data_A[78],
                data_A[7], data_A[16], data_A[25], data_A[34], data_A[43], data_A[52], data_A[61], data_A[70], data_A[79],
                data_A[8], data_A[17], data_A[26], data_A[35], data_A[44], data_A[53], data_A[62], data_A[71], data_A[80];
        }

        void inline castGradBToEigenMatrix(const double *data_B, Eigen::Matrix<double, 9, 2> &mat)
        {
            // NOTE: the blasfeo matrices are column major
            mat << data_B[0], data_B[9],
                data_B[1], data_B[10],
                data_B[2], data_B[11],
                data_B[3], data_B[12],
                data_B[4], data_B[13],
                data_B[5], data_B[14],
                data_B[6], data_B[15],
                data_B[7], data_B[16],
                data_B[8], data_B[17];
        }

    public:
        std::vector<Eigen::Matrix<double, 9, 9>> unc_matrices;

        EllipsoidalUncertaintyPropagation()
        {
            allocateMemory();
            setMatricesValues();
        }
        EllipsoidalUncertaintyPropagation(const OCPDimensions &ocp_dims, const AdditiveDisturbance &disturb_scales)
        {
            ocp_dims_ = ocp_dims;
            disturb_scales_ = disturb_scales;
            allocateMemory();
            setMatricesValues();
        }
        ~EllipsoidalUncertaintyPropagation()
        {
            deallocateMemory();
        }

        void inline setUncMatricesZeros()
        {
            for (int ii = 1; ii <= ocp_dims_.n_hrzn; ++ii)
            {
                unc_matrices[ii].setZero();
            }
        }

        void configureAcadosInterface(ocp_nlp_solver *solver);
        void propagateUncertainty(const std::vector<double> &shooting_nodes);
    };

} // namespace sipoc_mr_utils
#endif // SIPOC_MR_UTILS__ELLIPSOIDAL_UNCERTAINTY_PROPAGATION_HPP_
