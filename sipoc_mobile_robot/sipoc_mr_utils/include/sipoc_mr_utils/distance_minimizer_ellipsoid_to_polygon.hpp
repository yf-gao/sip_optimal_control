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

#ifndef SIPOC_MR_UTILS__DISTANCE_MINIMIZER_ELLIPSOID_TO_POLYGON_HPP_
#define SIPOC_MR_UTILS__DISTANCE_MINIMIZER_ELLIPSOID_TO_POLYGON_HPP_

#include "sipoc_mr_utils/common_structs.hpp"

#include <clarabel/clarabel.hpp>
#include <Eigen/Eigen>
#include <vector>
#include <optional>

namespace sipoc_mr_utils
{
    // objective function: (R gamma_polygon + Sigma_{1/2} gamma_ellipse - p).squaredNorm()
    // optimization variables [gamma_polygon, gamma_ellipse]

    class DistanceMinimizerEllipsoidToPolygon
    {
    public:
        DistanceMinimizerEllipsoidToPolygon(const std::vector<double> &vertices_flattened);
        DistanceMinimizerEllipsoidToPolygon(const RobotParam<double> &robot_param);

        Eigen::Matrix<double, Eigen::Dynamic, 2> A() const { return A_QCQP_.leftCols<2>().topRows(num_vertices_); }
        Eigen::Vector<double, Eigen::Dynamic> b() const { return b_QCQP_.head(num_vertices_); }

        bool argMinDistance(const DistanceMinimizerParameter<double> &param, DistanceMinimizerSolution<double> &sol);

    protected:
        Eigen::Matrix<double, 2, Eigen::Dynamic> vertices_;
        Eigen::Matrix<double, 4, 4> P_QCQP_;
        Eigen::Matrix<double, Eigen::Dynamic, 4> A_QCQP_;
        Eigen::Vector<double, Eigen::Dynamic> b_QCQP_;
        Eigen::Vector<double, 4> q_QCQP_;
        Eigen::Vector<double, 4> solution_primal_;
        Eigen::Matrix<double, 2, 2> rot_mat_;
        Eigen::Matrix<double, 2, 2> ellip_shape_matrix_sqrt_;
        Eigen::Matrix<double, 2, 2> nonzero_matrix_;
        Eigen::SparseMatrix<double, Eigen::ColMajor> P_sparse_;

        std::optional<clarabel::DefaultSettings<double>> settings_QCQP_;
        std::optional<clarabel::DefaultSolution<double>> solution_QCQP_;
        std::optional<clarabel::DefaultSolver<double>> solver_QCQP_;

        int num_vertices_;

        void setupQCQPSolver();
        void compute_polygon_half_planes();
        void explicitMatrixSquareRoot(const Eigen::Matrix2d &matrix, Eigen::Matrix2d &matrix_sqrt);
    };
}

#endif // SIPOC_MR_UTILS__DISTANCE_MINIMIZER_ELLIPSOID_TO_POLYGON_HPP_
