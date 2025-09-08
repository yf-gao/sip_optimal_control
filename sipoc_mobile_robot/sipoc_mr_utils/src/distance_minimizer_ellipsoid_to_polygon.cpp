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

#include "sipoc_mr_utils/distance_minimizer_ellipsoid_to_polygon.hpp"

#include <iostream>

#ifdef DEBUG_PRINTING
#define DEBUG_MSG(str)                 \
    do                                 \
    {                                  \
        std::cout << str << std::endl; \
    } while (false)
#else
#define DEBUG_MSG(str) \
    do                 \
    {                  \
    } while (false)
#endif

namespace sipoc_mr_utils
{

    DistanceMinimizerEllipsoidToPolygon::DistanceMinimizerEllipsoidToPolygon(const std::vector<double> &vertices_flattened)
    {
        // NOTE: the vertices are in counter-clockwise order
        num_vertices_ = vertices_flattened.size() / 2;
        vertices_ = Eigen::Map<const Eigen::Matrix<double, 2, Eigen::Dynamic, Eigen::ColMajor>>(vertices_flattened.data(), 2, num_vertices_);
        DEBUG_MSG("Vertices: " << vertices_);

        setupQCQPSolver();
    }

    DistanceMinimizerEllipsoidToPolygon::DistanceMinimizerEllipsoidToPolygon(const RobotParam<double> &robot_param)
    {
        // NOTE: the vertices are in counter-clockwise order
        num_vertices_ = robot_param.vertices_flattened.size() / 2;
        vertices_ = Eigen::Map<const Eigen::Matrix<double, 2, Eigen::Dynamic, Eigen::ColMajor>>(robot_param.vertices_flattened.data(), 2, num_vertices_);
        DEBUG_MSG("Vertices: " << vertices_);

        setupQCQPSolver();
    }

    void DistanceMinimizerEllipsoidToPolygon::setupQCQPSolver()
    {
        // NOTE: To make the sparsity of the P matrix consistant
        // NOTE: The lower left block of P is zero
        P_QCQP_ << 1., 0., 0.1, 0.1, 0., 1., 0.1, 0.1, 0., 0., 1., 0., 0., 0., 0., 1.;
        P_sparse_ = P_QCQP_.sparseView();
        P_sparse_.makeCompressed();
        q_QCQP_.setZero();

        A_QCQP_.resize(num_vertices_ + 3, Eigen::NoChange);
        A_QCQP_.setZero();
        b_QCQP_.resize(num_vertices_ + 3);
        b_QCQP_.setZero();
        this->compute_polygon_half_planes();

        A_QCQP_.block<2, 2>(num_vertices_ + 1, 2) = -1.0 * Eigen::Matrix2d::Identity();
        b_QCQP_(num_vertices_) = 1.0;

        Eigen::SparseMatrix<double, Eigen::ColMajor> A_sparse;
        A_sparse = A_QCQP_.sparseView();
        A_sparse.makeCompressed();

        std::vector<clarabel::SupportedConeT<double>> cones = {
            clarabel::NonnegativeConeT<double>(num_vertices_),
            clarabel::SecondOrderConeT<double>(3)};
        settings_QCQP_ = clarabel::DefaultSettings<double>::default_settings();
        settings_QCQP_->presolve_enable = false;
        settings_QCQP_->verbose = false;

        solver_QCQP_ = clarabel::DefaultSolver<double>(P_sparse_, q_QCQP_, A_sparse, b_QCQP_, cones, *settings_QCQP_);

        nonzero_matrix_ << 1e-27, 1e-27, 1e-27, 1e-27;
    }

    void DistanceMinimizerEllipsoidToPolygon::compute_polygon_half_planes()
    {
        // Ax <= b
        Eigen::Vector2d edge;
        for (int i = 0; i < num_vertices_; ++i)
        {
            int i_next = (i + 1) % num_vertices_;
            // NOTE: the vertices are in the counter-clockwise order
            edge = vertices_.col(i_next) - vertices_.col(i);
            edge.normalize();
            A_QCQP_(i, 0) = edge.coeff(1);
            A_QCQP_(i, 1) = -edge.coeff(0);
            b_QCQP_(i) = A_QCQP_.block<1, 2>(i, 0).dot(vertices_.col(i));
        }
    }

    void DistanceMinimizerEllipsoidToPolygon::explicitMatrixSquareRoot(const Eigen::Matrix2d &matrix, Eigen::Matrix2d &matrix_sqrt)
    {
        double a, b, radian, temp;
        temp = std::sqrt((matrix.coeff(0, 0) - matrix.coeff(1, 1)) * (matrix.coeff(0, 0) - matrix.coeff(1, 1)) + 4 * matrix.coeff(0, 1) * matrix.coeff(0, 1));
        a = (matrix.coeff(0, 0) + matrix.coeff(1, 1) + temp) / 2.0;
        b = (matrix.coeff(0, 0) + matrix.coeff(1, 1) - temp) / 2.0;
        radian = std::acos(std::sqrt((matrix.coeff(0, 0) - b) / temp));
        // if (mat[0, 1] <= 0.) {radian *= -1.0; }
        radian *= (2.0 * static_cast<double>(matrix.coeff(0, 1) >= 0.0) - 1.0);
        // p = R(radian) @ diag(sqrt(a), sqrt(b)) @ unit_p = matrix_sqrt @ unit_p
        matrix_sqrt << std::cos(radian), -std::sin(radian), std::sin(radian), std::cos(radian);
        matrix_sqrt.col(0) *= std::sqrt(a);
        matrix_sqrt.col(1) *= std::sqrt(b);
        return;
    }

    bool DistanceMinimizerEllipsoidToPolygon::argMinDistance(const DistanceMinimizerParameter<double> &param, DistanceMinimizerSolution<double> &sol)
    {
        rot_mat_ << std::cos(param.heading_line), -std::sin(param.heading_line),
            std::sin(param.heading_line), std::cos(param.heading_line);
        explicitMatrixSquareRoot(param.shape_matrix_ellipsoid, ellip_shape_matrix_sqrt_);

        // P_QCQP_ = [I  R^T\Sigma_{1/2}; \Sigma_{1/2}^TR \Sigma_{1/2}^T\Sigma_{1/2}]
        // updateP
        P_QCQP_.topLeftCorner<2, 2>().setIdentity();
        P_QCQP_.topRightCorner<2, 2>() = rot_mat_.transpose() * ellip_shape_matrix_sqrt_ + nonzero_matrix_;
        P_QCQP_.bottomRightCorner<2, 2>() = ellip_shape_matrix_sqrt_.transpose() * ellip_shape_matrix_sqrt_;
        // NOTE: \Sigma_{1/2}^T\Sigma_{1/2} = diag([a, b]) is a diagonal matrix
        P_QCQP_(2, 3) = 0.;
        P_QCQP_(3, 2) = 0.;
        P_sparse_ = P_QCQP_.sparseView();
        P_sparse_.makeCompressed();
        solver_QCQP_->update_P(P_sparse_);
        // q_QCQP = -[R^Tp; Sigma_{1/2}^Tp]
        q_QCQP_.head<2>() = -rot_mat_.transpose() * param.p_center_ellipsoid;
        q_QCQP_.tail<2>() = -ellip_shape_matrix_sqrt_.transpose() * param.p_center_ellipsoid;
        solver_QCQP_->update_q(q_QCQP_);

        // solve
        solver_QCQP_->solve();
        solution_QCQP_ = solver_QCQP_->solution();
        solution_primal_ = solution_QCQP_->x;

        clarabel::SolverStatus status = solution_QCQP_->status;
        if (status != clarabel::SolverStatus::Solved && status != clarabel::SolverStatus::AlmostSolved)
        {
            DEBUG_MSG("Clarabel solver failed. ");
            DEBUG_MSG("rot_mat: \n"
                      << rot_mat_);
            DEBUG_MSG("p_link2obs: " << param.p_center_ellipsoid.transpose());
            DEBUG_MSG("shape_matrix_ellipsoid: \n"
                      << param.shape_matrix_ellipsoid);
            return false;
        }
        sol.gamma_polygon(0) = solution_primal_.coeff(0);
        sol.gamma_polygon(1) = solution_primal_.coeff(1);
        sol.delta_p_ellipsoid = ellip_shape_matrix_sqrt_ * solution_primal_.tail<2>();
        sol.squared_distance = 2 * solution_QCQP_->obj_val + param.p_center_ellipsoid.squaredNorm();
        return true;
    }

}
