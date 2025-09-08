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

#include "sipoc_mr_utils/distance_minimizer_point_to_polygon.hpp"

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
    DistanceMinimizerPointToPolygon::DistanceMinimizerPointToPolygon(const std::vector<double> &vertices_flattened)
    {
        // NOTE: the vertices are in counter-clockwise order
        num_vertices_ = vertices_flattened.size() / 2;
        vertices_ = Eigen::Map<const Eigen::Matrix<double, 2, Eigen::Dynamic, Eigen::ColMajor>>(vertices_flattened.data(), 2, num_vertices_);
        DEBUG_MSG("Vertices: " << vertices_);

        compute_polygon_half_planes();
        setupQPSolver();
    }

    DistanceMinimizerPointToPolygon::DistanceMinimizerPointToPolygon(const RobotParam<double> &robot_param)
    {
        // NOTE: the vertices are in counter-clockwise order
        num_vertices_ = robot_param.vertices_flattened.size() / 2;
        vertices_ = Eigen::Map<const Eigen::Matrix<double, 2, Eigen::Dynamic, Eigen::ColMajor>>(robot_param.vertices_flattened.data(), 2, num_vertices_);
        DEBUG_MSG("Vertices: " << vertices_);

        compute_polygon_half_planes();
        setupQPSolver();
    }

    void DistanceMinimizerPointToPolygon::compute_polygon_half_planes()
    {
        // Ax <= b
        half_planes_A_.resize(num_vertices_, Eigen::NoChange);
        half_planes_b_.resize(num_vertices_);

        Eigen::Vector2d edge;
        for (int i = 0; i < num_vertices_; ++i)
        {
            int i_next = (i + 1) % num_vertices_;
            // NOTE: the vertices are in the counter-clockwise order
            edge = vertices_.col(i_next) - vertices_.col(i);
            edge.normalize();
            half_planes_A_(i, 0) = edge.coeff(1);
            half_planes_A_(i, 1) = -edge.coeff(0);
            half_planes_b_(i) = half_planes_A_.row(i).dot(vertices_.col(i));
        }
    }

    void DistanceMinimizerPointToPolygon::setupQPSolver()
    {
        Eigen::Matrix<double, 2, 2> P_QP;
        Eigen::SparseMatrix<double, Eigen::ColMajor> P_sparse;
        P_QP << 1., 0., 0., 1.;
        P_sparse = P_QP.sparseView();
        P_sparse.makeCompressed();
        q_QP_.setZero();

        Eigen::SparseMatrix<double, Eigen::ColMajor> A_sparse;
        A_sparse = half_planes_A_.sparseView();
        A_sparse.makeCompressed();
        std::vector<clarabel::SupportedConeT<double>> cones = {
            clarabel::NonnegativeConeT<double>(num_vertices_),
        };
        settings_QP_ = clarabel::DefaultSettings<double>::default_settings();
        settings_QP_->presolve_enable = false;
        settings_QP_->verbose = false;

        solver_QP_ = clarabel::DefaultSolver<double>(P_sparse, q_QP_, A_sparse, half_planes_b_, cones, *settings_QP_);
    }

    bool DistanceMinimizerPointToPolygon::argMinDistance(const DistanceMinimizerParameter<double> &param, DistanceMinimizerSolution<double> &sol)
    {
        rot_mat_ << std::cos(param.heading_line), -std::sin(param.heading_line),
            std::sin(param.heading_line), std::cos(param.heading_line);

        // NOTE: the variable `p_center_ellipsoid` passes the value of the vector between the robot and the polygon.
        q_QP_ = -rot_mat_.transpose() * param.p_center_ellipsoid;
        solver_QP_->update_q(q_QP_);
        // solve
        solver_QP_->solve();
        solution_QP_ = solver_QP_->solution();
        Eigen::Vector<double, 2> closest_point = solution_QP_->x;

        clarabel::SolverStatus status = solution_QP_->status;
        if (status != clarabel::SolverStatus::Solved && status != clarabel::SolverStatus::AlmostSolved)
        {
            DEBUG_MSG("Clarabel solver failed. ");
            DEBUG_MSG("rot_mat: \n"
                      << rot_mat_);
            DEBUG_MSG("p_link2obs: " << param.p_center_ellipsoid.transpose());
            return false;
        }
        sol.gamma_polygon(0) = closest_point(0);
        sol.gamma_polygon(1) = closest_point(1);
        return true;
    }
}
