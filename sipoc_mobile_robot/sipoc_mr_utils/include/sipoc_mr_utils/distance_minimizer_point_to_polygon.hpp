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

#ifndef SIPOC_MR_UTILS__DISTANCE_MINIMIZER_POINT_TO_POLYGON_HPP_
#define SIPOC_MR_UTILS__DISTANCE_MINIMIZER_POINT_TO_POLYGON_HPP_

#include "sipoc_mr_utils/common_structs.hpp"

#include <clarabel/clarabel.hpp>
#include <Eigen/Eigen>
#include <vector>
#include <optional>

namespace sipoc_mr_utils
{
    class DistanceMinimizerPointToPolygon
    {
    public:
        DistanceMinimizerPointToPolygon(const std::vector<double> &vertices_flattened);
        DistanceMinimizerPointToPolygon(const RobotParam<double> &robot_param);

        inline bool ifInsidePolygon(double polygon_yaw, const Eigen::Vector2d &pt) const
        {
            Eigen::Matrix2d rot_matrix2d;
            rot_matrix2d << std::cos(polygon_yaw), -std::sin(polygon_yaw), std::sin(polygon_yaw), std::cos(polygon_yaw);
            Eigen::VectorXd temp = half_planes_A_ * rot_matrix2d.transpose() * pt - half_planes_b_;
            return temp.maxCoeff() <= 0;
        }

        const Eigen::Matrix<double, Eigen::Dynamic, 2> &A() const { return half_planes_A_; }
        const Eigen::Vector<double, Eigen::Dynamic> &b() const { return half_planes_b_; }

        bool argMinDistance(const DistanceMinimizerParameter<double> &param, DistanceMinimizerSolution<double> &sol);

    protected:
        Eigen::Matrix<double, 2, Eigen::Dynamic> vertices_;
        Eigen::Matrix<double, Eigen::Dynamic, 2> half_planes_A_;
        Eigen::Vector<double, Eigen::Dynamic> half_planes_b_;

        Eigen::Matrix<double, 2, 2> rot_mat_;
        Eigen::Vector<double, 2> q_QP_;
        std::optional<clarabel::DefaultSettings<double>> settings_QP_;
        std::optional<clarabel::DefaultSolution<double>> solution_QP_;
        std::optional<clarabel::DefaultSolver<double>> solver_QP_;

        int num_vertices_;
        void compute_polygon_half_planes();
        void setupQPSolver();
    };
}

#endif // SIPOC_MR_UTILS__DISTANCE_MINIMIZER_POINT_TO_POLYGON_HPP_
