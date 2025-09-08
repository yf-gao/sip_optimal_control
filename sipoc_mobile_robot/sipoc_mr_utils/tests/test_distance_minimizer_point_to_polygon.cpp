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
#include "sipoc_mr_utils/common_structs.hpp"

#include <iostream>
#include <vector>
#include <Eigen/LU>
#include <Eigen/Eigen>

int main()
{
    std::vector<double> vertices_flattened = {-0.18, -0.11, 0.45, -0.11, 0.45, 0.11, -0.18, 0.11, -0.33, 0.};
    sipoc_mr_utils::DistanceMinimizerPointToPolygon obj(vertices_flattened);

    Eigen::Matrix<double, Eigen::Dynamic, 2> A = obj.A();
    Eigen::Vector<double, Eigen::Dynamic> b = obj.b();

    int num_vertices = vertices_flattened.size() / 2;
    Eigen::Matrix<double, 2, 2> sub_A;
    Eigen::Vector2d sub_b;
    Eigen::Vector2d v;
    for (int i = 0; i < num_vertices; ++i)
    {
        sub_A.row(0) = A.row(i);
        sub_A.row(1) = A.row((i + 1) % num_vertices);
        sub_b(0) = b(i);
        sub_b(1) = b((i + 1) % num_vertices);
        v = sub_A.lu().solve(sub_b);
        std::cout << "Intersection point: " << v.transpose() << std::endl;
    }

    sipoc_mr_utils::DistanceMinimizerParameter<double> param;
    param.p_center_ellipsoid << -0.2, 0.2;
    param.heading_line = M_PI / 6;
    sipoc_mr_utils::DistanceMinimizerSolution<double> sol;
    if (obj.argMinDistance(param, sol))
    {
        std::cout << "Minimum distance point: " << sol.gamma_polygon.transpose() << std::endl;
    }
    else
    {
        std::cerr << "Failed to find minimum distance point." << std::endl;
    }

    Eigen::Matrix2d rot_mat;
    rot_mat << std::cos(param.heading_line), -std::sin(param.heading_line),
        std::sin(param.heading_line), std::cos(param.heading_line);
    std::cout << "Rotated point: " << (rot_mat.transpose() * param.p_center_ellipsoid).transpose() << std::endl;

    if (obj.ifInsidePolygon(param.heading_line, param.p_center_ellipsoid))
    {
        std::cout << "Point is inside the polygon." << std::endl;
    }
    else
    {
        std::cout << "Point is outside the polygon." << std::endl;
    }
    return 0;
}
