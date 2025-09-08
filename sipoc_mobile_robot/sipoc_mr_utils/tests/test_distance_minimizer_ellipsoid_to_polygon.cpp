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
#include "sipoc_mr_utils/common_structs.hpp"

#include <iostream>

int main()
{
    std::vector<double> vertices_flattened = {-0.18, -0.11, 0.45, -0.11, 0.45, 0.11, -0.18, 0.11, -0.33, 0.};
    sipoc_mr_utils::DistanceMinimizerEllipsoidToPolygon obj(vertices_flattened);

    sipoc_mr_utils::DistanceMinimizerSolution<double> result;
    sipoc_mr_utils::DistanceMinimizerParameter<double> param;
    param.shape_matrix_ellipsoid << 0.04, 0.01, 0.01, 0.09;
    param.heading_line = 0.1;
    param.p_center_ellipsoid << 1., 1.;
    obj.argMinDistance(param, result);
    std::cout << "delta_p_ellipsoid=" << result.delta_p_ellipsoid.transpose() << std::endl;
    std::cout << "gamma_polygon=" << result.gamma_polygon.transpose() << std::endl;
    return 0;
}
