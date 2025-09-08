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

#ifndef SIPOC_MR_UTILS__DISTANCE_MINIMIZER_POINT_TO_LINE_IMPL_HPP_
#define SIPOC_MR_UTILS__DISTANCE_MINIMIZER_POINT_TO_LINE_IMPL_HPP_

#include <math.h>
#include <limits>

namespace sipoc_mr_utils
{
    template <typename T>
    bool DistanceMinimizerPointToLine<T>::argMinDistance(const DistanceMinimizerParameter<T> &param, DistanceMinimizerSolution<T> &sol)
    {
        Eigen::Vector<T, 2> vec;
        vec << std::cos(param.heading_line), std::sin(param.heading_line);
        T projection = vec.dot(param.p_center_ellipsoid);
        projection = std::clamp(projection, -robot_param_.robot_capsule_half_length, robot_param_.robot_capsule_half_length);
        sol.gamma_line = projection;
        // no ellipsoid
        sol.delta_p_ellipsoid << 0., 0.;
        return true;
    };
}

#endif // SIPOC_MR_UTILS__DISTANCE_MINIMIZER_POINT_TO_LINE_IMPL_HPP_
