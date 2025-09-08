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

#ifndef SIPOC_MR_UTILS__DISTANCE_MINIMIZER_POINT_TO_LINE_HPP_
#define SIPOC_MR_UTILS__DISTANCE_MINIMIZER_POINT_TO_LINE_HPP_

#include "sipoc_mr_utils/common_structs.hpp"

#include <Eigen/Dense>

namespace sipoc_mr_utils
{
    template <typename T>
    class DistanceMinimizerPointToLine
    {
    public:
        DistanceMinimizerPointToLine() = default;
        DistanceMinimizerPointToLine(T robot_capsule_half_length)
        {
            robot_param_.robot_capsule_half_length = robot_capsule_half_length;
        }
        DistanceMinimizerPointToLine(const RobotParam<T> &robot_param)
        {
            robot_param_ = robot_param;
        }
        ~DistanceMinimizerPointToLine() = default;

        bool argMinDistance(const DistanceMinimizerParameter<T> &param, DistanceMinimizerSolution<T> &sol);

    protected:
        struct RobotParam<T> robot_param_;
    };
} // namespace sipoc_mr_utils

#include "sipoc_mr_utils/impl/distance_minimizer_point_to_line_impl.hpp"
#endif // SIPOC_MR_UTILS__DISTANCE_MINIMIZER_POINT_TO_LINE_HPP_
