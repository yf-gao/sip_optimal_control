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
// Author: Niels van Duijkeren
//

#ifndef SIPOC_MR_UTILS__VELOCITY_PROFILE_OPTIMIZER_HPP_
#define SIPOC_MR_UTILS__VELOCITY_PROFILE_OPTIMIZER_HPP_

#include "sipoc_mr_utils/spline_wrapper.hpp"
#include "sipoc_mr_utils/spline_fitter.hpp"
#include "sipoc_mr_utils/time_optimal_velocity.hpp"
#include "sipoc_mr_utils/common_structs.hpp"

#include <Eigen/Core>
#include <Eigen/Sparse>
#include <manif/manif.h>

namespace sipoc_mr_utils
{

    class VelocityProfileOptimizer
    {
    private:
        static const uint32_t velopt_path_N{100};
        static const uint32_t ref_Ndata{40};
        static const uint32_t ref_nknots{5};
        static const uint32_t ref_degree{3};

    public:
        template <int32_t n>
        using Vector = Eigen::Matrix<double, n, 1>;

        using SE2 = manif::SE2<double>;
        using SE2Tangent = manif::SE2Tangent<double>;
        using SO2 = manif::SO2<double>;
        using WorldPoint = Vector<2>;
        using ReferencePath = std::vector<WorldPoint, Eigen::aligned_allocator<WorldPoint>>;
        using ReferenceFitter = SplineFitter;
        using ReferenceSpline = SplineWrapper;

        VelocityProfileOptimizer();
        void configureOptimizer(const std::vector<double> &shooting_nodes);
        void setRefPathFlattened(const std::vector<double> &waypoints);
        void setRefPath(const ReferencePath &waypoints);
        void computeReferenceTraj(const std::vector<double> &robot_state);

        inline const std::vector<double> &referenceTrajConstRef() const
        {
            return x_ref_;
        }

        inline const ReferenceSpline &referenceSplineConstRef() const
        {
            return reference_spline_;
        }

    private:
        ReferenceFitter reference_fitter_;
        ReferenceSpline reference_spline_;
        ReferencePath reference_path_;
        TimeOptimalVelocity velocity_optimizer_;
        OCPParameters ocp_params_;
        LocalPlannerParameters local_planner_params_;
        std::vector<double> velopt_path_grid_;

    protected:
        std::vector<double> x_ref_;
    };

} // namespace sipoc_mr_utils

#endif // SIPOC_MR_UTILS__VELOCITY_PROFILE_OPTIMIZER_HPP_
