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
// Authors: Niels van Duijkeren, Yunfan Gao
//

#ifndef SIPOC_RA_UTILS__SIA20D_CAR_SEAT_VELOCITY_PROFILE_OPTIMIZER_HPP_
#define SIPOC_RA_UTILS__SIA20D_CAR_SEAT_VELOCITY_PROFILE_OPTIMIZER_HPP_

#include "sipoc_ra_utils/spline_wrapper_8dim.hpp"
#include "sipoc_ra_utils/spline_fitter_8dim.hpp"
#include "sipoc_ra_utils/time_optimal_velocity.hpp"
#include "sipoc_ra_utils/common_structs.hpp"

#include <Eigen/Core>
#include <Eigen/Sparse>
#include <manif/manif.h>

namespace sipoc_ra_utils
{

    class Sia20dCarSeatVelocityProfileOptimizer
    {
    private:
        static const uint32_t velopt_path_N{100};
        static const uint32_t ref_Ndata{40};
        static const uint32_t ref_nknots{5};
        static const uint32_t ref_degree{3};
        static constexpr int NUM_JOINTS{8};

    public:
        using SE2 = manif::SE2<double>;
        using SE2Tangent = manif::SE2Tangent<double>;
        using SO2 = manif::SO2<double>;
        using JointValues = Eigen::Matrix<double, NUM_JOINTS, 1>;
        using ReferencePath = std::vector<JointValues, Eigen::aligned_allocator<JointValues>>;
        using ReferenceFitter = SplineFitter;
        using ReferenceSpline = SplineWrapper8Dim;

        Sia20dCarSeatVelocityProfileOptimizer();
        void configureOptimizer(double ocp_delta_t, unsigned int ocp_n_hrzn);
        void setRefPath(const std::vector<JointValues> &waypoints);
        void computeReferenceTraj(const Eigen::Matrix<double, 2 * NUM_JOINTS, 1> &joint_state);
        inline const std::vector<Eigen::Matrix<double, 2 * NUM_JOINTS, 1>> &referenceTrajConstRef() const
        {
            return x_ref_;
        }
        inline void splineEvalAtTh(double th, JointValues &joint) const
        {
            joint << reference_spline_(th);
            return;
        }

    private:
        ReferenceFitter reference_fitter_;
        ReferenceSpline reference_spline_;
        ReferencePath reference_path_;
        TimeOptimalVelocity velocity_optimizer_;
        ConfigVelAccBounds vel_acc_bounds_;
        std::vector<Eigen::Matrix<double, 2 * NUM_JOINTS, 1>> x_ref_;
        std::vector<double> velopt_path_grid_;
        unsigned int ocp_n_hrzn_;
        double ocp_delta_t_;
    };

} // namespace sipoc_ra_utils

#endif // SIPOC_RA_UTILS__SIA20D_CAR_SEAT_VELOCITY_PROFILE_OPTIMIZER_HPP_
