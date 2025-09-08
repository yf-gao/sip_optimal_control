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

#include "sipoc_ra_utils/sia20d_car_seat_velocity_profile_optimizer.hpp"

#include <manif/manif.h>
#include <manif/algorithms/interpolation.h>
#include <eigen3/Eigen/Dense>
#include <string>
#include <iostream>
#include <memory>


namespace sipoc_ra_utils
{
    Sia20dCarSeatVelocityProfileOptimizer::Sia20dCarSeatVelocityProfileOptimizer()
        : velopt_path_grid_(velopt_path_N, 0.0)
    {
    }

    void Sia20dCarSeatVelocityProfileOptimizer::configureOptimizer(double ocp_delta_t, unsigned int ocp_n_hrzn)
    {
        // Set OCP parameters
        ocp_delta_t_ = ocp_delta_t;
        ocp_n_hrzn_ = ocp_n_hrzn;
        x_ref_.resize(ocp_n_hrzn_ + 1, Eigen::Matrix<double, 2 * NUM_JOINTS, 1>::Zero());

        // Initialize reference fitter
        reference_fitter_.init(ref_Ndata, ref_degree, ref_nknots);
        reference_spline_.configure(
            reference_fitter_.knots(),
            SplineWrapper8Dim::ControlVector(ref_nknots + ref_degree - 1, SplineWrapper8Dim::Control::Zero()));

        // Set up velocity optimization path grid
        for (uint32_t i = 0; i < velopt_path_N; ++i)
        {
            velopt_path_grid_[i] = i / (static_cast<double>(velopt_path_N) - 1);
        }

        velocity_optimizer_.setTerminalVelocityBound(Bound{0, 0});
        velocity_optimizer_.setConstantAccelerationBound(Bound{-vel_acc_bounds_.max_spline_acc, vel_acc_bounds_.max_spline_acc});

        // Set constraint for maximum velocity
        velocity_optimizer_.addVelocityBound([&](const ReferenceSpline &path, double coordinate)
                                             {
            const JointValues &p = path.p(coordinate);
            double p_norm = p.norm();
            Bound result;
            result.lower = 0.;
            result.upper = 1e3;
            const double max_joint_vel = vel_acc_bounds_.max_joint_vel * vel_acc_bounds_.velopt_factor;
            const double max_lin_vel = vel_acc_bounds_.max_lin_vel * vel_acc_bounds_.velopt_factor;
            // The first joint is a prismatic joint
            if (p.coeff(0) > 1e-6)
            {
                result.upper = std::min(result.upper, max_lin_vel / p.coeff(0) * p_norm);
            }
            else if (p.coeff(0) < -1e-6)
            {
                result.upper = std::min(result.upper, -max_lin_vel / p.coeff(0) * p_norm);
            }
            // The rest are revolute joints
            for (unsigned int idx_joints = 1; idx_joints < NUM_JOINTS; ++idx_joints)
            {
                if (p.coeff(idx_joints) > 1e-6)
                {
                    result.upper = std::min(result.upper, max_joint_vel / p.coeff(idx_joints) * p_norm);
                }
                else if (p.coeff(idx_joints) < -1e-6)
                {
                    result.upper = std::min(result.upper, -max_joint_vel / p.coeff(idx_joints) * p_norm);
                }
            }
            return result; });

        // std::cout << "Optimizer Configured." << std::endl;
    }

    void Sia20dCarSeatVelocityProfileOptimizer::setRefPath(const std::vector<JointValues> &waypoints)
    {
        reference_path_.clear();
        reference_path_.insert(reference_path_.end(), waypoints.begin(), waypoints.end());
    }

    void Sia20dCarSeatVelocityProfileOptimizer::computeReferenceTraj(const Eigen::Matrix<double, 2 * NUM_JOINTS, 1> &joint_state)
    {
        /// Fit reference path
        double step;
        std::vector<JointValues, Eigen::aligned_allocator<JointValues>> reference_data_points(ref_Ndata);
        step = (static_cast<double>(reference_path_.size()) - 1) / (static_cast<double>(ref_Ndata) - 1);
        for (uint32_t i = 0; i < ref_Ndata - 1; ++i)
        {
            /// Interpolate reference points
            int32_t idx1 = i * step;
            int32_t idx2 = std::min(idx1 + 1, static_cast<int32_t>(reference_path_.size()) - 1);
            auto wc1 = reference_path_.at(idx1);
            auto wc2 = reference_path_.at(idx2);
            double alpha = i * step - static_cast<double>(idx1);
            reference_data_points[i] = (1 - alpha) * wc1 + alpha * wc2;
        }
        reference_data_points.back() = reference_path_.back();
        auto reference_controls = reference_fitter_.fit(reference_data_points);
        reference_spline_.setControlPoints(reference_controls);

        VelocityNode seed;
        uint32_t idx_path = 0;
        {
            // Grid search to find nearest position on path
            double closest_dist2 = std::numeric_limits<double>::infinity();
            for (uint32_t i = 0; i < velopt_path_N; ++i)
            {
                JointValues y = reference_spline_(velopt_path_grid_[i]);
                double current_dist2 = (y - joint_state.head<NUM_JOINTS>()).squaredNorm();
                if (current_dist2 < closest_dist2)
                {
                    closest_dist2 = current_dist2;
                    idx_path = i;
                }
            }
            double th = velopt_path_grid_[idx_path];

            // Write data to seed
            seed.coordinate = th;
            double tang_vel = reference_spline_.p(seed.coordinate).normalized().dot(joint_state.tail<NUM_JOINTS>());
            seed.velocity = std::max(tang_vel, 0.2); // NOTE: Force the trajectory to start with a positive velocity
            seed.distance = 0.0;
            seed.time = 0.0;
        }
        auto optimal_velocity = velocity_optimizer_.solve(seed, reference_spline_, velopt_path_grid_);

        // i_hrzn=0
        x_ref_[0] = joint_state;

        Eigen::Matrix<double, 2 * NUM_JOINTS, 1> ref_state;
        for (unsigned int i_hrzn = 1; i_hrzn <= ocp_n_hrzn_; i_hrzn++)
        {
            // Search current interval
            double t = i_hrzn * ocp_delta_t_;
            for (; idx_path < velopt_path_N - 1; ++idx_path)
            {
                if (t <= optimal_velocity[idx_path].time)
                    break;
            }
            // Linearly interpolate coordinate
            double th = reference_spline_.supportUpperLimit();
            double ref_forward_vel = 0.0;
            if (t < optimal_velocity.back().time)
            {
                double alpha = (t - optimal_velocity[idx_path - 1].time) /
                               (optimal_velocity[idx_path].time - optimal_velocity[idx_path - 1].time);
                th =
                    (1 - alpha) * optimal_velocity[idx_path - 1].coordinate + alpha * optimal_velocity[idx_path].coordinate;
                ref_forward_vel =
                    (1 - alpha) * optimal_velocity[idx_path - 1].velocity + alpha * optimal_velocity[idx_path].velocity;
            }
            // Compute reference position
            ref_state << reference_spline_(th), reference_spline_.p(th).normalized() * ref_forward_vel;
            x_ref_[i_hrzn] = ref_state;
        }
    }

} // namespace sipoc_ra_utils
