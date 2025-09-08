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

#include <iostream>
#include <vector>

#include "sipoc_mr_utils/velocity_profile_optimizer.hpp"

namespace sipoc_mr_utils
{

    VelocityProfileOptimizer::VelocityProfileOptimizer()
        : velopt_path_grid_(velopt_path_N, 0.0)
    {
    }

    void VelocityProfileOptimizer::configureOptimizer(const std::vector<double> &shooting_nodes)
    {
        // Set OCP parameters
        ocp_params_.shooting_nodes = shooting_nodes;
        ocp_params_.ocp_dims.n_hrzn = shooting_nodes.size() - 1;
        x_ref_.resize(5 * (ocp_params_.ocp_dims.n_hrzn + 1), 0);

        // Initialize reference fitter
        reference_fitter_.init(ref_Ndata, ref_degree, ref_nknots);
        reference_spline_.configure(
            reference_fitter_.knots(),
            SplineWrapper::ControlVector(ref_nknots + ref_degree - 1, SplineWrapper::Control::Zero()));

        // Set up velocity optimization path grid
        for (uint32_t i = 0; i < velopt_path_N; ++i)
        {
            velopt_path_grid_[i] = i / (static_cast<double>(velopt_path_N) - 1);
        }
        // Configure constant bounds
        velocity_optimizer_.setConstantVelocityBound(
            Bound{0, local_planner_params_.velopt.mpc_factor * ocp_params_.ocp_bounds.max_forward_velocity});
        velocity_optimizer_.setConstantAccelerationBound(
            Bound{local_planner_params_.velopt.mpc_factor * ocp_params_.ocp_bounds.min_forward_acceleration,
                  local_planner_params_.velopt.mpc_factor * ocp_params_.ocp_bounds.max_angular_acceleration});
        velocity_optimizer_.setTerminalVelocityBound(Bound{0, 0});

        // Curvature derivative
        auto compute_curv_der = [](const ReferenceSpline &path, double coordinate, double curv)
        {
            const double eps = 1e-7;
            double dth = 0;
            double curv0 = curv;
            double curv1 = curv;
            if (coordinate - eps >= path.supportLowerLimit())
            {
                dth += eps;
                curv0 = path.curvature(coordinate - eps);
            }
            if (coordinate + eps <= path.supportUpperLimit())
            {
                dth += eps;
                curv1 = path.curvature(coordinate + eps);
            }
            return (curv1 - curv0) / dth;
        };

        /// Set constraint for maximum velocity
        velocity_optimizer_.addVelocityBound([&](const ReferenceSpline &path, double coordinate)
                                             {
        const double track_width = 0.183;

        // Compute curvature, its derivative and tangent norm
        const double curv = path.curvature(coordinate);
        const double tang_norm = path.p(coordinate).norm();
        const double curv_der = compute_curv_der(path, coordinate, curv);

        double upper = std::numeric_limits<double>::infinity();
        if (curv > 1e-6) {
            // maximum lateral acceleratoin
            upper = sqrt(local_planner_params_.velopt.max_lat_acc / curv);
            // maximum angular velocity
            upper = std::min(upper, local_planner_params_.velopt.max_ang_vel / curv);
        }

        // enforce velocity bound to keep acceleration limits feasible
        double upper2 = std::numeric_limits<double>::infinity();
        if (std::abs(curv_der) > 1e-6) {
            upper2 = std::abs(2 * local_planner_params_.velopt.max_wheel_acc * tang_norm / (track_width * curv_der));
            upper2 =
                std::min(upper2, std::abs(2 * local_planner_params_.velopt.max_wheel_dec * tang_norm / (track_width * curv_der)));
        }
        upper = std::min(upper, sqrt(upper2));

        return Bound{0.0, local_planner_params_.velopt.mpc_factor * upper - std::numeric_limits<double>::epsilon()}; });

        /// Set constraint for maximum wheel acceleration and deceleration
        velocity_optimizer_.addAccelerationBound([&](const ReferenceSpline &path, double coordinate, double velocity)
                                                 {
        const double track_width = 0.183;

        // Compute curvature, its derivative and tangent norm
        const double curv = path.curvature(coordinate);
        const double tang_norm = path.p(coordinate).norm();
        const double curv_der = compute_curv_der(path, coordinate, curv);

        // Function for bound:
        // l_r_sign = -1 for left wheel
        // l_r_sign = +1 for right wheel
        auto fb = [&](double a, double l_r_sign) {
            return (2 * a * tang_norm - l_r_sign * track_width * curv_der * velocity * velocity) /
                   (tang_norm * (2 + l_r_sign * track_width * curv));
        };

        // Compute bounds for acceleration
        double lower, upper;
        lower = fb(local_planner_params_.velopt.max_wheel_dec, -1);                  // due to left wheel
        lower = std::max(lower, fb(local_planner_params_.velopt.max_wheel_dec, 1));  // due to right wheel
        lower = std::min(lower, -1e-7);
        upper = fb(local_planner_params_.velopt.max_wheel_acc, -1);                  // due to left wheel
        upper = std::min(upper, fb(local_planner_params_.velopt.max_wheel_acc, 1));  // due to right wheel
        upper = std::max(upper, 1e-7);

        return Bound{local_planner_params_.velopt.mpc_factor * lower, local_planner_params_.velopt.mpc_factor * upper}; });

        // std::cout << "Optimizer Configured." << std::endl;
    }

    void VelocityProfileOptimizer::setRefPathFlattened(const std::vector<double> &waypoints)
    {
        reference_path_.clear();

        for (uint32_t idx = 0; idx < waypoints.size(); idx += 2)
        {
            WorldPoint ref_pos;
            ref_pos << waypoints[idx], waypoints[idx + 1];
            reference_path_.push_back(ref_pos);
        }
    }

    void VelocityProfileOptimizer::setRefPath(const ReferencePath &waypoints)
    {
        reference_path_.clear();
        reference_path_.insert(reference_path_.end(), waypoints.begin(), waypoints.end());
    }

    void VelocityProfileOptimizer::computeReferenceTraj(const std::vector<double> &robot_state)
    {
        if (robot_state.size() != 5)
        {
            throw std::runtime_error("The size of the robot_state vector should be five");
        }

        SE2 current_pose(robot_state[0], robot_state[1], robot_state[2]);
        SE2Tangent current_meas_vel(robot_state[3], 0., robot_state[4]);
        double step;

        /// Fit reference path
        std::vector<Vector<2>, Eigen::aligned_allocator<Vector<2>>> reference_data_points(ref_Ndata);
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

        // Optimize reference trajectory
        VelocityNode seed;
        uint32_t idx_path = 0;
        {
            // Grid search to find nearest position on path
            double closest_dist2 = std::numeric_limits<double>::infinity();
            for (uint32_t i = 0; i < velopt_path_N; ++i)
            {
                Vector<2> y = reference_spline_(velopt_path_grid_[i]);
                double current_dist2 = (y - current_pose.translation()).squaredNorm();
                if (current_dist2 < closest_dist2)
                {
                    closest_dist2 = current_dist2;
                    idx_path = i;
                }
            }
            double th = velopt_path_grid_[idx_path];
            // Refine nearest position
            for (uint32_t i = 0; i < 2; ++i)
            {
                Vector<2> p;
                Vector<2> dp;
                Vector<2> ddp = reference_spline_.pp(th, p, dp);
                Vector<2> y = (p - current_pose.translation());
                double F = y.transpose() * dp;
                double DF = y.transpose() * ddp;
                DF += dp.transpose() * dp;
                th -= F / DF;
                th = std::clamp(th, reference_spline_.supportLowerLimit(), reference_spline_.supportUpperLimit());
            }
            // Write data to seed
            seed.coordinate = th;
            Vector<2> velocity;
            velocity << current_meas_vel.x(), current_meas_vel.y(); // NOTE: current_meas_vel or current_ref_vel
            double tang_vel = reference_spline_.p(seed.coordinate).normalized().dot(current_pose.rotation() * velocity);
            seed.velocity = std::max(tang_vel, 0.0); // Negative velocities are not supported
            seed.distance = 0.0;
            seed.time = 0.0;
        }
        auto optimal_velocity = velocity_optimizer_.solve(seed, reference_spline_, velopt_path_grid_);

        double prev_reference_angle = current_pose.angle();
        SO2 prev_reference_orientation(prev_reference_angle);

        x_ref_[0] = current_pose.x();
        x_ref_[1] = current_pose.y();
        x_ref_[2] = current_pose.angle();
        x_ref_[3] = current_meas_vel.x();
        x_ref_[4] = current_meas_vel.angle();

        for (int i_hrzn = 1; i_hrzn < ocp_params_.ocp_dims.n_hrzn; i_hrzn++)
        {
            // Search current interval
            double t = ocp_params_.shooting_nodes[i_hrzn];
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
            Vector<2> reference_position = reference_spline_(th);
            // Compute reference angle
            const auto tangent = reference_spline_.p(th).normalized();
            SO2 reference_orientation(tangent(0), tangent(1));
            double reference_angle = prev_reference_angle + (reference_orientation - prev_reference_orientation).angle();
            // Parse reference velocity
            double ref_angular_vel = reference_spline_.curvature_signed(th) * ref_forward_vel;
            x_ref_[i_hrzn * 5 + 0] = reference_position(0);
            x_ref_[i_hrzn * 5 + 1] = reference_position(1);
            x_ref_[i_hrzn * 5 + 2] = reference_angle;
            x_ref_[i_hrzn * 5 + 3] = ref_forward_vel;
            x_ref_[i_hrzn * 5 + 4] = ref_angular_vel;
        }
        //// last
        {
            // Search current interval
            double t = ocp_params_.shooting_nodes.back();
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
            Vector<2> reference_position = reference_spline_(th);
            // Compute reference angle
            const auto tangent = reference_spline_.p(th).normalized();
            SO2 reference_orientation(tangent(0), tangent(1));
            double reference_angle = prev_reference_angle + (reference_orientation - prev_reference_orientation).angle();
            // Parse reference velocity
            double ref_angular_vel = reference_spline_.curvature_signed(th) * ref_forward_vel;
            x_ref_[5 * ocp_params_.ocp_dims.n_hrzn + 0] = reference_position(0);
            x_ref_[5 * ocp_params_.ocp_dims.n_hrzn + 1] = reference_position(1);
            x_ref_[5 * ocp_params_.ocp_dims.n_hrzn + 2] = reference_angle;
            x_ref_[5 * ocp_params_.ocp_dims.n_hrzn + 3] = ref_forward_vel;
            x_ref_[5 * ocp_params_.ocp_dims.n_hrzn + 4] = ref_angular_vel;
        }
    }

} // namespace sipoc_mr_utils
