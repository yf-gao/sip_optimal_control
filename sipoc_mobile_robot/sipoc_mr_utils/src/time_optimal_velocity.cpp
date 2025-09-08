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

#include <algorithm>
#include <stdexcept>

#include "sipoc_mr_utils/time_optimal_velocity.hpp"

namespace sipoc_mr_utils
{

    TimeOptimalVelocity::TimeOptimalVelocity(uint32_t bisection_iterations) : bisection_iterations_{bisection_iterations}
    {
    }

    auto TimeOptimalVelocity::solve(VelocityNode seed,
                                    const SplineWrapper &path,
                                    const std::vector<double> &grid,
                                    std::pair<uint32_t, uint32_t> &range) const -> std::vector<VelocityNode>
    {
        auto intermediate_result = solve_step1_geometry(path, grid);
        intermediate_result = solve_step2_velocity(path, grid, intermediate_result);
        return solve_step3_acceleration(seed, path, grid, intermediate_result, range);
    }

    auto TimeOptimalVelocity::solve(VelocityNode seed, const SplineWrapper &path, const std::vector<double> &grid) const
        -> std::vector<VelocityNode>
    {
        std::pair<uint32_t, uint32_t> range;
        return solve(seed, path, grid, range);
    }

    auto TimeOptimalVelocity::solve_step1_geometry(const SplineWrapper &path, const std::vector<double> &grid) const
        -> std::vector<VelocityNode>
    {
        std::vector<VelocityNode> intermediate_result(grid.size(), VelocityNode{});
        if (grid.size() == 0)
        {
            return intermediate_result;
        }

        for (uint32_t i = 0; i < grid.size(); ++i)
        {
            intermediate_result[i].coordinate = grid[i];
            intermediate_result[i].velocity = 0.0;
            intermediate_result[i].time = std::numeric_limits<double>::infinity();
        }
        /// The distance is first computed relative to the start of the grid.
        /// Step 2 shifts the 0-distance to the seed point.
        intermediate_result[0].distance = 0.0;
        for (uint32_t i = 0; i < grid.size() - 1; ++i)
        {
            intermediate_result[i + 1].distance =
                simulateDistance(path, intermediate_result[i].coordinate, intermediate_result[i + 1].coordinate,
                                 intermediate_result[i].distance);
        }

        return intermediate_result;
    }

    auto TimeOptimalVelocity::solve_step2_velocity(const SplineWrapper &path,
                                                   const std::vector<double> &grid,
                                                   std::vector<VelocityNode> &intermediate_result) const
        -> std::vector<VelocityNode> &
    {
        if (grid.size() == 0)
        {
            return intermediate_result;
        }

        for (uint32_t i = 0; i < grid.size(); ++i)
        {
            intermediate_result[i].velocity = computeAllowedVelocity(path, grid[i]);
        }

        return intermediate_result;
    }

    auto TimeOptimalVelocity::solve_step3_acceleration(VelocityNode seed,
                                                       const SplineWrapper &path,
                                                       const std::vector<double> &grid,
                                                       std::vector<VelocityNode> &intermediate_result,
                                                       std::pair<uint32_t, uint32_t> &range) const
        -> std::vector<VelocityNode> &
    {
        uint32_t idx_before_seed, idx_after_seed;
        std::tie(idx_before_seed, idx_after_seed) = locateSeedOnGrid(seed, grid);
        bool seed_on_grid = (idx_before_seed == idx_after_seed);

        intermediate_result = shiftDistanceTrajectory(seed, path, intermediate_result, idx_before_seed, idx_after_seed);

        if (seed_on_grid)
        {
            intermediate_result[idx_after_seed].velocity =
                std::clamp(seed.velocity, 0., intermediate_result[idx_after_seed].velocity);
            intermediate_result[idx_after_seed].time = seed.time;
        }
        else
        {
            double max_deceleration = computeAllowedDeceleration(path, seed.coordinate, seed.velocity);
            intermediate_result[idx_before_seed].velocity =
                singleStep(path, seed.coordinate, grid[idx_before_seed], seed.velocity,
                           intermediate_result[idx_before_seed].velocity, max_deceleration);

            double max_acceleration = computeAllowedAcceleration(path, seed.coordinate, seed.velocity);
            intermediate_result[idx_after_seed].velocity =
                singleStep(path, seed.coordinate, grid[idx_after_seed], seed.velocity,
                           intermediate_result[idx_after_seed].velocity, max_acceleration);
        }

        for (int32_t i = idx_before_seed; i > 0; --i)
        {
            double max_deceleration = computeAllowedDeceleration(path, grid[i], intermediate_result[i].velocity);
            intermediate_result[i - 1].velocity = singleStep(path, grid[i], grid[i - 1], intermediate_result[i].velocity,
                                                             intermediate_result[i - 1].velocity, max_deceleration);
        }

        intermediate_result[0].velocity = std::min(intermediate_result[0].velocity, initial_velocity_bound_.upper);
        for (int32_t i = 0; i < idx_before_seed; ++i)
        {
            double max_acceleration = computeAllowedAcceleration(path, grid[i], intermediate_result[i].velocity);
            intermediate_result[i + 1].velocity = singleStep(path, grid[i], grid[i + 1], intermediate_result[i].velocity,
                                                             intermediate_result[i + 1].velocity, max_acceleration);
        }

        for (int32_t i = idx_after_seed; i < grid.size() - 1; ++i)
        {
            double max_acceleration = computeAllowedAcceleration(path, grid[i], intermediate_result[i].velocity);
            intermediate_result[i + 1].velocity = singleStep(path, grid[i], grid[i + 1], intermediate_result[i].velocity,
                                                             intermediate_result[i + 1].velocity, max_acceleration);
        }

        intermediate_result[grid.size() - 1].velocity =
            std::min(intermediate_result[grid.size() - 1].velocity, terminal_velocity_bound_.upper);
        for (int32_t i = grid.size() - 1; i > idx_after_seed; --i)
        {
            double max_deceleration = computeAllowedDeceleration(path, grid[i], intermediate_result[i].velocity);
            intermediate_result[i - 1].velocity = singleStep(path, grid[i], grid[i - 1], intermediate_result[i].velocity,
                                                             intermediate_result[i - 1].velocity, max_deceleration);
        }

        {
            auto range_start_it = std::find_if(std::prev(intermediate_result.rend(), idx_before_seed),
                                               intermediate_result.rend(), [](const auto &n)
                                               { return n.velocity < eps; });
            auto range_end_it = std::find_if(std::next(intermediate_result.begin(), idx_after_seed + 1),
                                             intermediate_result.end(), [](const auto &n)
                                             { return n.velocity < eps; });
            if (range_start_it != intermediate_result.rend())
            {
                range.first = std::distance(range_start_it, std::prev(intermediate_result.rend()));
            }
            else
            {
                range.first = 0;
            }
            if (range_end_it != intermediate_result.end())
            {
                range.second = std::distance(intermediate_result.begin(), range_end_it);
            }
            else
            {
                range.second = grid.size() - 1;
            }
        }

        // Compute time trajectory
        computeTimeTrajectory(seed, path, intermediate_result, idx_before_seed, idx_after_seed, range);

        // Fill remainder of trajectory
        return finalize(intermediate_result, idx_before_seed, idx_after_seed, range);
    }

    void TimeOptimalVelocity::setBisectionIterations(uint32_t bisection_iterations)
    {
        bisection_iterations_ = bisection_iterations;
    }

    void TimeOptimalVelocity::setConstantAccelerationBound(const Bound &accel_bound)
    {
        constant_acceleration_bound_ = accel_bound;
    }

    void TimeOptimalVelocity::setConstantVelocityBound(const Bound &vel_bound)
    {
        constant_velocity_bound_ = vel_bound;
    }

    void TimeOptimalVelocity::setInitialVelocityBound(const Bound &vel_bound)
    {
        initial_velocity_bound_ = vel_bound;
    }

    void TimeOptimalVelocity::setTerminalVelocityBound(const Bound &vel_bound)
    {
        terminal_velocity_bound_ = vel_bound;
    }

    auto TimeOptimalVelocity::getConstantAccelerationBound() const -> const Bound &
    {
        return constant_acceleration_bound_;
    }

    auto TimeOptimalVelocity::getConstantVelocityBound() const -> const Bound &
    {
        return constant_velocity_bound_;
    }

    auto TimeOptimalVelocity::getInitialVelocityBound() const -> const Bound &
    {
        return initial_velocity_bound_;
    }

    auto TimeOptimalVelocity::getTerminalVelocityBound() const -> const Bound &
    {
        return terminal_velocity_bound_;
    }

    void TimeOptimalVelocity::addAccelerationBound(std::function<AccBoundFun> accel_bound)
    {
        acceleration_bounds_.push_back(std::move(accel_bound));
    }

    void TimeOptimalVelocity::addVelocityBound(std::function<VelBoundFun> vel_bound)
    {
        velocity_bounds_.push_back(std::move(vel_bound));
    }

    auto TimeOptimalVelocity::acceleration_bounds() -> std::vector<std::function<AccBoundFun>> &
    {
        return acceleration_bounds_;
    }

    auto TimeOptimalVelocity::velocity_bounds() -> std::vector<std::function<VelBoundFun>> &
    {
        return velocity_bounds_;
    }

    double TimeOptimalVelocity::computeAllowedVelocity(const SplineWrapper &path, double coordinate) const
    {
        // Compute velocity bounds according to constraints
        double allowed_velocity = constant_velocity_bound_.upper;
        for (const auto &f : velocity_bounds_)
        {
            allowed_velocity = std::min(allowed_velocity, f(path, coordinate).upper);
        }

        // Finite maximum velocity required in case we need to perform bisection, otherwise we cannot guarantee correct results
        if ((std::isinf(allowed_velocity) || std::isnan(allowed_velocity)) && !acceleration_bounds_.empty() &&
            bisection_iterations_ > 0)
        {
            throw std::runtime_error(
                "A finite maximum velocity is required to ensure correct results concerning acceleration bound "
                "feasibility.");
        }

        // Ensure acceleration feasibility, we require it to be possible to maintain velocity
        double step_size = allowed_velocity / 2.0;
        // Perform bisection
        for (uint32_t i = 0; i < bisection_iterations_; ++i)
        {
            // Compute acceleration bounds for the current velocity bound
            double acc_lower = constant_acceleration_bound_.lower;
            double acc_upper = constant_acceleration_bound_.upper;
            for (const auto &f : acceleration_bounds_)
            {
                auto b = f(path, coordinate, allowed_velocity);
                acc_lower = std::max(acc_lower, b.lower);
                acc_upper = std::min(acc_upper, b.upper);
            }
            // Check if set of allowed accelerations/decelerations is suitable
            if ((i == 0) && (acc_lower <= -eps) && (acc_upper >= eps))
            {
                return allowed_velocity;
            }
            // Add or subtract half the section width based on acceptance criterium
            if ((acc_lower > -eps) || (acc_upper < eps))
            {
                allowed_velocity -= step_size;
            }
            else
            {
                allowed_velocity += step_size;
            }
            // Adapt step size for next iteration
            step_size /= 2.0;
        }
        // Perform centering step to mitigate possibility of result being optimistic
        if (bisection_iterations_ > 0)
        {
            allowed_velocity -= step_size;
        }

        return allowed_velocity;
    }

    double TimeOptimalVelocity::computeAllowedAcceleration(const SplineWrapper &path,
                                                           double coordinate,
                                                           double velocity) const
    {
        double allowed_acceleration = constant_acceleration_bound_.upper;
        for (const auto &f : acceleration_bounds_)
        {
            allowed_acceleration = std::min(allowed_acceleration, f(path, coordinate, velocity).upper);
        }
        // Catch case where allowed_acceleration == 0, avoid numerical problems
        return std::max(allowed_acceleration, eps);
    }

    double TimeOptimalVelocity::computeAllowedDeceleration(const SplineWrapper &path,
                                                           double coordinate,
                                                           double velocity) const
    {
        double allowed_deceleration = constant_acceleration_bound_.lower;
        for (const auto &f : acceleration_bounds_)
        {
            allowed_deceleration = std::max(allowed_deceleration, f(path, coordinate, velocity).lower);
        }
        // Catch case where allowed_deceleration == 0, avoid numerical problems
        return std::min(allowed_deceleration, -eps);
    }

    template <bool CheckZeroCrossing>
    double TimeOptimalVelocity::simulateVelocity(const SplineWrapper &path,
                                                 double current_coordinate,
                                                 double next_coordinate,
                                                 double current_velocity,
                                                 double acceleration) const
    {
        double dth = next_coordinate - current_coordinate;
        double mid_point = (current_coordinate + next_coordinate) / 2;
        double vel_sq = current_velocity * current_velocity + 2 * acceleration * path.p(mid_point).norm() * dth;
        if constexpr (CheckZeroCrossing)
        {
            if (vel_sq < 0.0)
            {
                return 0.0;
            }
        }
        return sqrt(vel_sq);
    }

    double TimeOptimalVelocity::simulateTime(const SplineWrapper &path,
                                             double current_coordinate,
                                             double next_coordinate,
                                             double current_velocity,
                                             double next_velocity,
                                             double current_time) const
    {
        double dth = next_coordinate - current_coordinate;
        double tangent_norm_mid = path.p((current_coordinate + next_coordinate) / 2).norm();
        return current_time + 2 * dth * tangent_norm_mid / (current_velocity + next_velocity);
    }

    double TimeOptimalVelocity::simulateDistance(const SplineWrapper &path,
                                                 double current_coordinate,
                                                 double next_coordinate,
                                                 double current_distance) const
    {
        double tangent_norm1 = path.p(current_coordinate).norm();
        double tangent_norm2 = path.p(next_coordinate).norm();
        double dth = next_coordinate - current_coordinate;
        return current_distance + 0.5 * dth * (tangent_norm1 + tangent_norm2);
    }

    double TimeOptimalVelocity::singleStep(const SplineWrapper &path,
                                           double current_coordinate,
                                           double next_coordinate,
                                           double current_velocity,
                                           double next_maximum_velocity,
                                           double acceleration) const
    {
        double next_velocity = simulateVelocity(path, current_coordinate, next_coordinate, current_velocity, acceleration);
        return std::min(next_velocity, next_maximum_velocity);
    }

    std::pair<uint32_t, uint32_t> TimeOptimalVelocity::locateSeedOnGrid(VelocityNode &seed,
                                                                        const std::vector<double> &grid) const
    {
        // Ensure that grid is sorted
        if (!std::is_sorted(grid.begin(), grid.end()))
        {
            throw std::runtime_error("Grid points must be sorted in ascending order.");
        }

        // Find coordinates in grid neighboring the seed.
        auto it_after_seed = std::lower_bound(grid.begin(), grid.end(), seed.coordinate);
        // Cap the two indices to lay in [0, N). In case seed.coordinate is outside the grid, either idx_after_seed
        // will be N-1, or idx_before seed will be 0. The solution will otherwise remain valid, usability is unclear.
        uint32_t idx_after_seed, idx_before_seed;
        if (it_after_seed == grid.begin())
        {
            idx_before_seed = 0;
            idx_after_seed = 0;
            seed.coordinate = grid.front();
        }
        else if (it_after_seed == grid.end())
        {
            idx_before_seed = grid.size() - 1;
            idx_after_seed = grid.size() - 1;
            seed.coordinate = grid.back();
        }
        else if (seed.coordinate == *it_after_seed)
        {
            idx_after_seed = std::distance(grid.begin(), it_after_seed);
            idx_before_seed = idx_after_seed;
        }
        else
        {
            idx_after_seed = std::distance(grid.begin(), it_after_seed);
            idx_before_seed = idx_after_seed - 1;
        }

        return {idx_before_seed, idx_after_seed};
    }

    auto TimeOptimalVelocity::shiftDistanceTrajectory(const VelocityNode &seed,
                                                      const SplineWrapper &path,
                                                      std::vector<VelocityNode> &intermediate_result,
                                                      uint32_t idx_before_seed,
                                                      uint32_t idx_after_seed) const -> std::vector<VelocityNode> &
    {
        bool seed_on_grid = (idx_before_seed == idx_after_seed);

        // Compute distance from start of path to seed
        double delta_distance = intermediate_result[idx_before_seed].distance;
        if (!seed_on_grid)
        {
            delta_distance += simulateDistance(path, intermediate_result[idx_before_seed].coordinate, seed.coordinate,
                                               intermediate_result[idx_before_seed].distance);
        }

        // Shift distances such that the seed is at zero distance
        std::transform(intermediate_result.begin(), intermediate_result.end(), intermediate_result.begin(),
                       [delta_distance](auto &n)
                       {
                           n.distance -= delta_distance;
                           return n;
                       });

        return intermediate_result;
    }

    auto TimeOptimalVelocity::computeTimeTrajectory(VelocityNode seed,
                                                    const SplineWrapper &path,
                                                    std::vector<VelocityNode> &intermediate_result,
                                                    uint32_t idx_before_seed,
                                                    uint32_t idx_after_seed,
                                                    const std::pair<uint32_t, uint32_t> &range) const
        -> std::vector<VelocityNode> &
    {
        bool seed_on_grid = (idx_before_seed == idx_after_seed);

        if (!seed_on_grid)
        {
            {
                // Before seed
                VelocityNode &node = intermediate_result[idx_before_seed];
                if (seed.velocity < eps && intermediate_result[idx_before_seed].velocity < eps)
                {
                    node.time = seed.time;
                }
                else
                {
                    node.time = simulateTime(path, seed.coordinate, intermediate_result[idx_before_seed].coordinate,
                                             seed.velocity, intermediate_result[idx_before_seed].velocity, seed.time);
                }
            }
            {
                // After seed
                VelocityNode &node = intermediate_result[idx_after_seed];
                if (seed.velocity < eps && intermediate_result[idx_after_seed].velocity < eps)
                {
                    node.time = seed.time;
                }
                else
                {
                    node.time = simulateTime(path, seed.coordinate, intermediate_result[idx_after_seed].coordinate,
                                             seed.velocity, intermediate_result[idx_after_seed].velocity, seed.time);
                }
            }
        }

        // Compute time trajectory from the seed down to the start of the range
        for (int32_t i = idx_before_seed; i > range.first; --i)
        {
            VelocityNode &node = intermediate_result[i - 1];
            node.time = simulateTime(path, intermediate_result[i].coordinate, intermediate_result[i - 1].coordinate,
                                     intermediate_result[i].velocity, intermediate_result[i - 1].velocity,
                                     intermediate_result[i].time);
        }

        // Compute time trajectory from the seed up to the end of the range
        for (int32_t i = idx_after_seed; i < range.second; ++i)
        {
            VelocityNode &node = intermediate_result[i + 1];
            node.time = simulateTime(path, intermediate_result[i].coordinate, intermediate_result[i + 1].coordinate,
                                     intermediate_result[i].velocity, intermediate_result[i + 1].velocity,
                                     intermediate_result[i].time);
        }

        return intermediate_result;
    }

    auto TimeOptimalVelocity::finalize(std::vector<VelocityNode> &intermediate_result,
                                       uint32_t idx_before_seed,
                                       uint32_t idx_after_seed,
                                       const std::pair<uint32_t, uint32_t> &range) const -> std::vector<VelocityNode> &
    {
        bool seed_on_grid = (idx_before_seed == idx_after_seed);

        // Fill the remainder of the intermediate_result behind of the range
        for (int32_t i = range.first; i > 0; --i)
        {
            VelocityNode &node = intermediate_result[i - 1];
            node.velocity = 0.0;
            node.time = -std::numeric_limits<double>::infinity();
        }

        // Fill the remainder of the intermediate_result behind of the range
        for (int32_t i = range.second + 1; i < intermediate_result.size(); ++i)
        {
            VelocityNode &node = intermediate_result[i];
            node.velocity = 0.0;
            node.time = std::numeric_limits<double>::infinity();
        }

        return intermediate_result;
    }

} // namespace sipoc_mr_utils
