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

#ifndef SIPOC_RA_UTILS__TIME_OPTIMAL_VELOCITY_HPP_
#define SIPOC_RA_UTILS__TIME_OPTIMAL_VELOCITY_HPP_

#include "sipoc_ra_utils/spline_wrapper_8dim.hpp"

#include <functional>
#include <limits>
#include <vector>

namespace sipoc_ra_utils
{

    struct Bound
    {
        double lower{-std::numeric_limits<double>::infinity()};
        double upper{std::numeric_limits<double>::infinity()};
    };

    struct VelocityNode
    {
        double coordinate{0.0};
        double velocity{0.0};
        double distance{0.0};
        double time{0.0};
    };

    class TimeOptimalVelocity
    {
        static constexpr double eps = std::numeric_limits<double>::epsilon();

    public:
        /**
         * Args: path, path coordinate [1]
         * Ret: tangential velocity bound [m/s]
         */
        using VelBoundFun = Bound(const SplineWrapper8Dim & /*path*/, double /*th*/);
        /**
         * Args: path, path coordinate [1], tangential velocity [m/s]
         * Ret: tangential velocity/acceleration bound [m/s^2]
         */
        using AccBoundFun = Bound(const SplineWrapper8Dim & /*path*/, double /*th*/, double /*sd*/);

        explicit TimeOptimalVelocity(uint32_t bisection_iterations = 10);

        auto solve(VelocityNode seed,
                   const SplineWrapper8Dim &path,
                   const std::vector<double> &grid,
                   std::pair<uint32_t, uint32_t> &range) const -> std::vector<VelocityNode>;
        auto solve(VelocityNode seed, const SplineWrapper8Dim &path, const std::vector<double> &grid) const
            -> std::vector<VelocityNode>;
        auto solve_step1_geometry(const SplineWrapper8Dim &path, const std::vector<double> &grid) const
            -> std::vector<VelocityNode>;
        auto solve_step2_velocity(const SplineWrapper8Dim &path,
                                  const std::vector<double> &grid,
                                  std::vector<VelocityNode> &intermediate_result) const -> std::vector<VelocityNode> &;
        auto solve_step3_acceleration(VelocityNode seed,
                                      const SplineWrapper8Dim &path,
                                      const std::vector<double> &grid,
                                      std::vector<VelocityNode> &intermediate_result,
                                      std::pair<uint32_t, uint32_t> &range) const -> std::vector<VelocityNode> &;

        void setBisectionIterations(uint32_t bisection_iterations);
        void setConstantAccelerationBound(const Bound &accel_bound);
        void setConstantVelocityBound(const Bound &vel_bound);
        void setInitialVelocityBound(const Bound &vel_bound);
        void setTerminalVelocityBound(const Bound &vel_bound);

        auto getConstantAccelerationBound() const -> const Bound &;
        auto getConstantVelocityBound() const -> const Bound &;
        auto getInitialVelocityBound() const -> const Bound &;
        auto getTerminalVelocityBound() const -> const Bound &;

        void addAccelerationBound(std::function<AccBoundFun> accel_bound);
        void addVelocityBound(std::function<VelBoundFun> vel_bound);

        auto acceleration_bounds() -> std::vector<std::function<AccBoundFun>> &;
        auto velocity_bounds() -> std::vector<std::function<VelBoundFun>> &;

    private:
        ///
        ///  Routines operating on single nodes
        ///
        double computeAllowedVelocity(const SplineWrapper8Dim &path, double coordinate) const;
        double computeAllowedAcceleration(const SplineWrapper8Dim &path, double coordinate, double velocity) const;
        double computeAllowedDeceleration(const SplineWrapper8Dim &path, double coordinate, double velocity) const;
        template <bool CheckZeroCrossing = false>
        double simulateVelocity(const SplineWrapper8Dim &path,
                                double current_coordinate,
                                double next_coordinate,
                                double current_velocity,
                                double acceleration) const;
        double simulateTime(const SplineWrapper8Dim &path,
                            double current_coordinate,
                            double next_coordinate,
                            double current_velocity,
                            double next_velocity,
                            double current_time) const;
        double simulateDistance(const SplineWrapper8Dim &path,
                                double current_coordinate,
                                double next_coordinate,
                                double current_distance) const;
        double singleStep(const SplineWrapper8Dim &path,
                          double current_coordinate,
                          double next_coordinate,
                          double current_velocity,
                          double next_maximum_velocity,
                          double acceleration) const;

        ///
        ///  Routines operating on trajectories
        ///
        std::pair<uint32_t, uint32_t> locateSeedOnGrid(VelocityNode &seed, const std::vector<double> &grid) const;
        auto shiftDistanceTrajectory(const VelocityNode &seed,
                                     const SplineWrapper8Dim &path,
                                     std::vector<VelocityNode> &intermediate_result,
                                     uint32_t idx_before_seed,
                                     uint32_t idx_after_seed) const -> std::vector<VelocityNode> &;
        auto computeTimeTrajectory(VelocityNode seed,
                                   const SplineWrapper8Dim &path,
                                   std::vector<VelocityNode> &intermediate_result,
                                   uint32_t idx_before_seed,
                                   uint32_t idx_after_seed,
                                   const std::pair<uint32_t, uint32_t> &range) const -> std::vector<VelocityNode> &;
        auto finalize(std::vector<VelocityNode> &intermediate_result,
                      uint32_t idx_before_seed,
                      uint32_t idx_after_seed,
                      const std::pair<uint32_t, uint32_t> &range) const -> std::vector<VelocityNode> &;

        Bound constant_acceleration_bound_;
        Bound constant_velocity_bound_;
        Bound initial_velocity_bound_;
        Bound terminal_velocity_bound_;

        std::vector<std::function<AccBoundFun>> acceleration_bounds_;
        std::vector<std::function<VelBoundFun>> velocity_bounds_;

        uint32_t bisection_iterations_{10};

    }; // class TimeOptimalVelocity

} // namespace sipoc_ra_utils

#endif // SIPOC_RA_UTILS__TIME_OPTIMAL_VELOCITY_HPP_
