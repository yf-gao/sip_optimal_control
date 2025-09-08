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

#ifndef SIPOC_RA_ROS2__SIA20D_CAR_SEAT_MPC_NODE_HPP_
#define SIPOC_RA_ROS2__SIA20D_CAR_SEAT_MPC_NODE_HPP_

#include "sipoc_ra_utils/sia20d_car_seat_velocity_profile_optimizer.hpp"
#include "sipoc_ra_solver/car_seat_trajectory_solver.hpp"
#include "sipoc_ra_utils/common_structs.hpp"
#include "sipoc_ra_utils/utils.hpp"

#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <sensor_msgs/msg/joint_state.hpp>

#include <manif/manif.h>
#include <memory>
#include <string>

namespace sipoc_ra_ros2
{
    class Sia20dCarSeatMPCNode : public rclcpp::Node
    {
        static constexpr uint32_t NUM_JOINTS = 8;

    public:
        Sia20dCarSeatMPCNode();

    private:
        rclcpp::TimerBase::SharedPtr timer_;
        std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
        rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_publisher_;
        std::shared_ptr<sipoc_ra_solver::CarSeatTrajectorySolver> solver_;
        std::shared_ptr<sipoc_ra_utils::Sia20dCarSeatVelocityProfileOptimizer> vel_opt_;
        sipoc_ra_utils::ConfigTrajectorySIPSolver config_sip_solver_;
        Eigen::Vector<double, 2 * NUM_JOINTS> current_joint_state_;
        Eigen::Vector<double, NUM_JOINTS> target_joints_;
        std::string urdf_path_;
        std::string env_path_;
        std::string mode_;
        double seat_scale_ = 0.8;
        unsigned int idx_callback_;
        bool flag_reinitialize_ = true;

        void timer_callback();
        void declare_and_load_parameters();
        bool prepare_global_plan();
        void get_current_joint_angles();
        void integrateRobotJoints(const Eigen::Vector<double, NUM_JOINTS> &joint_accelerations, const sipoc_ra_utils::ConfigOCP &config_ocp, Eigen::Vector<double, 2 * NUM_JOINTS> &joint_state);
    };
} // namespace sipoc_ra_ros2

#endif // SIPOC_RA_ROS2__SIA20D_CAR_SEAT_MPC_NODE_HPP_
