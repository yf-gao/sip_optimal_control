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

#ifndef SIPOC_RA_ROS2__SIA20D_CAR_SEAT_PLANNER_NODE_HPP_
#define SIPOC_RA_ROS2__SIA20D_CAR_SEAT_PLANNER_NODE_HPP_

#include "sipoc_ra_solver/car_seat_trajectory_solver.hpp"
#include "sipoc_ra_utils/common_structs.hpp"
#include "sipoc_ra_utils/utils.hpp"

#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <sensor_msgs/msg/joint_state.hpp>
#include "visualization_msgs/msg/marker.hpp"

#include <manif/manif.h>
#include <memory>
#include <string>

namespace sipoc_ra_ros2
{

    class Sia20dCarSeatPlannerNode : public rclcpp::Node
    {
        static constexpr uint32_t NUM_JOINTS = 8;

    public:
        Sia20dCarSeatPlannerNode();

    private:
        rclcpp::TimerBase::SharedPtr timer_;
        std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
        rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_publisher_;
        rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr publisher_pc_marker_;
        rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr publisher_sep_plane_marker_;
        std::shared_ptr<sipoc_ra_solver::CarSeatTrajectorySolver> solver_;
        std::shared_ptr<sipoc_ra_solver::PinocchioKinematics> pinocchio_kinematics_;
        sipoc_ra_utils::ConfigTrajectorySIPSolver config_sip_solver_;
        Eigen::Vector<double, 2 * NUM_JOINTS> current_joint_state_;
        Eigen::Vector<double, NUM_JOINTS> target_joints_;
        std::vector<Eigen::VectorXd> vec_joint_state_sol_;
        std::vector<Eigen::VectorXd> ref_trajectory_, vec_trajectory_init_;
        std::string urdf_path_;
        std::string env_path_;
        std::string mode_;
        double seat_scale_ = 0.8;
        unsigned int idx_callback_;
        bool flag_reinitialize_ = true;

        void timer_callback();
        void set_reference_and_initialize_solver();
        void declare_and_load_parameters();
    };

} // namespace sipoc_ra_ros2

#endif // SIPOC_RA_ROS2__SIA20D_CAR_SEAT_PLANNER_NODE_HPP_
