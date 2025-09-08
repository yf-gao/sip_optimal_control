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

#include "sipoc_ra_ros2/sia20d_car_seat_planner_node.hpp"
#include "sipoc_ra_utils/common_structs.hpp"
#include "sipoc_ra_utils/utils.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <cstdlib>
#include <filesystem>

using namespace std::chrono_literals;
using namespace sipoc_ra_ros2;

Sia20dCarSeatPlannerNode::Sia20dCarSeatPlannerNode()
    : Node("sia20d_car_seat_planner_node"), idx_callback_(0)
{
    config_sip_solver_.ocp.nx = 8;
    config_sip_solver_.ocp.nu = 8;
    config_sip_solver_.ocp.num_max_constr = 30;
    config_sip_solver_.ocp.delta_t = 0.3;
    config_sip_solver_.ocp.joint_vel_as_state = false;
    config_sip_solver_.ocp.terminal_equality_constraints = true;
    config_sip_solver_.use_cropped_octomap = false;
    this->declare_and_load_parameters();

    solver_ = std::make_shared<sipoc_ra_solver::CarSeatTrajectorySolver>(config_sip_solver_, urdf_path_, env_path_, seat_scale_);
    pinocchio_kinematics_ = solver_->getPinocchioKinematicsSharedPtr();
    this->set_reference_and_initialize_solver();

    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(this);
    joint_state_publisher_ = this->create_publisher<sensor_msgs::msg::JointState>("joint_states", 1);
    publisher_pc_marker_ = this->create_publisher<visualization_msgs::msg::Marker>("pc_marker_array", 1);
    publisher_sep_plane_marker_ = this->create_publisher<visualization_msgs::msg::Marker>("sep_plane_marker_array", 1);
    if (mode_ == "ocp")
    {
        timer_ = this->create_wall_timer(7s, std::bind(&Sia20dCarSeatPlannerNode::timer_callback, this));
    }
    else if (mode_ == "sip_iterations")
    {
        RCLCPP_INFO(this->get_logger(), "Animate trajectories throughout Iterations");
        timer_ = this->create_wall_timer(7s, std::bind(&Sia20dCarSeatPlannerNode::timer_callback, this));
    }
    else
    {
        RCLCPP_ERROR(this->get_logger(), "Invalid mode specified. Use 'sip_iterations' or 'ocp'.");
        rclcpp::shutdown();
        return;
    }
}

void Sia20dCarSeatPlannerNode::declare_and_load_parameters()
{
    this->declare_parameter<std::string>("urdf_path", urdf_path_);
    this->declare_parameter<std::string>("env_path", env_path_);
    this->declare_parameter<double>("seat_scale", seat_scale_);
    this->declare_parameter<std::string>("mode", mode_);
    this->get_parameter("urdf_path", urdf_path_);
    std::filesystem::path path = ament_index_cpp::get_package_share_directory("sipoc_ra_support");
    path += urdf_path_;
    urdf_path_ = path.string();
    path = ament_index_cpp::get_package_share_directory("sipoc_ra_support");
    this->get_parameter("env_path", env_path_);
    path += env_path_;
    env_path_ = path.string();
    this->get_parameter("mode", mode_);
    this->get_parameter("seat_scale", seat_scale_);
    config_sip_solver_.ocp.nx = NUM_JOINTS;

    // OCP configuration
    this->declare_parameter<int>("ocp.n_hrzn", config_sip_solver_.ocp.n_hrzn);
    this->declare_parameter<double>("ocp.delta_t", config_sip_solver_.ocp.delta_t);
    this->get_parameter("ocp.n_hrzn", config_sip_solver_.ocp.n_hrzn);
    this->get_parameter("ocp.delta_t", config_sip_solver_.ocp.delta_t);

    // Bounds
    this->declare_parameter<double>("bounds.max_joint_vel", config_sip_solver_.bounds.max_joint_vel);
    this->get_parameter("bounds.max_joint_vel", config_sip_solver_.bounds.max_joint_vel);
    this->declare_parameter<double>("bounds.max_lin_vel", config_sip_solver_.bounds.max_lin_vel);
    this->get_parameter("bounds.max_lin_vel", config_sip_solver_.bounds.max_lin_vel);

    // Weights
    this->declare_parameter<double>("weights.joint_angle", config_sip_solver_.ocp.weight_joint_angle);
    this->declare_parameter<double>("weights.joint_vel", config_sip_solver_.ocp.weight_joint_vel);
    this->declare_parameter<double>("weights.joint_angle_terminal", config_sip_solver_.ocp.weight_joint_angle_terminal);
    this->get_parameter("weights.joint_angle", config_sip_solver_.ocp.weight_joint_angle);
    this->get_parameter("weights.joint_vel", config_sip_solver_.ocp.weight_joint_vel);
    this->get_parameter("weights.joint_angle_terminal", config_sip_solver_.ocp.weight_joint_angle_terminal);
}

void Sia20dCarSeatPlannerNode::set_reference_and_initialize_solver()
{
    Eigen::Vector<double, 8> start_joints;
    start_joints << 2.22, -3.14, 0.5, 0, -1.45, 0, 0.39, -0.29;
    Eigen::VectorXd target_joints(8);
    target_joints << 4.05856, -0.0378665, 0.701035, 0.0240165, -1.22282, 0.0585929, 0.353481, -0.32;
    for (unsigned int i = 0; i <= config_sip_solver_.ocp.n_hrzn; ++i)
    {
        vec_trajectory_init_.emplace_back(Eigen::VectorXd(start_joints + (target_joints - start_joints) * static_cast<double>(i) / static_cast<double>(config_sip_solver_.ocp.n_hrzn)));
        ref_trajectory_.emplace_back(target_joints);
    }

    solver_->setOCPCurrentJointState(start_joints);
    solver_->setOCPTerminalJointState(target_joints);
    solver_->setOCPRefTrajectory(ref_trajectory_, false);
    solver_->initializeSolver(vec_trajectory_init_);
}

void Sia20dCarSeatPlannerNode::timer_callback()
{
    manif::SE3<double> pose;
    sensor_msgs::msg::JointState msg = sensor_msgs::msg::JointState();
    msg.header.frame_id = "map";
    msg.name = {"carriage_rail", "joint_s", "joint_l", "joint_e", "joint_u", "joint_r", "joint_b", "joint_t"};

    if (mode_ == "ocp")
    {
        if (idx_callback_ == 0)
        {
            RCLCPP_INFO(this->get_logger(), "Animate initialization");
            for (unsigned int idx_hrzn = 0; idx_hrzn <= solver_->getNumHorizons(); ++idx_hrzn)
            {
                msg.header.stamp = this->now();
                msg.position = {vec_trajectory_init_[idx_hrzn].coeff(0), vec_trajectory_init_[idx_hrzn].coeff(1),
                                vec_trajectory_init_[idx_hrzn].coeff(2), vec_trajectory_init_[idx_hrzn].coeff(3),
                                vec_trajectory_init_[idx_hrzn].coeff(4), vec_trajectory_init_[idx_hrzn].coeff(5),
                                vec_trajectory_init_[idx_hrzn].coeff(6), vec_trajectory_init_[idx_hrzn].coeff(7)};
                joint_state_publisher_->publish(msg);
                rclcpp::sleep_for(std::chrono::milliseconds(500));
            }
        }
        else if (idx_callback_ == 1)
        {
            RCLCPP_INFO(this->get_logger(), "Animate Planned trajectory");
            sipoc_ra_utils::SolverStatus status = solver_->solve();
            if (status == sipoc_ra_utils::SolverStatus::SUCCESS)
            {
                RCLCPP_INFO(this->get_logger(), "Open-loop trajectory planning succeeded.");
            }
            else
            {
                RCLCPP_ERROR(this->get_logger(), "Open-loop trajectory planning failed with status: %d", static_cast<int>(status));
            }
            vec_joint_state_sol_ = solver_->getSolRobotJointsOverTime();
            for (unsigned int idx_hrzn = 0; idx_hrzn <= solver_->getNumHorizons(); ++idx_hrzn)
            {
                msg.header.stamp = this->now();
                msg.position = {vec_joint_state_sol_[idx_hrzn].coeff(0), vec_joint_state_sol_[idx_hrzn].coeff(1),
                                vec_joint_state_sol_[idx_hrzn].coeff(2), vec_joint_state_sol_[idx_hrzn].coeff(3),
                                vec_joint_state_sol_[idx_hrzn].coeff(4), vec_joint_state_sol_[idx_hrzn].coeff(5),
                                vec_joint_state_sol_[idx_hrzn].coeff(6), vec_joint_state_sol_[idx_hrzn].coeff(7)};
                joint_state_publisher_->publish(msg);
                rclcpp::sleep_for(std::chrono::milliseconds(500));
            }
        }
        else
        {
            rclcpp::shutdown();
            return;
        }
    }
    else // "sip_iterations"
    {
        RCLCPP_INFO(this->get_logger(), "idx_iter = %d", idx_callback_);
        sipoc_ra_utils::SolverStatus status = solver_->solveOneIteration(idx_callback_ >= 1);
        if (status != sipoc_ra_utils::SolverStatus::SUCCESS)
        {
            RCLCPP_ERROR(this->get_logger(), "idx_iter = %d. Error in solveOneIteration: %d", idx_callback_, static_cast<int>(status));
            rclcpp::shutdown();
            return;
        }
        vec_joint_state_sol_ = solver_->getSolRobotJointsOverTime();
        solver_->initializeSolver(vec_joint_state_sol_);

        std::vector<Eigen::Vector<double, 6>> vec_sep_planes;
        for (unsigned int idx = 0; idx < vec_joint_state_sol_.size(); ++idx)
        {
            msg.header.stamp = this->now();
            msg.position = {vec_joint_state_sol_[idx].coeff(0), vec_joint_state_sol_[idx].coeff(1),
                            vec_joint_state_sol_[idx].coeff(2), vec_joint_state_sol_[idx].coeff(3),
                            vec_joint_state_sol_[idx].coeff(4), vec_joint_state_sol_[idx].coeff(5),
                            vec_joint_state_sol_[idx].coeff(6), vec_joint_state_sol_[idx].coeff(7)};
            joint_state_publisher_->publish(msg);

            RCLCPP_INFO(this->get_logger(), "idx_hrzn = %d, joint_state = [%f, %f, %f, %f, %f, %f, %f, %f]",
                        idx, msg.position[0], msg.position[1], msg.position[2], msg.position[3],
                        msg.position[4], msg.position[5], msg.position[6], msg.position[7]);

            solver_->getCollSeparatingPlanes(idx, vec_sep_planes);

            auto marker_points = visualization_msgs::msg::Marker();
            marker_points.header.frame_id = "map";
            marker_points.header.stamp = this->get_clock()->now();
            marker_points.id = 0;
            marker_points.type = visualization_msgs::msg::Marker::POINTS;
            marker_points.action = visualization_msgs::msg::Marker::MODIFY;
            marker_points.pose.position.x = 0.;
            marker_points.pose.position.y = 0.;
            marker_points.pose.position.z = 0.;
            marker_points.pose.orientation.x = 0.0;
            marker_points.pose.orientation.y = 0.0;
            marker_points.pose.orientation.z = 0.0;
            marker_points.pose.orientation.w = 1.0;
            marker_points.scale.x = 0.05;
            marker_points.scale.y = 0.05;
            marker_points.color.r = 0.0;
            marker_points.color.g = 1.0;
            marker_points.color.b = 0.0;
            marker_points.color.a = 1.0;
            marker_points.lifetime = rclcpp::Duration(0, 0);

            auto marker_sep_normals = visualization_msgs::msg::Marker();
            marker_sep_normals.header.frame_id = "map";
            marker_sep_normals.header.stamp = this->get_clock()->now();
            marker_sep_normals.id = 1;
            marker_sep_normals.type = visualization_msgs::msg::Marker::LINE_LIST;
            marker_sep_normals.action = visualization_msgs::msg::Marker::MODIFY;
            marker_sep_normals.pose.position.x = 0.;
            marker_sep_normals.pose.position.y = 0.;
            marker_sep_normals.pose.position.z = 0.;
            marker_sep_normals.pose.orientation.x = 0.0;
            marker_sep_normals.pose.orientation.y = 0.0;
            marker_sep_normals.pose.orientation.z = 0.0;
            marker_sep_normals.pose.orientation.w = 1.0;
            marker_sep_normals.scale.x = 0.05;
            marker_sep_normals.scale.y = 0.05;
            marker_sep_normals.color.r = 0.5;
            marker_sep_normals.color.g = 0.0;
            marker_sep_normals.color.b = 1.0;
            marker_sep_normals.color.a = 1.0;
            marker_sep_normals.lifetime = rclcpp::Duration(0, 0);

            unsigned int num_marker = vec_sep_planes.size();
            RCLCPP_INFO(this->get_logger(), "num_marker = %d", num_marker);
            for (unsigned int idx_pc = 0; idx_pc < num_marker; ++idx_pc)
            {
                RCLCPP_INFO(this->get_logger(), "idx_pc = %d, sep_plane = [%f, %f, %f, %f, %f, %f]",
                            idx_pc, vec_sep_planes[idx_pc](0), vec_sep_planes[idx_pc](1),
                            vec_sep_planes[idx_pc](2), vec_sep_planes[idx_pc](3),
                            vec_sep_planes[idx_pc](4), vec_sep_planes[idx_pc](5));
                geometry_msgs::msg::Point point_start;
                point_start.x = vec_sep_planes[idx_pc](0);
                point_start.y = vec_sep_planes[idx_pc](1);
                point_start.z = vec_sep_planes[idx_pc](2);
                marker_points.points.push_back(point_start);
                marker_sep_normals.points.push_back(point_start);
                geometry_msgs::msg::Point point_end;
                point_end.x = vec_sep_planes[idx_pc](0) + vec_sep_planes[idx_pc](3);
                point_end.y = vec_sep_planes[idx_pc](1) + vec_sep_planes[idx_pc](4);
                point_end.z = vec_sep_planes[idx_pc](2) + vec_sep_planes[idx_pc](5);
                marker_sep_normals.points.push_back(point_end);
            }
            publisher_pc_marker_->publish(marker_points);
            publisher_pc_marker_->publish(marker_sep_normals);

            rclcpp::sleep_for(std::chrono::milliseconds(500));
        }
    }
    ++idx_callback_;
}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Sia20dCarSeatPlannerNode>());
    rclcpp::shutdown();
    return 0;
}
