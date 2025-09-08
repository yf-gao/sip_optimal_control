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

#include "sipoc_ra_ros2/sia20d_car_seat_mpc_node.hpp"
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

Sia20dCarSeatMPCNode::Sia20dCarSeatMPCNode()
    : Node("sia20d_car_seat_mpc_node"), idx_callback_(0)
{
    config_sip_solver_.ocp.delta_t = 0.1;
    config_sip_solver_.ocp.joint_vel_as_state = true;
    config_sip_solver_.ocp.terminal_equality_constraints = false;
    config_sip_solver_.use_cropped_octomap = true;

    this->declare_and_load_parameters();
    solver_ = std::make_shared<sipoc_ra_solver::CarSeatTrajectorySolver>(config_sip_solver_, urdf_path_, env_path_, seat_scale_);
    vel_opt_ = std::make_shared<sipoc_ra_utils::Sia20dCarSeatVelocityProfileOptimizer>();
    vel_opt_->configureOptimizer(config_sip_solver_.ocp.delta_t, config_sip_solver_.ocp.n_hrzn);
    this->prepare_global_plan();
    this->get_current_joint_angles();

    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(this);
    joint_state_publisher_ = this->create_publisher<sensor_msgs::msg::JointState>("joint_states", 1);
    if (mode_ == "ocp")
    {
        timer_ = this->create_wall_timer(6s, std::bind(&Sia20dCarSeatMPCNode::timer_callback, this));
    }
    else if (mode_ == "closed_loop")
    {
        RCLCPP_INFO(this->get_logger(), "Animate MPC trajectory");
        timer_ = this->create_wall_timer(500ms, std::bind(&Sia20dCarSeatMPCNode::timer_callback, this));
    }
    else if (mode_ == "global_plan")
    {
        RCLCPP_INFO(this->get_logger(), "Animate the spline reference path");
        timer_ = this->create_wall_timer(200ms, std::bind(&Sia20dCarSeatMPCNode::timer_callback, this));
    }
    else
    {
        RCLCPP_ERROR(this->get_logger(), "Invalid mode specified. Use 'ocp' or 'closed_loop' or 'global_plan'.");
        rclcpp::shutdown();
        return;
    }
}

void Sia20dCarSeatMPCNode::declare_and_load_parameters()
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
    config_sip_solver_.ocp.nx = 2 * NUM_JOINTS; // joints + velocities

    // OCP configuration
    this->declare_parameter<int>("ocp.n_hrzn", config_sip_solver_.ocp.n_hrzn);
    this->declare_parameter<double>("ocp.delta_t", config_sip_solver_.ocp.delta_t);
    this->get_parameter("ocp.n_hrzn", config_sip_solver_.ocp.n_hrzn);
    this->get_parameter("ocp.delta_t", config_sip_solver_.ocp.delta_t);

    // Bounds
    this->declare_parameter<double>("bounds.max_joint_vel", config_sip_solver_.bounds.max_joint_vel);
    this->declare_parameter<double>("bounds.max_joint_acc", config_sip_solver_.bounds.max_joint_acc);
    this->declare_parameter<double>("bounds.max_spline_acc", config_sip_solver_.bounds.max_spline_acc);
    this->declare_parameter<double>("bounds.velopt_factor", config_sip_solver_.bounds.velopt_factor);
    this->get_parameter("bounds.max_joint_vel", config_sip_solver_.bounds.max_joint_vel);
    this->get_parameter("bounds.max_joint_acc", config_sip_solver_.bounds.max_joint_acc);
    this->get_parameter("bounds.max_spline_acc", config_sip_solver_.bounds.max_spline_acc);
    this->get_parameter("bounds.velopt_factor", config_sip_solver_.bounds.velopt_factor);

    // Weights
    this->declare_parameter<double>("weights.joint_angle", config_sip_solver_.ocp.weight_joint_angle);
    this->declare_parameter<double>("weights.joint_vel", config_sip_solver_.ocp.weight_joint_vel);
    this->declare_parameter<double>("weights.joint_acc", config_sip_solver_.ocp.weight_joint_acc);
    this->declare_parameter<double>("weights.joint_angle_terminal", config_sip_solver_.ocp.weight_joint_angle_terminal);
    this->declare_parameter<double>("weights.joint_vel_terminal", config_sip_solver_.ocp.weight_joint_vel_terminal);
    this->get_parameter("weights.joint_angle", config_sip_solver_.ocp.weight_joint_angle);
    this->get_parameter("weights.joint_vel", config_sip_solver_.ocp.weight_joint_vel);
    this->get_parameter("weights.joint_acc", config_sip_solver_.ocp.weight_joint_acc);
    this->get_parameter("weights.joint_angle_terminal", config_sip_solver_.ocp.weight_joint_angle_terminal);
    this->get_parameter("weights.joint_vel_terminal", config_sip_solver_.ocp.weight_joint_vel_terminal);
}

bool Sia20dCarSeatMPCNode::prepare_global_plan()
{
    Eigen::Vector<double, 8> start_joints;
    start_joints << 2.22, -3.14, 0.5, 0, -1.45, 0, 0.39, -0.29;
    target_joints_ << 4.06036, -0.0356958, 0.680113, 0.0250365, -1.18485, 0.0585558, 0.294533, -0.330305;

    std::vector<Eigen::Vector<double, 8>> vec_global_plan;
    vec_global_plan.push_back(start_joints);
    Eigen::Vector<double, 8> joints_intermediate;
    joints_intermediate << 2.37675, -2.81352, 0.507139, 0.0132931, -1.4246, -0.00415536, 0.38223, -0.304372;
    vec_global_plan.push_back(joints_intermediate);
    joints_intermediate << 2.53199, -2.48953, 0.514122, 0.0265756, -1.39943, -0.00836716, 0.374539, -0.31872;
    vec_global_plan.push_back(joints_intermediate);
    joints_intermediate << 2.68585, -2.16776, 0.520956, 0.0398596, -1.37444, -0.0126392, 0.36692, -0.333059;
    vec_global_plan.push_back(joints_intermediate);
    joints_intermediate << 2.83848, -1.84791, 0.527647, 0.0531568, -1.34962, -0.0169753, 0.359366, -0.347399;
    vec_global_plan.push_back(joints_intermediate);
    joints_intermediate << 2.99, -1.52968, 0.5342, 0.0664794, -1.32496, -0.0213794, 0.35187, -0.361755;
    vec_global_plan.push_back(joints_intermediate);
    joints_intermediate << 3.14056, -1.2128, 0.540622, 0.0798393, -1.30041, -0.0258554, 0.344426, -0.37614;
    vec_global_plan.push_back(joints_intermediate);
    joints_intermediate << 3.2903, -0.896981, 0.546919, 0.0932485, -1.27598, -0.0304074, 0.337027, -0.390565;
    vec_global_plan.push_back(joints_intermediate);
    joints_intermediate << 3.48149, -0.60701, 0.576906, 0.0848655, -1.25399, -0.03047, 0.333097, -0.397251;
    vec_global_plan.push_back(joints_intermediate);
    joints_intermediate << 3.77055, -0.30854, 0.651534, 0.0702163, -1.2214, 0.00413815, 0.317032, -0.375991;
    vec_global_plan.push_back(joints_intermediate);
    vec_global_plan.push_back(target_joints_);
    vel_opt_->setRefPath(vec_global_plan);
    return true;
}

void Sia20dCarSeatMPCNode::get_current_joint_angles()
{
    std::vector<double> temp_joint(NUM_JOINTS, 0.0);
    this->declare_parameter<std::vector<double>>("joints_current", temp_joint);
    this->get_parameter("joints_current", temp_joint);
    if (temp_joint.size() != NUM_JOINTS)
    {
        RCLCPP_ERROR(this->get_logger(), "The size of joints_current does not match the number of joints in the robot.");
        rclcpp::shutdown();
        return;
    }
    std::copy(temp_joint.data(), temp_joint.data() + NUM_JOINTS, current_joint_state_.data());
    current_joint_state_.tail(NUM_JOINTS).setZero();
}

void Sia20dCarSeatMPCNode::timer_callback()
{
    std::vector<Eigen::VectorXd> ref_trajectory_Xd;
    manif::SE3<double> pose;
    sensor_msgs::msg::JointState msg = sensor_msgs::msg::JointState();
    msg.header.frame_id = "map";
    msg.name = {"carriage_rail", "joint_s", "joint_l", "joint_e", "joint_u", "joint_r", "joint_b", "joint_t"};
    if (mode_ == "ocp")
    {
        vel_opt_->computeReferenceTraj(current_joint_state_);
        const std::vector<Eigen::Vector<double, 2 * NUM_JOINTS>> &ref_trajectory = vel_opt_->referenceTrajConstRef();
        if (idx_callback_ == 0)
        {
            RCLCPP_INFO(this->get_logger(), "Animate the spline reference path");
            for (unsigned int idx_hrzn = 0; idx_hrzn <= solver_->getNumHorizons(); ++idx_hrzn)
            {
                msg.header.stamp = this->now();
                msg.position = {ref_trajectory[idx_hrzn].coeff(0), ref_trajectory[idx_hrzn].coeff(1),
                                ref_trajectory[idx_hrzn].coeff(2), ref_trajectory[idx_hrzn].coeff(3),
                                ref_trajectory[idx_hrzn].coeff(4), ref_trajectory[idx_hrzn].coeff(5),
                                ref_trajectory[idx_hrzn].coeff(6), ref_trajectory[idx_hrzn].coeff(7)};
                joint_state_publisher_->publish(msg);
                rclcpp::sleep_for(std::chrono::milliseconds(200));
            }
        }
        else if (idx_callback_ == 1)
        {
            RCLCPP_INFO(this->get_logger(), "Animate optimized trajectory");
            solver_->setOCPCurrentJointState(current_joint_state_);
            for (auto const &ref_state : ref_trajectory)
            {
                ref_trajectory_Xd.emplace_back(Eigen::VectorXd(ref_state));
            }
            solver_->setOCPRefTrajectory(ref_trajectory_Xd, true);
            sipoc_ra_utils::SolverStatus status = solver_->solve();
            const std::vector<Eigen::VectorXd> &vec_robot_joints = solver_->getSolRobotJointsOverTime();
            for (unsigned int idx_hrzn = 0; idx_hrzn <= solver_->getNumHorizons(); ++idx_hrzn)
            {
                msg.header.stamp = this->now();
                msg.position = {vec_robot_joints[idx_hrzn].coeff(0), vec_robot_joints[idx_hrzn].coeff(1),
                                vec_robot_joints[idx_hrzn].coeff(2), vec_robot_joints[idx_hrzn].coeff(3),
                                vec_robot_joints[idx_hrzn].coeff(4), vec_robot_joints[idx_hrzn].coeff(5),
                                vec_robot_joints[idx_hrzn].coeff(6), vec_robot_joints[idx_hrzn].coeff(7)};
                joint_state_publisher_->publish(msg);
                rclcpp::sleep_for(std::chrono::milliseconds(200));
            }
        }
        else
        {
            rclcpp::shutdown();
            return;
        }
    }
    else if (mode_ == "global_plan")
    {
        {
            if (idx_callback_ == 0)
            {
                vel_opt_->computeReferenceTraj(current_joint_state_);
            }
            double th = idx_callback_ / 100.0;
            if (th > 1.0)
            {
                rclcpp::shutdown();
                return;
            }
            Eigen::Vector<double, NUM_JOINTS> joint_angles;
            vel_opt_->splineEvalAtTh(th, joint_angles);
            msg.header.stamp = this->now();
            msg.position = {joint_angles.coeff(0), joint_angles.coeff(1),
                            joint_angles.coeff(2), joint_angles.coeff(3),
                            joint_angles.coeff(4), joint_angles.coeff(5),
                            joint_angles.coeff(6), joint_angles.coeff(7)};
            joint_state_publisher_->publish(msg);
        }
    }
    else // "closed_loop"
    {
        if ((current_joint_state_.head<NUM_JOINTS>() - target_joints_).lpNorm<Eigen::Infinity>() > 1e-2 && idx_callback_ <= 500)
        {
            RCLCPP_INFO(this->get_logger(), "L-infinity to target_joints: %f", (current_joint_state_.head<NUM_JOINTS>() - target_joints_).lpNorm<Eigen::Infinity>());
            msg.header.stamp = this->now();
            msg.position = {current_joint_state_.coeff(0), current_joint_state_.coeff(1),
                            current_joint_state_.coeff(2), current_joint_state_.coeff(3),
                            current_joint_state_.coeff(4), current_joint_state_.coeff(5),
                            current_joint_state_.coeff(6), current_joint_state_.coeff(7)};
            joint_state_publisher_->publish(msg);

            solver_->setOCPCurrentJointState(current_joint_state_);
            vel_opt_->computeReferenceTraj(current_joint_state_);
            const std::vector<Eigen::Vector<double, 2 * NUM_JOINTS>> &ref_trajectory = vel_opt_->referenceTrajConstRef();
            std::vector<Eigen::VectorXd> ref_trajectory_Xd;
            for (auto const &ref_state : ref_trajectory)
            {
                ref_trajectory_Xd.emplace_back(Eigen::VectorXd(ref_state));
            }
            solver_->setOCPRefTrajectory(ref_trajectory_Xd, flag_reinitialize_);
            sipoc_ra_utils::SolverStatus status = solver_->solve();

            if (status == sipoc_ra_utils::SolverStatus::SUCCESS || status == sipoc_ra_utils::SolverStatus::UL_MAX_ITERATIONS_REACHED)
            {
                flag_reinitialize_ = false;
            }
            else
            {
                flag_reinitialize_ = true;
            }
            Eigen::VectorXd u0_star(NUM_JOINTS);
            solver_->getU0Star(u0_star);
            integrateRobotJoints(u0_star, solver_->getSolverConfig().ocp, current_joint_state_);
            // Sanity check
            std::vector<Eigen::VectorXd> vec_robot_state_sol = solver_->getSolRobotJointsOverTime();
            if ((vec_robot_state_sol[1] - current_joint_state_).lpNorm<Eigen::Infinity>() > 1e-6)
            {
                throw std::runtime_error("Error: The first solution does not match the computed next joint state.");
            }

            if (!flag_reinitialize_)
            {
                solver_->shiftInputStateTrajectory();
                solver_->gatherActiveConstraintsAndShiftOneStep();
            }
            else
            {
                solver_->clearPCSubsets();
                // The x_sol will be initialized by the reference trajectory.
            }
        }
        else
        {
            RCLCPP_INFO(this->get_logger(), "Target joint state reached or maximum iterations exceeded.");
            rclcpp::shutdown();
            return;
        }
    }
    ++idx_callback_;
}

void Sia20dCarSeatMPCNode::integrateRobotJoints(const Eigen::Vector<double, NUM_JOINTS> &joint_accelerations, const sipoc_ra_utils::ConfigOCP &config_ocp, Eigen::Vector<double, 2 * NUM_JOINTS> &joint_state)
{
    joint_state.head<NUM_JOINTS>() += joint_state.tail<NUM_JOINTS>() * config_ocp.delta_t + 0.5 * joint_accelerations * config_ocp.delta_t * config_ocp.delta_t;
    joint_state.tail<NUM_JOINTS>() += joint_accelerations * config_ocp.delta_t;
    return;
}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Sia20dCarSeatMPCNode>());
    rclcpp::shutdown();
    return 0;
}
