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
// Authors: Yunfan Gao, Niels van Duijkeren
//

#include "sipoc_mr_nav2_controller/nav2_capsule_nominal_controller_plugin.hpp"

#include "rcpputils/asserts.hpp"
#include "nav2_util/node_utils.hpp"
#include "nav2_util/geometry_utils.hpp"

#include <algorithm>
#include <string>
#include <memory>
#include <chrono>

using nav2_util::declare_parameter_if_not_declared;
using nav2_util::geometry_utils::euclidean_distance;
using std::abs;
using std::max;
using std::min;

using namespace std::chrono_literals;

namespace sipoc_mr_nav2_controller
{

    void NominalCapsuleController::configure(
        const rclcpp_lifecycle::LifecycleNode::WeakPtr &parent,
        std::string name, std::shared_ptr<tf2_ros::Buffer> tf,
        std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
    {
        node_ = parent;
        auto node = node_.lock();

        costmap_ros_ = costmap_ros;
        tf_ = tf;
        plugin_name_ = name;
        logger_ = node->get_logger();
        clock_ = node->get_clock();

        declareAndGetParams();

        // Initialize command velocity
        cmd_vel_.setZero();
        t_prev_ = clock_->now();

        // Configure velocity optimizer
        velocity_optimizer_.configureOptimizer(nav2_plugin_params_.ocp.shooting_nodes);

        // create acados solver
        ptr_controller_ = std::make_shared<sipoc_mr_solver::CapsuleNominalController>(nav2_plugin_params_.ocp);
        ptr_controller_->setupOcpSolver();

        // state_predictor_filter_filter_
        Eigen::Matrix<double, 4, 4> A;
        Eigen::Matrix<double, 4, 2> B;
        Eigen::Matrix<double, 2, 4> C;
        Eigen::Matrix<double, 2, 2> D;
        Eigen::Matrix<double, 4, 4> Q;
        Eigen::Matrix<double, 2, 2> R;
        Eigen::Matrix<double, 4, 4> P;

        // Consider a discrete-time linear system for the robot dynamics:
        // \nu_{k+1} = disc_A \nu_k + disc_B (linVCmd_k, angVCmd_k)
        // (linVMeas_k, angVMeas_k) = dist_C \nu_k + dist_D (linVCmd_k, angVCmd_k),
        // where $\nu_k \in R^{4}$ represents the dynamic state.
        // The values of the disc_A, disc_B, dist_C, and dist_D matrices in the following are specific for the Neobotix MP-500 robot (https://www.neobotix-robots.com/products/mobile-robots/mobile-robot-mp-500) with a maximum forward velocity of 3m/s and angular velocity of 5rad/s,
        // and are obtained by using the N4SID method (https://nfoursid.readthedocs.io/en/latest/) based on the data collected from the robot.

        A << 0.73698832, 0.01597382, 0.06026152, 0.07303846,
            -0.0078049, 0.69396595, -0.11718393, 0.01482203,
            -0.77785672, 0.81754223, 0.92222123, -0.01603535,
            -1.2356698, -0.81602845, -0.02435645, 0.86768727;
        B << -4.16130160e-02, 3.29138983e-02,
            -4.30305613e-02, -5.23138700e-02,
            -1.93680236e-02, 2.28163012e-01,
            -3.16698394e-01, -1.01792606e-04;
        C << -3.70147307, -2.65703515, -0.16787712, -0.21704129,
            2.50575244, -3.52984477, 0.24767055, -0.28634375;
        D << 0.02128009, -0.03711049,
            -0.06290439, 0.09915334;

        R << 5.47929084e-04, 1.29847829e-04,
            1.29847829e-04, 2.38751462e-03;
        Q << 1.66671625e-05, -2.00701988e-06, 2.51139125e-05, -3.08985736e-05,
            -2.00701988e-06, 2.89403774e-05, -5.06132829e-06, 2.27151396e-05,
            2.51139125e-05, -5.06132829e-06, 1.45129030e-04, -4.10594701e-05,
            -3.08985736e-05, 2.27151396e-05, -4.10594701e-05, 2.27623707e-04;
        P = Eigen::Matrix<double, 4, 4>::Identity() * 1e-5;
        // NOTE: The discretization time step is 1.0 / controller_frequency
        ptr_state_predictor_filter_ = std::make_shared<sipoc_mr_utils::StatePredictorFilter>(A, B, C, D, Q, R, P, 1.0 / nav2_plugin_params_.local_planner.controller_frequency);
        ptr_state_predictor_filter_->init();
        cb_acc_hist_.set_capacity(2 + ptr_state_predictor_filter_->NumStepDelayComp() + ptr_state_predictor_filter_->NumStepDelayMeas());
        cb_cmd_vel_hist_.set_capacity(2 + ptr_state_predictor_filter_->NumStepDelayComp() + ptr_state_predictor_filter_->NumStepDelayMeas());
        this->resetFilterBuffers();

        // create publisher
        pub_local_plan_ = node->create_publisher<nav_msgs::msg::Path>("local_plan", 1);
        pub_ref_path_ = node->create_publisher<visualization_msgs::msg::MarkerArray>("sipoc_reference_path", 1);
        pub_ref_traj_ = node->create_publisher<visualization_msgs::msg::MarkerArray>("sipoc_reference_trajectory", 1);
        pub_timings_ = node->create_publisher<sipoc_ros2_interfaces::msg::SipocControllerTimings>("sipoc_timings", 1);
        pub_obs_ = node->create_publisher<visualization_msgs::msg::Marker>("obs_imposed_constr", 1);
        pub_vel_values_ = node->create_publisher<geometry_msgs::msg::TwistStamped>("vel_ocp_t0", 1);

        // create subscription
        if (nav2_plugin_params_.if_subscribe_laser_scan)
        {
            sub_laser_scan_ = node->create_subscription<sensor_msgs::msg::LaserScan>("scan", 1, std::bind(&NominalCapsuleController::FilteredLaserCallback, this, std::placeholders::_1));
            sub_occ_map_ = nullptr;
        }
        else
        {
            sub_occ_map_ = node->create_subscription<nav_msgs::msg::OccupancyGrid>("global_costmap", 1, std::bind(&NominalCapsuleController::OccupancyMapCallback, this, std::placeholders::_1));
            sub_laser_scan_ = nullptr;
        }
    }

    void NominalCapsuleController::declareAndGetParams()
    {
        auto node = node_.lock();

        declare_parameter_if_not_declared(
            node, plugin_name_ + ".transform_tolerance", rclcpp::ParameterValue(0.1));
        double d_transform_tolerance;
        node->get_parameter(plugin_name_ + ".transform_tolerance", d_transform_tolerance);
        transform_tolerance_ = rclcpp::Duration::from_seconds(d_transform_tolerance);

        std::vector<double> sub_delta_t;
        declare_parameter_if_not_declared(node, plugin_name_ + ".sub_delta_t", rclcpp::ParameterValue(sub_delta_t));
        node->get_parameter(plugin_name_ + ".sub_delta_t", sub_delta_t);
        std::vector<long int> sub_n_hrzn;
        declare_parameter_if_not_declared(node, plugin_name_ + ".sub_n_hrzn", rclcpp::ParameterValue(sub_n_hrzn));
        node->get_parameter(plugin_name_ + ".sub_n_hrzn", sub_n_hrzn);
        rcpputils::require_true(sub_delta_t.size() == sub_n_hrzn.size(), "The length of sub_delta_t should be the same as the length of sub_n_hrzn.");
        nav2_plugin_params_.ocp.ocp_dims.n_hrzn = 0;
        nav2_plugin_params_.ocp.shooting_nodes = {0.};
        double last_shooting_node = 0.;
        for (unsigned int idx = 0; idx < sub_n_hrzn.size(); ++idx)
        {
            nav2_plugin_params_.ocp.ocp_dims.n_hrzn += sub_n_hrzn[idx];
            for (unsigned int jj = 0; jj < sub_n_hrzn[idx]; ++jj)
            {
                last_shooting_node += sub_delta_t[idx];
                nav2_plugin_params_.ocp.shooting_nodes.push_back(last_shooting_node);
            }
        }

        declare_parameter_if_not_declared(
            node, "controller_frequency", rclcpp::ParameterValue(nav2_plugin_params_.local_planner.controller_frequency));
        node->get_parameter("controller_frequency", nav2_plugin_params_.local_planner.controller_frequency);

        bool ocp_disc_int_ctrl_freq_align = true;
        declare_parameter_if_not_declared(
            node, plugin_name_ + ".ocp_disc_int_ctrl_freq_align", rclcpp::ParameterValue(ocp_disc_int_ctrl_freq_align));
        node->get_parameter(plugin_name_ + ".ocp_disc_int_ctrl_freq_align", ocp_disc_int_ctrl_freq_align);
        if (ocp_disc_int_ctrl_freq_align)
        {
            rcpputils::require_true(std::abs(nav2_plugin_params_.ocp.shooting_nodes[1] * nav2_plugin_params_.local_planner.controller_frequency - 1.0) <= 1e-4, "The product of ocp.shooting_nodes[1] and controller_frequency must be equal to 1.");
        }

        // Bounds
        declare_parameter_if_not_declared(
            node, plugin_name_ + ".ocp_bounds.min_forward_velocity", rclcpp::ParameterValue(nav2_plugin_params_.ocp.ocp_bounds.min_forward_velocity));
        declare_parameter_if_not_declared(
            node, plugin_name_ + ".ocp_bounds.max_forward_velocity", rclcpp::ParameterValue(nav2_plugin_params_.ocp.ocp_bounds.max_forward_velocity));
        declare_parameter_if_not_declared(
            node, plugin_name_ + ".ocp_bounds.max_angular_velocity", rclcpp::ParameterValue(nav2_plugin_params_.ocp.ocp_bounds.max_angular_velocity));
        declare_parameter_if_not_declared(
            node, plugin_name_ + ".ocp_bounds.min_forward_acceleration", rclcpp::ParameterValue(nav2_plugin_params_.ocp.ocp_bounds.min_forward_acceleration));
        declare_parameter_if_not_declared(
            node, plugin_name_ + ".ocp_bounds.max_forward_acceleration", rclcpp::ParameterValue(nav2_plugin_params_.ocp.ocp_bounds.max_forward_acceleration));
        declare_parameter_if_not_declared(
            node, plugin_name_ + ".ocp_bounds.max_angular_acceleration", rclcpp::ParameterValue(nav2_plugin_params_.ocp.ocp_bounds.max_angular_acceleration));
        declare_parameter_if_not_declared(
            node, plugin_name_ + ".ocp_bounds.max_forward_velocity_e", rclcpp::ParameterValue(nav2_plugin_params_.ocp.ocp_bounds.max_forward_velocity_e));
        declare_parameter_if_not_declared(
            node, plugin_name_ + ".ocp_bounds.max_angular_velocity_e", rclcpp::ParameterValue(nav2_plugin_params_.ocp.ocp_bounds.max_angular_velocity_e));
        node->get_parameter(plugin_name_ + ".ocp_bounds.min_forward_velocity", nav2_plugin_params_.ocp.ocp_bounds.min_forward_velocity);
        node->get_parameter(plugin_name_ + ".ocp_bounds.max_forward_velocity", nav2_plugin_params_.ocp.ocp_bounds.max_forward_velocity);
        node->get_parameter(plugin_name_ + ".ocp_bounds.max_angular_velocity", nav2_plugin_params_.ocp.ocp_bounds.max_angular_velocity);
        node->get_parameter(plugin_name_ + ".ocp_bounds.min_forward_acceleration", nav2_plugin_params_.ocp.ocp_bounds.min_forward_acceleration);
        node->get_parameter(plugin_name_ + ".ocp_bounds.max_forward_acceleration", nav2_plugin_params_.ocp.ocp_bounds.max_forward_acceleration);
        node->get_parameter(plugin_name_ + ".ocp_bounds.max_angular_acceleration", nav2_plugin_params_.ocp.ocp_bounds.max_angular_acceleration);
        node->get_parameter(plugin_name_ + ".ocp_bounds.max_forward_velocity_e", nav2_plugin_params_.ocp.ocp_bounds.max_forward_velocity_e);
        node->get_parameter(plugin_name_ + ".ocp_bounds.max_angular_velocity_e", nav2_plugin_params_.ocp.ocp_bounds.max_angular_velocity_e);

        // OCP iterations
        declare_parameter_if_not_declared(
            node, plugin_name_ + ".ocp_iters.max_iter_num", rclcpp::ParameterValue(static_cast<int>(nav2_plugin_params_.ocp.ocp_iters.max_iter_num)));
        declare_parameter_if_not_declared(
            node, plugin_name_ + ".ocp_iters.max_constr_num", rclcpp::ParameterValue(static_cast<int>(nav2_plugin_params_.ocp.ocp_iters.max_constr_num)));
        declare_parameter_if_not_declared(
            node, plugin_name_ + ".ocp_iters.eps_converg", rclcpp::ParameterValue(nav2_plugin_params_.ocp.ocp_iters.eps_converg));
        int temp_int;
        node->get_parameter(plugin_name_ + ".ocp_iters.max_iter_num", temp_int);
        nav2_plugin_params_.ocp.ocp_iters.max_iter_num = static_cast<unsigned int>(temp_int);
        node->get_parameter(plugin_name_ + ".ocp_iters.max_constr_num", temp_int);
        nav2_plugin_params_.ocp.ocp_iters.max_constr_num = static_cast<unsigned int>(temp_int);
        node->get_parameter(plugin_name_ + ".ocp_iters.eps_converg", nav2_plugin_params_.ocp.ocp_iters.eps_converg);

        // OCP dims
        declare_parameter_if_not_declared(
            node, plugin_name_ + ".ocp_dims.nx", rclcpp::ParameterValue(static_cast<int>(nav2_plugin_params_.ocp.ocp_dims.nx)));
        node->get_parameter(plugin_name_ + ".ocp_dims.nx", temp_int);
        nav2_plugin_params_.ocp.ocp_dims.nx = static_cast<unsigned int>(temp_int);

        // weights
        declare_parameter_if_not_declared(
            node, plugin_name_ + ".ocp_weights.position", rclcpp::ParameterValue(nav2_plugin_params_.ocp.ocp_weights.position));
        declare_parameter_if_not_declared(
            node, plugin_name_ + ".ocp_weights.heading", rclcpp::ParameterValue(nav2_plugin_params_.ocp.ocp_weights.heading));
        declare_parameter_if_not_declared(
            node, plugin_name_ + ".ocp_weights.forward_velocity", rclcpp::ParameterValue(nav2_plugin_params_.ocp.ocp_weights.forward_velocity));
        declare_parameter_if_not_declared(
            node, plugin_name_ + ".ocp_weights.angular_velocity", rclcpp::ParameterValue(nav2_plugin_params_.ocp.ocp_weights.angular_velocity));
        declare_parameter_if_not_declared(
            node, plugin_name_ + ".ocp_weights.forward_acceleration", rclcpp::ParameterValue(nav2_plugin_params_.ocp.ocp_weights.forward_acceleration));
        declare_parameter_if_not_declared(
            node, plugin_name_ + ".ocp_weights.angular_acceleration", rclcpp::ParameterValue(nav2_plugin_params_.ocp.ocp_weights.angular_acceleration));
        declare_parameter_if_not_declared(
            node, plugin_name_ + ".ocp_weights.slack_l1_cost", rclcpp::ParameterValue(nav2_plugin_params_.ocp.ocp_weights.slack_l1_cost));
        declare_parameter_if_not_declared(
            node, plugin_name_ + ".ocp_weights.slack_l2_cost", rclcpp::ParameterValue(nav2_plugin_params_.ocp.ocp_weights.slack_l2_cost));
        node->get_parameter(plugin_name_ + ".ocp_weights.position", nav2_plugin_params_.ocp.ocp_weights.position);
        node->get_parameter(plugin_name_ + ".ocp_weights.heading", nav2_plugin_params_.ocp.ocp_weights.heading);
        node->get_parameter(plugin_name_ + ".ocp_weights.forward_velocity", nav2_plugin_params_.ocp.ocp_weights.forward_velocity);
        node->get_parameter(plugin_name_ + ".ocp_weights.angular_velocity", nav2_plugin_params_.ocp.ocp_weights.angular_velocity);
        node->get_parameter(plugin_name_ + ".ocp_weights.forward_acceleration", nav2_plugin_params_.ocp.ocp_weights.forward_acceleration);
        node->get_parameter(plugin_name_ + ".ocp_weights.angular_acceleration", nav2_plugin_params_.ocp.ocp_weights.angular_acceleration);
        node->get_parameter(plugin_name_ + ".ocp_weights.slack_l1_cost", nav2_plugin_params_.ocp.ocp_weights.slack_l1_cost);
        node->get_parameter(plugin_name_ + ".ocp_weights.slack_l2_cost", nav2_plugin_params_.ocp.ocp_weights.slack_l2_cost);

        // sdf
        declare_parameter_if_not_declared(
            node, plugin_name_ + ".sdf.map_height", rclcpp::ParameterValue(nav2_plugin_params_.ocp.sdf.map_height));
        declare_parameter_if_not_declared(
            node, plugin_name_ + ".sdf.map_width", rclcpp::ParameterValue(nav2_plugin_params_.ocp.sdf.map_width));
        declare_parameter_if_not_declared(
            node, plugin_name_ + ".sdf.grid_size", rclcpp::ParameterValue(nav2_plugin_params_.ocp.sdf.grid_size));
        declare_parameter_if_not_declared(
            node, plugin_name_ + ".sdf.lb_px_grid", rclcpp::ParameterValue(nav2_plugin_params_.ocp.sdf.lb_px_grid));
        declare_parameter_if_not_declared(
            node, plugin_name_ + ".sdf.lb_py_grid", rclcpp::ParameterValue(nav2_plugin_params_.ocp.sdf.lb_py_grid));
        declare_parameter_if_not_declared(
            node, plugin_name_ + ".sdf.map_occ_threshold", rclcpp::ParameterValue(nav2_plugin_params_.ocp.sdf.map_occ_threshold));
        node->get_parameter(plugin_name_ + ".sdf.map_height", nav2_plugin_params_.ocp.sdf.map_height);
        node->get_parameter(plugin_name_ + ".sdf.map_width", nav2_plugin_params_.ocp.sdf.map_width);
        node->get_parameter(plugin_name_ + ".sdf.grid_size", nav2_plugin_params_.ocp.sdf.grid_size);
        node->get_parameter(plugin_name_ + ".sdf.lb_px_grid", nav2_plugin_params_.ocp.sdf.lb_px_grid);
        node->get_parameter(plugin_name_ + ".sdf.lb_py_grid", nav2_plugin_params_.ocp.sdf.lb_py_grid);
        node->get_parameter(plugin_name_ + ".sdf.map_occ_threshold", nav2_plugin_params_.ocp.sdf.map_occ_threshold);

        // robot
        declare_parameter_if_not_declared(
            node, plugin_name_ + ".robot.num_gamma_grids", rclcpp::ParameterValue(nav2_plugin_params_.ocp.robot.num_gamma_grids));
        declare_parameter_if_not_declared(
            node, plugin_name_ + ".robot.robot_capsule_half_length", rclcpp::ParameterValue(nav2_plugin_params_.ocp.robot.robot_capsule_half_length));
        declare_parameter_if_not_declared(
            node, plugin_name_ + ".robot.robot_capsule_radius", rclcpp::ParameterValue(nav2_plugin_params_.ocp.robot.robot_capsule_radius));
        node->get_parameter(plugin_name_ + ".robot.num_gamma_grids", nav2_plugin_params_.ocp.robot.num_gamma_grids);
        node->get_parameter(plugin_name_ + ".robot.robot_capsule_half_length", nav2_plugin_params_.ocp.robot.robot_capsule_half_length);
        node->get_parameter(plugin_name_ + ".robot.robot_capsule_radius", nav2_plugin_params_.ocp.robot.robot_capsule_radius);

        // nav
        declare_parameter_if_not_declared(
            node, plugin_name_ + ".nav.path_following_tail_length", rclcpp::ParameterValue(nav2_plugin_params_.local_planner.nav.path_following_tail_length));
        declare_parameter_if_not_declared(
            node, plugin_name_ + ".nav.path_following_reference_length", rclcpp::ParameterValue(nav2_plugin_params_.local_planner.nav.path_following_reference_length));
        declare_parameter_if_not_declared(
            node, plugin_name_ + ".nav.path_following_min_step", rclcpp::ParameterValue(nav2_plugin_params_.local_planner.nav.path_following_min_step));
        node->get_parameter(plugin_name_ + ".nav.path_following_tail_length", nav2_plugin_params_.local_planner.nav.path_following_tail_length);
        node->get_parameter(plugin_name_ + ".nav.path_following_reference_length", nav2_plugin_params_.local_planner.nav.path_following_reference_length);
        node->get_parameter(plugin_name_ + ".nav.path_following_min_step", nav2_plugin_params_.local_planner.nav.path_following_min_step);

        // delay
        declare_parameter_if_not_declared(
            node, plugin_name_ + ".delay.dt", rclcpp::ParameterValue(nav2_plugin_params_.local_planner.delay.dt));
        declare_parameter_if_not_declared(
            node, plugin_name_ + ".delay.tau_v", rclcpp::ParameterValue(nav2_plugin_params_.local_planner.delay.tau_v));
        declare_parameter_if_not_declared(
            node, plugin_name_ + ".delay.tau_w", rclcpp::ParameterValue(nav2_plugin_params_.local_planner.delay.tau_w));
        node->get_parameter(plugin_name_ + ".delay.dt", nav2_plugin_params_.local_planner.delay.dt);
        node->get_parameter(plugin_name_ + ".delay.tau_v", nav2_plugin_params_.local_planner.delay.tau_v);
        node->get_parameter(plugin_name_ + ".delay.tau_w", nav2_plugin_params_.local_planner.delay.tau_w);

        // velopt
        declare_parameter_if_not_declared(
            node, plugin_name_ + ".velopt.mpc_factor", rclcpp::ParameterValue(nav2_plugin_params_.local_planner.velopt.mpc_factor));
        declare_parameter_if_not_declared(
            node, plugin_name_ + ".velopt.max_lat_acc", rclcpp::ParameterValue(nav2_plugin_params_.local_planner.velopt.max_lat_acc));
        declare_parameter_if_not_declared(
            node, plugin_name_ + ".velopt.max_ang_vel", rclcpp::ParameterValue(nav2_plugin_params_.local_planner.velopt.max_ang_vel));
        declare_parameter_if_not_declared(
            node, plugin_name_ + ".velopt.track_width", rclcpp::ParameterValue(nav2_plugin_params_.local_planner.velopt.track_width));
        declare_parameter_if_not_declared(
            node, plugin_name_ + ".velopt.min_wheel_vel", rclcpp::ParameterValue(nav2_plugin_params_.local_planner.velopt.min_wheel_vel));
        declare_parameter_if_not_declared(
            node, plugin_name_ + ".velopt.max_wheel_vel", rclcpp::ParameterValue(nav2_plugin_params_.local_planner.velopt.max_wheel_vel));
        declare_parameter_if_not_declared(
            node, plugin_name_ + ".velopt.max_wheel_acc", rclcpp::ParameterValue(nav2_plugin_params_.local_planner.velopt.max_wheel_acc));
        declare_parameter_if_not_declared(
            node, plugin_name_ + ".velopt.max_wheel_dec", rclcpp::ParameterValue(nav2_plugin_params_.local_planner.velopt.max_wheel_dec));
        node->get_parameter(plugin_name_ + ".velopt.mpc_factor", nav2_plugin_params_.local_planner.velopt.mpc_factor);
        node->get_parameter(plugin_name_ + ".velopt.max_lat_acc", nav2_plugin_params_.local_planner.velopt.max_lat_acc);
        node->get_parameter(plugin_name_ + ".velopt.max_ang_vel", nav2_plugin_params_.local_planner.velopt.max_ang_vel);
        node->get_parameter(plugin_name_ + ".velopt.track_width", nav2_plugin_params_.local_planner.velopt.track_width);
        node->get_parameter(plugin_name_ + ".velopt.min_wheel_vel", nav2_plugin_params_.local_planner.velopt.min_wheel_vel);
        node->get_parameter(plugin_name_ + ".velopt.max_wheel_vel", nav2_plugin_params_.local_planner.velopt.max_wheel_vel);
        node->get_parameter(plugin_name_ + ".velopt.max_wheel_acc", nav2_plugin_params_.local_planner.velopt.max_wheel_acc);
        node->get_parameter(plugin_name_ + ".velopt.max_wheel_dec", nav2_plugin_params_.local_planner.velopt.max_wheel_dec);

        // occupancy map
        declare_parameter_if_not_declared(
            node, plugin_name_ + ".if_subscribe_laser_scan", rclcpp::ParameterValue(nav2_plugin_params_.if_subscribe_laser_scan));
        node->get_parameter(plugin_name_ + ".if_subscribe_laser_scan", nav2_plugin_params_.if_subscribe_laser_scan);

        // visualization
        declare_parameter_if_not_declared(
            node, plugin_name_ + ".visualization.npoints_reference_path", rclcpp::ParameterValue(nav2_plugin_params_.visualization.npoints_reference_path));
        declare_parameter_if_not_declared(
            node, plugin_name_ + ".visualization.publish_reference_path", rclcpp::ParameterValue(nav2_plugin_params_.visualization.publish_reference_path));
        declare_parameter_if_not_declared(
            node, plugin_name_ + ".visualization.publish_reference_trajectory", rclcpp::ParameterValue(nav2_plugin_params_.visualization.publish_reference_trajectory));
        declare_parameter_if_not_declared(
            node, plugin_name_ + ".visualization.publish_local_plan", rclcpp::ParameterValue(nav2_plugin_params_.visualization.publish_local_plan));
        declare_parameter_if_not_declared(
            node, plugin_name_ + ".visualization.publish_timings", rclcpp::ParameterValue(nav2_plugin_params_.visualization.publish_timings));
        declare_parameter_if_not_declared(
            node, plugin_name_ + ".visualization.publish_obs_imposed_constr", rclcpp::ParameterValue(nav2_plugin_params_.visualization.publish_obs_imposed_constr));
        declare_parameter_if_not_declared(
            node, plugin_name_ + ".visualization.publish_predicted_vel_values", rclcpp::ParameterValue(nav2_plugin_params_.visualization.publish_predicted_vel_values));
        node->get_parameter(plugin_name_ + ".visualization.npoints_reference_path", nav2_plugin_params_.visualization.npoints_reference_path);
        node->get_parameter(plugin_name_ + ".visualization.publish_reference_path", nav2_plugin_params_.visualization.publish_reference_path);
        node->get_parameter(plugin_name_ + ".visualization.publish_reference_trajectory", nav2_plugin_params_.visualization.publish_reference_trajectory);
        node->get_parameter(plugin_name_ + ".visualization.publish_local_plan", nav2_plugin_params_.visualization.publish_local_plan);
        node->get_parameter(plugin_name_ + ".visualization.publish_timings", nav2_plugin_params_.visualization.publish_timings);
        node->get_parameter(plugin_name_ + ".visualization.publish_obs_imposed_constr", nav2_plugin_params_.visualization.publish_obs_imposed_constr);
        node->get_parameter(plugin_name_ + ".visualization.publish_predicted_vel_values", nav2_plugin_params_.visualization.publish_predicted_vel_values);

        // log
        declare_parameter_if_not_declared(
            node, plugin_name_ + ".log.robot_current_state", rclcpp::ParameterValue(nav2_plugin_params_.log.robot_current_state));
        node->get_parameter(plugin_name_ + ".log.robot_current_state", nav2_plugin_params_.log.robot_current_state);
        declare_parameter_if_not_declared(
            node, plugin_name_ + ".log.nlp_sol_status", rclcpp::ParameterValue(nav2_plugin_params_.log.nlp_sol_status));
        node->get_parameter(plugin_name_ + ".log.nlp_sol_status", nav2_plugin_params_.log.nlp_sol_status);
    }

    void NominalCapsuleController::activate()
    {
        RCLCPP_INFO(logger_, "Activating controller: sipoc_mr_nav2_controller::NominalCapsuleController");

        pub_ref_path_->on_activate();
        pub_ref_traj_->on_activate();
        pub_local_plan_->on_activate();
        pub_timings_->on_activate();
        pub_obs_->on_activate();
        pub_vel_values_->on_activate();

        yref_ = (double *)malloc(nav2_plugin_params_.ocp.ocp_dims.n_hrzn * nav2_plugin_params_.ocp.ocp_dims.ny * sizeof(double));
        yref_e_ = (double *)malloc(nav2_plugin_params_.ocp.ocp_dims.nyn * sizeof(double));
    }

    void NominalCapsuleController::deactivate()
    {
        RCLCPP_INFO(logger_, "Dectivating controller: sipoc_mr_nav2_controller::NominalCapsuleController");
        pub_ref_path_->on_deactivate();
        pub_ref_traj_->on_deactivate();
        pub_local_plan_->on_deactivate();
        pub_timings_->on_deactivate();
        pub_obs_->on_deactivate();
        pub_vel_values_->on_deactivate();
    }

    void NominalCapsuleController::cleanup()
    {
        RCLCPP_INFO(logger_, "Cleaning up controller: sipoc_mr_nav2_controller::NominalCapsuleController");
        pub_ref_path_.reset();
        pub_ref_traj_.reset();
        pub_local_plan_.reset();
        pub_timings_.reset();
        pub_obs_.reset();
        pub_vel_values_.reset();
        free(yref_);
        free(yref_e_);
    }

    void NominalCapsuleController::setSpeedLimit(const double &speed_limit, const bool &percentage)
    {
        RCLCPP_INFO(logger_, "Speed limits are treated in the OCP");
        (void)speed_limit;
        (void)percentage;
    }

    void NominalCapsuleController::setPlan(const nav_msgs::msg::Path &path)
    {
        global_plan_ = path;
    }

    geometry_msgs::msg::TwistStamped NominalCapsuleController::computeVelocityCommands(
        const geometry_msgs::msg::PoseStamped &pose,
        const geometry_msgs::msg::Twist &velocity, nav2_core::GoalChecker *)
    {
        auto node = node_.lock();
        // Record time
        rclcpp::Time t = clock_->now();
        auto deadline = std::chrono::system_clock::now() + std::chrono::microseconds(static_cast<long int>(1e6 * nav2_plugin_params_.local_planner.delay.dt));

        const auto time_start = std::chrono::steady_clock::now();

        if (!occ_map_updated_)
        {
            if ((t - t_prev_).seconds() >= 1.0)
            {
                RCLCPP_INFO(logger_, "Occupancy map not received.");
                t_prev_ = t;
            }
            geometry_msgs::msg::TwistStamped cmd_vel_msg;
            cmd_vel_msg.twist.linear.x = cmd_vel_msg.twist.linear.y = cmd_vel_msg.twist.angular.z = 0;
            return cmd_vel_msg;
        }

        // In case the last control command was a "long time ago"
        if (t - t_prev_ > rclcpp::Duration::from_nanoseconds(5.0 / nav2_plugin_params_.local_planner.controller_frequency * 1e9 /*ns*/))
        {
            cmd_vel_.setZero();
            this->resetFilterBuffers();
            RCLCPP_INFO(logger_, "%f ms have passed since the last call, resetting velocity commands.", (t - t_prev_).seconds() * 1e3);
        }

        // Kalmen filter for state estimation
        Eigen::Vector3d pose_measured;
        pose_measured << pose.pose.position.x, pose.pose.position.y, tf2::getYaw(pose.pose.orientation);
        Eigen::Vector2d v_measured;
        v_measured << velocity.linear.x, velocity.angular.z;

        ptr_state_predictor_filter_->update(pose_measured, v_measured, cb_cmd_vel_hist_.front());
        Eigen::Vector<double, 9> ocp_init_state;
        ptr_state_predictor_filter_->predictOCPInitialState(cb_acc_hist_, ocp_init_state, true);
        ocp_init_state(3) = std::clamp(ocp_init_state(3), nav2_plugin_params_.ocp.ocp_bounds.min_forward_velocity, nav2_plugin_params_.ocp.ocp_bounds.max_forward_velocity);
        ocp_init_state(4) = std::clamp(ocp_init_state(4), -nav2_plugin_params_.ocp.ocp_bounds.max_angular_velocity, nav2_plugin_params_.ocp.ocp_bounds.max_angular_velocity);

        // Interpret current state
        manif::SE2<double> robot_pose(ocp_init_state(0), ocp_init_state(1), ocp_init_state(2));
        manif::SE2Tangent<double> robot_twist(ocp_init_state(3), 0., ocp_init_state(4));

        // Obtain points for reference path
        ReferencePath reference_path;
        buildReferencePath(global_plan_, robot_pose, reference_path);

        // Compute reference trajectory
        computeReferenceTraj(robot_pose, robot_twist, reference_path);
        const auto time_ref_traj_end = std::chrono::steady_clock::now();
        nlp_preparation_elapsed_time_ = static_cast<double>(std::chrono::duration_cast<std::chrono::microseconds>(time_start - time_ref_traj_end).count() / 1000.0);
        ptr_controller_->updateInitialState(ocp_init_state);
        ptr_controller_->updateReferenceTrajectory(yref_, yref_e_);

        // Compute control
        if (!ptr_controller_->solver_initialized)
        {
            ptr_controller_->initializeOCPByReference();
            ptr_controller_->solveEndpointGammaConstrainedSubproblem();
            // ptr_state_predictor_filter_->init();
            ptr_controller_->solver_initialized = true;
        }
        ptr_controller_->resetElapsedTime();
        sipoc_mr_utils::SIPSolverStatus solver_status = ptr_controller_->solveOCP(true);
        if (nav2_plugin_params_.log.nlp_sol_status)
        {
            RCLCPP_INFO(logger_, "nlp solver status (0:success, 1:fail, 2:notConverged) %d", solver_status);
        }

        // Integrate and cap
        Eigen::Vector<double, 2> u0_star;
        ptr_controller_->exportU0Star(u0_star);

        // modify cb_acc_hist_ and cb_cmd_vel_hist_
        cb_acc_hist_.pop_front();
        cb_acc_hist_.push_back(u0_star);
        cb_cmd_vel_hist_.pop_front();
        cb_cmd_vel_hist_.push_back(cmd_vel_);

        cmd_vel_ += u0_star / nav2_plugin_params_.local_planner.controller_frequency;
        /// cap velocity reference
        if (cmd_vel_(0) < nav2_plugin_params_.ocp.ocp_bounds.min_forward_velocity)
            cmd_vel_(0) = nav2_plugin_params_.ocp.ocp_bounds.min_forward_velocity;
        if (cmd_vel_(0) > nav2_plugin_params_.ocp.ocp_bounds.max_forward_velocity)
            cmd_vel_(0) = nav2_plugin_params_.ocp.ocp_bounds.max_forward_velocity;
        if (cmd_vel_(1) < -nav2_plugin_params_.ocp.ocp_bounds.max_angular_velocity)
            cmd_vel_(1) = -nav2_plugin_params_.ocp.ocp_bounds.max_angular_velocity;
        if (cmd_vel_(1) > nav2_plugin_params_.ocp.ocp_bounds.max_angular_velocity)
            cmd_vel_(1) = nav2_plugin_params_.ocp.ocp_bounds.max_angular_velocity;

        if (solver_status == sipoc_mr_utils::acadosFail)
        {
            RCLCPP_ERROR(logger_, "acados failed");
            // cmd_vel_.setZero();
            ptr_controller_->solver_initialized = false;
        }
        else
        {
            // warm start for OCP of next time step
            // NOTE: u0_star will be overwritten
            ptr_controller_->shiftInputStateTrajectory(1);
        }

        const auto time_end = std::chrono::steady_clock::now();
        total_elapsed_time_ = static_cast<double>(std::chrono::duration_cast<std::chrono::microseconds>(time_start - time_end).count() / 1000.0);

        // Store time
        t_prev_ = t;

        publishReferencePath(t);
        publishReferenceTrajectory(t);
        publishLocalPlan(t);
        publishTimings(t);
        publishObstacle(t);
        publishVelPredictOCP(t, robot_twist);

        geometry_msgs::msg::TwistStamped cmd_vel_msg;
        cmd_vel_msg.twist.linear.x = cmd_vel_msg.twist.linear.y = cmd_vel_msg.twist.angular.z = 0;
        cmd_vel_msg.header.stamp = clock_->now();
        cmd_vel_msg.twist.linear.x = cmd_vel_.coeff(0);  // x_opt_[1][3];
        cmd_vel_msg.twist.angular.z = cmd_vel_.coeff(1); // x_opt_[1][4];

        // Wait until controller frequency time has passed
        if (std::chrono::system_clock::now() < deadline)
        {
            std::this_thread::sleep_until(deadline);
        }
        return cmd_vel_msg;
    }

    void NominalCapsuleController::OccupancyMapCallback(nav_msgs::msg::OccupancyGrid msg)
    {
        if (!occ_map_updated_)
        {
            RCLCPP_INFO(logger_, "sg.info.height%d, nav2_plugin_params_.ocp.sdf.map_height%d", msg.info.height, nav2_plugin_params_.ocp.sdf.map_height);
            rcpputils::require_true(std::abs(msg.info.resolution - nav2_plugin_params_.ocp.sdf.grid_size) <= 1e-4, "msg.info.resolution must be equal to nav2_plugin_params_.ocp.sdf.grid_size");
            rcpputils::require_true(static_cast<int>(msg.info.width) == nav2_plugin_params_.ocp.sdf.map_width, "msg.info.width must be equal to nav2_plugin_params_.ocp.sdf.map_width");
            rcpputils::require_true(static_cast<int>(msg.info.height) == nav2_plugin_params_.ocp.sdf.map_height, "msg.info.height must be equal to nav2_plugin_params_.ocp.sdf.map_height");
            rcpputils::require_true(std::abs(msg.info.origin.position.x - nav2_plugin_params_.ocp.sdf.lb_px_grid) <= 1e-4, "msg.info.origin.position.x must be equal to nav2_plugin_params_.ocp.sdf.lb_px_grid");
            rcpputils::require_true(std::abs(msg.info.origin.position.y - nav2_plugin_params_.ocp.sdf.lb_py_grid) <= 1e-4, "msg.info.origin.position.y must be equal to nav2_plugin_params_.ocp.sdf.lb_py_grid");
        }
        ptr_controller_->UpdateOccAndSDF(msg.data, nav2_plugin_params_.ocp.sdf.map_occ_threshold);

        occ_map_updated_ = true;
    }

    void NominalCapsuleController::FilteredLaserCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
    {
        // MUTEX NEEDED?!?
        auto laser_scan_frame = msg->header.frame_id;
        manif::SE2<double> laser_scan_pose;
        if (!tf_->canTransform("map", laser_scan_frame, tf2::TimePointZero))
        {
            RCLCPP_WARN(logger_, "Cannot transform laser scan frame %s to `map` frame", laser_scan_frame.c_str());
            return;
        }
        try
        {
            geometry_msgs::msg::TransformStamped laser_scan_transform = tf_->lookupTransform("map", laser_scan_frame, tf2::TimePointZero);
            double laser_scan_x = laser_scan_transform.transform.translation.x;
            double laser_scan_y = laser_scan_transform.transform.translation.y;
            double laser_scan_angle = tf2::getYaw(laser_scan_transform.transform.rotation);
            laser_scan_pose = manif::SE2<double>(laser_scan_x, laser_scan_y, laser_scan_angle);
        }
        catch (const tf2::TransformException &ex)
        {
            RCLCPP_ERROR(logger_, "Transform error: %s", ex.what());
            return;
        }

        std::vector<int8_t> laser_data_grid(nav2_plugin_params_.ocp.sdf.map_width * nav2_plugin_params_.ocp.sdf.map_height, 0);
        const double angle_increment = msg->angle_increment;
        const double angle_min = msg->angle_min;
        const double angle_max = msg->angle_max;
        const double range_min = msg->range_min;
        const double range_max = msg->range_max;

        // Assuming the laser scan data appears counter-clockwise
        // https://docs.ros2.org/foxy/api/sensor_msgs/msg/LaserScan.html
        for (size_t i = 0; i < msg->ranges.size(); ++i)
        {
            double angle = angle_min + i * angle_increment;
            if (angle < angle_min || angle > angle_max)
                continue;

            double range = msg->ranges[i];
            if (range < range_min || range > range_max)
                continue;

            // Convert polar coordinates to Cartesian coordinates
            // Add thickness to the observed point along the ray
            std::array<double, 5> range_values = {0., 0., 0., 0., 0.};
            for (size_t j = 0; j < range_values.size(); ++j)
            {
                range_values[j] = range + j * nav2_plugin_params_.ocp.sdf.grid_size;
            }
            for (const double range_j : range_values)
            {
                Eigen::Vector2d range_i_xy(range_j * cos(-angle), range_j * sin(-angle));
                Eigen::Vector2d range_i_xy_world = laser_scan_pose.act(range_i_xy);
                double x = range_i_xy_world.x();
                double y = range_i_xy_world.y();

                // Convert to grid coordinates
                int grid_x = static_cast<int>((x - nav2_plugin_params_.ocp.sdf.lb_px_grid + 0.5 * nav2_plugin_params_.ocp.sdf.grid_size) / nav2_plugin_params_.ocp.sdf.grid_size);
                int grid_y = static_cast<int>((y - nav2_plugin_params_.ocp.sdf.lb_py_grid + 0.5 * nav2_plugin_params_.ocp.sdf.grid_size) / nav2_plugin_params_.ocp.sdf.grid_size);

                if (grid_x >= 0 && grid_x < nav2_plugin_params_.ocp.sdf.map_width &&
                    grid_y >= 0 && grid_y < nav2_plugin_params_.ocp.sdf.map_height)
                {
                    laser_data_grid[grid_y * nav2_plugin_params_.ocp.sdf.map_width + grid_x] = nav2_plugin_params_.ocp.sdf.map_occ_threshold + 1; // Mark as occupied
                }
            }
        }

        ptr_controller_->UpdateOccAndSDF(laser_data_grid, nav2_plugin_params_.ocp.sdf.map_occ_threshold);

        occ_map_updated_ = true;
    }

    bool NominalCapsuleController::buildReferencePath(const nav_msgs::msg::Path &global_plan,
                                                      const manif::SE2<double> &current_pose,
                                                      ReferencePath &reference_path)
    {
        const double tail_length = nav2_plugin_params_.local_planner.nav.path_following_tail_length;
        const double reference_length = nav2_plugin_params_.local_planner.nav.path_following_reference_length;
        const double min_step = nav2_plugin_params_.local_planner.nav.path_following_min_step;
        double accumulated_dist = 0;

        // Catch case where path is of length 0 or 1
        if (global_plan.poses.size() < 2)
        {
            reference_path.push_back(current_pose.translation());
            reference_path.push_back(current_pose.translation());
            return true;
        }

        // Find nearest point along the path
        uint32_t i_path = 0;
        double least_distance_squared = std::numeric_limits<double>::max();
        for (uint32_t idx = 1; idx < global_plan.poses.size(); ++idx)
        {
            const auto &p = global_plan.poses[idx];

            // compute squared distance of global plan point to the current robot pose
            double dx = p.pose.position.x - current_pose.x();
            double dy = p.pose.position.y - current_pose.y();
            double dist_squared = dx * dx + dy * dy;

            if (dist_squared < least_distance_squared)
            {
                least_distance_squared = dist_squared;
                i_path = idx;
            }
        }

        // Walk backwards to achieve tail distance
        accumulated_dist = 0;
        while (i_path > 0 && accumulated_dist < tail_length)
        {
            i_path--;
            auto prev_pos = global_plan.poses[i_path + 1].pose.position;
            auto curr_pos = global_plan.poses[i_path].pose.position;

            double dx = curr_pos.x - prev_pos.x;
            double dy = curr_pos.y - prev_pos.y;
            accumulated_dist += sqrt(dx * dx + dy * dy);
        }

        // Walk forwards and collect reference points
        uint32_t added_points = 0;
        accumulated_dist = 0;
        // Add first point
        {
            auto curr_pos = global_plan.poses[i_path].pose.position;

            WorldPoint ref_pos;
            ref_pos << curr_pos.x, curr_pos.y;
            reference_path.push_back(ref_pos);
            added_points++;

            i_path++;
        }
        // Add points until reference length is achieved
        while (i_path < global_plan.poses.size() && accumulated_dist < reference_length)
        {
            auto curr_pos = global_plan.poses[i_path].pose.position;

            double dx = curr_pos.x - reference_path[added_points - 1](0);
            double dy = curr_pos.y - reference_path[added_points - 1](1);
            double step_size = sqrt(dx * dx + dy * dy);

            if (step_size >= min_step)
            {
                WorldPoint ref_pos;
                ref_pos << curr_pos.x, curr_pos.y;
                reference_path.push_back(ref_pos);
                accumulated_dist += step_size;
                added_points++;
            }

            i_path++;
        }

        return true;
    }

    void NominalCapsuleController::resetFilterBuffers()
    {
        cb_acc_hist_.clear();
        cb_cmd_vel_hist_.clear();
        // NOTE: Assuming that NumStepDelayMeas = 1 and NumStepDelayComp = 1
        for (unsigned int idx = 0; idx < 2; ++idx)
        {
            cb_acc_hist_.push_back(Eigen::Vector2d::Zero());
        }
        for (unsigned int idx = 0; idx < 2; ++idx)
        {
            cb_cmd_vel_hist_.push_back(Eigen::Vector2d::Zero());
        }
    }

    void NominalCapsuleController::computeReferenceTraj(const manif::SE2<double> &current_pose,
                                                        const manif::SE2Tangent<double> &current_vel,
                                                        ReferencePath &reference_path)
    {
        if (nav2_plugin_params_.log.robot_current_state)
        {
            RCLCPP_INFO(logger_, " =========== compute Reference Traj =========== \n current_pose=%f, %f, %f, %f, %f", current_pose.x(), current_pose.y(), current_pose.angle(), current_vel.x(), current_vel.angle());
        }

        velocity_optimizer_.setRefPath(reference_path);
        std::vector<double> robot_state = {current_pose.x(), current_pose.y(), current_pose.angle(), current_vel.x(), current_vel.angle()};
        velocity_optimizer_.computeReferenceTraj(robot_state);
        const std::vector<double> &x_ref = velocity_optimizer_.referenceTrajConstRef();

        for (int idx = 0; idx < nav2_plugin_params_.ocp.ocp_dims.n_hrzn; ++idx)
        {
            std::copy(x_ref.begin() + idx * 5, x_ref.begin() + idx * 5 + 5, yref_ + idx * nav2_plugin_params_.ocp.ocp_dims.ny);
        }
        yref_e_[0] = x_ref[nav2_plugin_params_.ocp.ocp_dims.n_hrzn * 5];
        yref_e_[1] = x_ref[nav2_plugin_params_.ocp.ocp_dims.n_hrzn * 5 + 1];
        yref_e_[2] = x_ref[nav2_plugin_params_.ocp.ocp_dims.n_hrzn * 5 + 2];
        yref_e_[3] = x_ref[nav2_plugin_params_.ocp.ocp_dims.n_hrzn * 5 + 3];
        yref_e_[4] = x_ref[nav2_plugin_params_.ocp.ocp_dims.n_hrzn * 5 + 4];
    }

    void NominalCapsuleController::publishReferencePath(const rclcpp::Time &t)
    {
        if (nav2_plugin_params_.visualization.publish_reference_path)
        {
            const uint32_t npoints = nav2_plugin_params_.visualization.npoints_reference_path;

            // compute center and radius
            ReferencePath reference_path(npoints);
            const ReferenceSpline &reference_spline = velocity_optimizer_.referenceSplineConstRef();
            double step = 1 / (npoints - 1.0);
            for (std::size_t i = 0; i < npoints; ++i)
            {
                reference_path[i] = reference_spline(i * step);
            }

            visualization_msgs::msg::MarkerArray markers;
            // visualize reference
            {
                visualization_msgs::msg::Marker marker;
                marker.color.r = 0xFF;
                marker.color.g = 0x00;
                marker.color.b = 0x00;
                marker.color.a = 1.0;
                marker.header.frame_id = costmap_ros_->getGlobalFrameID();
                marker.header.stamp = t;
                marker.ns = "reference path";
                marker.id = 2000;
                marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
                marker.action = visualization_msgs::msg::Marker::ADD;
                marker.pose.orientation.w = 1.0;
                marker.scale.x = 0.04;
                for (auto ref_point : reference_path)
                {
                    geometry_msgs::msg::Point point;
                    point.x = ref_point(0);
                    point.y = ref_point(1);
                    point.z = 0.06;
                    marker.points.push_back(point);
                }
                markers.markers.push_back(marker);
            }

            pub_ref_path_->publish(markers);
        }
    }

    void NominalCapsuleController::publishReferenceTrajectory(const rclcpp::Time &t)
    {
        if (nav2_plugin_params_.visualization.publish_reference_trajectory)
        {
            visualization_msgs::msg::MarkerArray markers;
            // visualize reference
            visualization_msgs::msg::Marker marker;
            marker.color.r = 0x00;
            marker.color.g = 0xFF;
            marker.color.b = 0xFF;
            marker.color.a = 1.0;
            marker.header.frame_id = costmap_ros_->getGlobalFrameID();
            marker.header.stamp = t;
            marker.ns = "reference traj";
            marker.id = 5000;
            marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
            marker.action = visualization_msgs::msg::Marker::ADD;
            marker.pose.orientation.w = 1.0;
            marker.scale.x = 0.04;
            // NOTE: reference trajectories rather than positions predicted by mpc
            for (int i_hrzn = 0; i_hrzn < nav2_plugin_params_.ocp.ocp_dims.n_hrzn; i_hrzn++)
            {
                Eigen::Matrix<double, 5, 1> robot_state;
                geometry_msgs::msg::Point point;
                point.x = yref_[i_hrzn * nav2_plugin_params_.ocp.ocp_dims.ny];
                point.y = yref_[i_hrzn * nav2_plugin_params_.ocp.ocp_dims.ny + 1];
                point.z = 0.06;
                marker.points.push_back(point);
            }
            {
                geometry_msgs::msg::Point point;
                point.x = yref_e_[0];
                point.y = yref_e_[1];
                point.z = 0.06;
                marker.points.push_back(point);
            }
            markers.markers.push_back(marker);

            pub_ref_traj_->publish(markers);
        }
    }

    void NominalCapsuleController::publishLocalPlan(const rclcpp::Time &t)
    {
        if (nav2_plugin_params_.visualization.publish_local_plan)
        {
            // create a path message
            nav_msgs::msg::Path local_plan;
            local_plan.poses.resize(nav2_plugin_params_.ocp.ocp_dims.n_hrzn);
            local_plan.header.frame_id = costmap_ros_->getGlobalFrameID();
            local_plan.header.stamp = t;
            Eigen::Vector<double, 3> pose_k;

            // Extract the plan in world co-ordinates, we assume the path is all in the same frame
            tf2::Quaternion pred_quat;
            for (int i_hrzn = 1; i_hrzn <= nav2_plugin_params_.ocp.ocp_dims.n_hrzn; ++i_hrzn)
            {
                ptr_controller_->poseSolAtTk(pose_k, i_hrzn);
                pred_quat.setRPY(0, 0, pose_k(2));

                local_plan.poses[i_hrzn - 1].header.frame_id = costmap_ros_->getGlobalFrameID();
                local_plan.poses[i_hrzn - 1].header.stamp = t;
                local_plan.poses[i_hrzn - 1].pose.position.x = pose_k(0);
                local_plan.poses[i_hrzn - 1].pose.position.y = pose_k(1);
                local_plan.poses[i_hrzn - 1].pose.position.z = 0;
                local_plan.poses[i_hrzn - 1].pose.orientation.x = pred_quat.x();
                local_plan.poses[i_hrzn - 1].pose.orientation.y = pred_quat.y();
                local_plan.poses[i_hrzn - 1].pose.orientation.z = pred_quat.z();
                local_plan.poses[i_hrzn - 1].pose.orientation.w = pred_quat.w();
            }

            pub_local_plan_->publish(local_plan);
        }
    }

    void NominalCapsuleController::publishTimings(const rclcpp::Time &t)
    {
        if (nav2_plugin_params_.visualization.publish_timings)
        {
            sipoc_ros2_interfaces::msg::SipocControllerTimings msg;
            msg.header.stamp = t;

            msg.compute_ref_traj = nlp_preparation_elapsed_time_;
            msg.solve_lower_level = ptr_controller_->lower_level_elapsed_time() * 1e-3;
            msg.solve_upper_level = ptr_controller_->upper_level_elapsed_time() * 1e-3;
            msg.total = total_elapsed_time_;

            pub_timings_->publish(msg);
        }
    }

    void NominalCapsuleController::publishObstacle(const rclcpp::Time &t)
    {
        if (nav2_plugin_params_.visualization.publish_obs_imposed_constr)
        {
            visualization_msgs::msg::Marker marker;
            marker.color.r = 1.0;
            marker.color.g = 0.;
            marker.color.b = 1.0;
            marker.color.a = 1.0;
            marker.header.frame_id = costmap_ros_->getGlobalFrameID();
            marker.header.stamp = t;
            marker.ns = "obs";
            marker.type = visualization_msgs::msg::Marker::POINTS;
            marker.action = visualization_msgs::msg::Marker::ADD;
            marker.pose.orientation.w = 1.0;
            marker.scale.x = 0.1;
            marker.scale.y = 0.1;
            marker.scale.z = 0.1;
            std::vector<Eigen::Vector2d> vec_obs;
            vec_obs.resize(nav2_plugin_params_.ocp.ocp_iters.max_constr_num);
            for (int i_hrzn = 1; i_hrzn <= nav2_plugin_params_.ocp.ocp_dims.n_hrzn; ++i_hrzn)
            {
                ptr_controller_->obsSolAtTk(i_hrzn, vec_obs);
                for (const Eigen::Vector2d &obs : vec_obs)
                {
                    if (obs.coeff(0) >= 1e3 || obs.coeff(1) >= 1e3)
                    {
                        break;
                    }
                    geometry_msgs::msg::Point point;
                    point.x = obs.coeff(0);
                    point.y = obs.coeff(1);
                    point.z = 0.06;
                    marker.points.push_back(point);
                }
            }
            pub_obs_->publish(marker);
        }
    }

    void NominalCapsuleController::publishVelPredictOCP(const rclcpp::Time &t, const manif::SE2Tangent<double> &robot_twist)
    {
        if (nav2_plugin_params_.visualization.publish_predicted_vel_values)
        {
            geometry_msgs::msg::TwistStamped msg;
            msg.header.stamp = t;
            msg.header.frame_id = "base_link";

            msg.twist.linear.x = robot_twist.x();
            msg.twist.linear.y = robot_twist.y();
            msg.twist.linear.z = 0.;
            msg.twist.angular.z = robot_twist.angle();

            pub_vel_values_->publish(msg);
        }
    }

}

// Register this controller as a nav2_core plugin
PLUGINLIB_EXPORT_CLASS(sipoc_mr_nav2_controller::NominalCapsuleController, nav2_core::Controller)
