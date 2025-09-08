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

#ifndef SIPOC_MR_UTILS__COMMON_STRUCTS_HPP_
#define SIPOC_MR_UTILS__COMMON_STRUCTS_HPP_

#include <Eigen/Dense>

namespace sipoc_mr_utils
{
    template <typename T>
    struct DistanceMinimizerSolution
    {
        Eigen::Vector<T, 2> delta_p_ellipsoid;
        Eigen::Vector<T, 2> gamma_polygon;
        T gamma_line;
        T delta_theta;
        T squared_distance;
    };

    template <typename T>
    struct DistanceMinimizerParameter
    {
        Eigen::Matrix<T, 2, 2> shape_matrix_ellipsoid;
        Eigen::Vector<T, 2> p_center_ellipsoid;
        T heading_line;
        T scaled_ex;
    };
    struct SDFParam
    {
        int map_height; // py
        int map_width;  // px
        double grid_size;
        double lb_px_grid;
        double lb_py_grid;
        unsigned char map_occ_threshold;
    };
    template <typename T>
    struct RobotParam
    {
        int num_gamma_grids;
        T robot_capsule_half_length;
        T robot_capsule_radius;
        T robot_polygon_dilation_radius;
        std::vector<T> vertices_flattened;
    };
    struct OCPDimensions
    {
        unsigned int nx = 5;
        unsigned int nu = 2;
        unsigned int nbu = 2;
        unsigned int nbx = 2;
        unsigned int nbxe = 2;
        unsigned int ny = 7;
        unsigned int nyn = 5;
        int n_hrzn = 30;
    };
    struct AdditiveDisturbance
    {
        // double linear_vel_cov = 1e-5;
        // double angular_vel_cov = 4e-5;
        // double linear_fdbk = -1.0;
        // double angular_fdbk = -1.0;
        double robust_scale = 1.0;
    };
    template <typename T>
    struct MinimizerResult
    {
        T minimizer[2] = {0., 0.};
        T min_value;
    };
    struct OCPParameters
    {
        struct Bounds
        {
            double min_forward_velocity = 0.0;
            double max_forward_velocity = 3.0;
            double max_angular_velocity = 1.0;
            double min_forward_acceleration = -6.0;
            double max_forward_acceleration = 3.0;
            double max_angular_acceleration = 1.0;
            double max_forward_velocity_e = 0.2;
            double max_angular_velocity_e = 0.2;
        } ocp_bounds;
        struct Weights
        {
            double position = 1e-1;
            double heading = 1e-3;
            double forward_velocity = 1e-1;
            double angular_velocity = 1e-5;
            double forward_acceleration = 1e-3;
            double angular_acceleration = 1e-3;
            // NOTE: angular_acceleration = 2e-3 for capsule_disturbed
            double slack_l1_cost = 1e3;
            double slack_l2_cost = 1e3;
        } ocp_weights;
        struct Iterations
        {
            unsigned int max_iter_num = 100;
            unsigned int max_constr_num = 25;
            // NOTE: max_constr_num = 35 for capsule_disturbed
            double eps_converg = 1e-6;
        } ocp_iters;
        // struct OCPDimensions ocp_dims{.nx=9, .nu=2, .nbu=2, .nbx=2, .nbxe=2, .ny=7, .nyn=5, .n_hrzn=30};
        struct OCPDimensions ocp_dims{
            9, 2, 2, 2, 2, 7, 5, 30};
        // struct AdditiveDisturbance disturbance{.robust_scale=1.0};
        struct AdditiveDisturbance disturbance{
            1.0};
        // struct RobotParam<double> robot{.num_gamma_grids = 110, .robot_capsule_half_length=0.5, .robot_capsule_radius=0.45, .robot_polygon_dilation_radius=0.2, .vertices_flattened={}};
        struct RobotParam<double> robot{
            110, 0.5, 0.45, 0.2, {}
            // NOTE: num_gamma_grids = 50 for capsule
        };
        // struct SDFParam sdf{.map_height = 160, .map_width = 640, .grid_size = 0.02, .lb_px_grid = -7.9875, .lb_py_grid = -1.9875, .map_occ_threshold=90};
        struct SDFParam sdf{
            160, 640, 0.02, -7.9875, -1.9875, 90};
        double delta_t_if_shooting_nodes_is_empty = 0.2;
        std::vector<double> shooting_nodes;
        double acados_inf = 1e3;
        bool ifBilinearInterpArgMinGamma = true;
    };
    struct LocalPlannerParameters
    {
        struct Nav
        {
            double twist_goal_tolerance = 0.05;
            double lookahead_dist_goal_point = 3.0;
            double path_following_tail_length = 0.5;
            double path_following_reference_length = 3.5;
            double path_following_min_step = 0.05;
        } nav;
        struct VelOpt
        {
            double mpc_factor = 0.98;
            double max_lat_acc = 1.0;
            double max_ang_vel = 0.7;
            double track_width = 0.507;
            double min_wheel_vel = -3.0;
            double max_wheel_vel = 3.0;
            double max_wheel_acc = 2.0;
            double max_wheel_dec = -2.0;
        } velopt;
        struct Delay
        {
            double dt = 0.01;
            double tau_v = 0.2;
            double tau_w = 0.2;
        } delay;
        double controller_frequency = 10;
    };
    struct Nav2PluginParameters
    {
        struct Visualization
        {
            int npoints_reference_path = 20;
            bool publish_reference_path = true;
            bool publish_reference_trajectory = true;
            bool publish_local_plan = true;
            bool publish_timings = true;
            bool publish_obs_imposed_constr = true;
            bool publish_predicted_vel_values = true;
            bool publish_ustar_values = true;
        } visualization;
        struct Log
        {
            bool robot_current_state = true;
            bool nlp_sol_status = true;
        } log;
        struct OCPParameters ocp;
        struct LocalPlannerParameters local_planner;
        bool if_subscribe_laser_scan = false;
    };
    enum SIPSolverStatus
    {
        success,
        acadosFail,
        reachMaxIters
    };
}

#endif // SIPOC_MR_UTILS__COMMON_STRUCTS_HPP_
