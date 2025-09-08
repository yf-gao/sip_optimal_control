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

#ifndef SIPOC_MR_NAV2_CONTROLLER__ROBUST_DILATED_POLYGON_HPP_
#define SIPOC_MR_NAV2_CONTROLLER__ROBUST_DILATED_POLYGON_HPP_

#include "sipoc_mr_solver/dilated_polygon_robust_controller.hpp"
#include "sipoc_mr_utils/spline_wrapper.hpp"
#include "sipoc_mr_utils/state_predictor_filter.hpp"
#include "sipoc_mr_utils/velocity_profile_optimizer.hpp"
#include "sipoc_ros2_interfaces/msg/sipoc_controller_timings.hpp"

#include <cstdio>
extern "C"
{
// general acados
#include "acados/ocp_nlp/ocp_nlp_constraints_bgh.h"
#include "acados/ocp_nlp/ocp_nlp_cost_ls.h"
#include "acados/utils/math.h"
#include "acados/utils/print.h"
#include "acados/utils/types.h"
#include "acados_c/external_function_interface.h"
#include "acados_c/ocp_nlp_interface.h"
}

#include "nav2_core/controller.hpp"
#include "pluginlib/class_loader.hpp"
#include "pluginlib/class_list_macros.hpp"

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "geometry_msgs/msg/accel_stamped.hpp"
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <string>
#include <vector>
#include <memory>
#include <boost/circular_buffer.hpp>

namespace sipoc_mr_nav2_controller
{
    class RobustDilatedPolygonController : public nav2_core::Controller
    {
        static const uint32_t velopt_path_N{100};

    public:
        using WorldPoint = Eigen::Matrix<double, 2, 1>;
        using ReferencePath = std::vector<WorldPoint, Eigen::aligned_allocator<WorldPoint>>;
        using ReferenceSpline = sipoc_mr_utils::SplineWrapper;

        RobustDilatedPolygonController() = default;
        ~RobustDilatedPolygonController() override = default;

        void configure(
            const rclcpp_lifecycle::LifecycleNode::WeakPtr &parent,
            std::string name, std::shared_ptr<tf2_ros::Buffer> tf,
            std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros);

        void cleanup() override;
        void activate() override;
        void deactivate() override;

        geometry_msgs::msg::TwistStamped computeVelocityCommands(
            const geometry_msgs::msg::PoseStamped &pose,
            const geometry_msgs::msg::Twist &velocity,
            nav2_core::GoalChecker * /*goal_checker*/) override;

        void setPlan(const nav_msgs::msg::Path &path) override;
        void setSpeedLimit(const double &speed_limit, const bool &percentage) override;

    protected:
        rclcpp_lifecycle::LifecycleNode::WeakPtr node_;
        std::shared_ptr<tf2_ros::Buffer> tf_;
        std::string plugin_name_;
        std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
        rclcpp::Logger logger_{rclcpp::get_logger("AcadosController")};
        rclcpp::Clock::SharedPtr clock_;

        nav_msgs::msg::Path global_plan_;
        std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<visualization_msgs::msg::MarkerArray>> pub_ref_path_;
        std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<visualization_msgs::msg::MarkerArray>> pub_ref_traj_;
        std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<visualization_msgs::msg::Marker>> pub_obs_;
        std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::Path>> pub_local_plan_;
        std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<sipoc_ros2_interfaces::msg::SipocControllerTimings>> pub_timings_;
        std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::TwistStamped>> pub_vel_values_;
        std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::AccelStamped>> pub_ustar_values_;

        // Delay Compensator
        rclcpp::Time t_prev_;

        // Velocity Optimizer
        sipoc_mr_utils::VelocityProfileOptimizer velocity_optimizer_;
        double *yref_;
        double *yref_e_;

        // Elapsed time
        double nlp_preparation_elapsed_time_ = 0.;
        double total_elapsed_time_ = 0.;
        Eigen::Matrix<double, 2, 1> cmd_vel_;

        // Occupancy map
        bool occ_map_updated_ = false;
        rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr sub_occ_map_;
        rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr sub_laser_scan_;

        // Controller
        std::shared_ptr<sipoc_mr_solver::DilatedPolygonRobustController> ptr_controller_;

        // Kalman filter
        std::shared_ptr<sipoc_mr_utils::StatePredictorFilter> ptr_state_predictor_filter_;
        boost::circular_buffer<Eigen::Vector2d> cb_acc_hist_;
        boost::circular_buffer<Eigen::Vector2d> cb_cmd_vel_hist_;

        // Parameters
        sipoc_mr_utils::Nav2PluginParameters nav2_plugin_params_;
        rclcpp::Duration transform_tolerance_{0, 0};

        void declareAndGetParams();
        void OccupancyMapCallback(nav_msgs::msg::OccupancyGrid msg);
        void FilteredLaserCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg);
        bool buildReferencePath(const nav_msgs::msg::Path &global_plan,
                                const manif::SE2<double> &current_pose,
                                ReferencePath &reference_path);
        void computeReferenceTraj(const manif::SE2<double> &current_pose,
                                  const manif::SE2Tangent<double> &current_vel,
                                  ReferencePath &reference_path);
        void resetFilterBuffers();

        ///< Publish the reference spline path
        void publishReferencePath(const rclcpp::Time &t);
        ///< Publish the reference trajectory (obtained by velocity_optimizer)
        void publishReferenceTrajectory(const rclcpp::Time &t);
        ///< Publish local plan
        void publishLocalPlan(const rclcpp::Time &t);
        ///< Publish timings of the planning algorithm
        void publishTimings(const rclcpp::Time &t);
        ///< Publish obstacles in the imposed constraints
        void publishObstacle(const rclcpp::Time &t);
        ///< Publish predictied velocities that the OCP takes as the current velocity in OCP
        void publishVelPredictOCP(const rclcpp::Time &t, const manif::SE2Tangent<double> &robot_twist);
        ///< Publish the u^* at k=0
        void publishUStar(const rclcpp::Time &t, const Eigen::Vector<double, 2> &u0_star);
    };

} // namespace sipoc_mr_nav2_controller

#endif // SIPOC_MR_NAV2_CONTROLLER__ROBUST_DILATED_POLYGON_HPP_
