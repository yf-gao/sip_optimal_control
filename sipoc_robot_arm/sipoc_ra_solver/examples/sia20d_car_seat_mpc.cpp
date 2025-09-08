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

#include "sipoc_ra_utils/sia20d_car_seat_velocity_profile_optimizer.hpp"
#include "sipoc_ra_solver/car_seat_trajectory_solver.hpp"
#include "sipoc_ra_utils/common_structs.hpp"
#include "sipoc_ra_utils/utils.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>

#include <manif/manif.h>
#include <eigen3/Eigen/Dense>

#include <vector>
#include <memory>
#include <iostream>
#include <filesystem>
#include <map>
#include <variant>
#include <chrono>
#include <iomanip>
#include <ctime>

using VecVariant = std::variant<std::vector<double>, std::vector<int>>;

void integrate_robot_joints(const Eigen::Vector<double, 8> &joint_accelerations, const sipoc_ra_utils::ConfigOCP &config_ocp, const Eigen::Vector<double, 16> &current_joint_state, Eigen::Vector<double, 16> &next_joint_state)
{
    next_joint_state.head<8>() = current_joint_state.head<8>() + current_joint_state.tail<8>() * config_ocp.delta_t + 0.5 * joint_accelerations * config_ocp.delta_t * config_ocp.delta_t;
    next_joint_state.tail<8>() = current_joint_state.tail<8>() + joint_accelerations * config_ocp.delta_t;
    return;
}

bool closed_loop_sim()
{
    sipoc_ra_utils::ConfigTrajectorySIPSolver config_sip_solver;
    config_sip_solver.ocp.nx = 16; // 8 joints + 8 velocities
    config_sip_solver.ocp.nu = 8;
    config_sip_solver.ocp.n_hrzn = 20;
    config_sip_solver.ocp.delta_t = 0.1; // NOTE: Align with the Python code. Not automatically checked.
    config_sip_solver.ocp.num_max_constr = 30;
    config_sip_solver.tol = 1e-5;
    config_sip_solver.num_iterations = 8;
    config_sip_solver.ocp.joint_vel_as_state = true;
    config_sip_solver.ocp.terminal_equality_constraints = false;
    config_sip_solver.use_cropped_octomap = true;
    std::cout << "delta_t: " << config_sip_solver.ocp.delta_t << std::endl;
    double seat_scale = 0.8;

    std::string package_name = "sipoc_ra_support";
    std::filesystem::path env_path = ament_index_cpp::get_package_share_directory(package_name);
    env_path += "/meshes/car_seat/car_whole";
    std::filesystem::path urdf_path = ament_index_cpp::get_package_share_directory(package_name);
    urdf_path += ("/urdf/sia20d_car_seat.urdf");
    std::shared_ptr<sipoc_ra_solver::CarSeatTrajectorySolver> solver = std::make_shared<sipoc_ra_solver::CarSeatTrajectorySolver>(config_sip_solver, urdf_path.string(), env_path.string(), seat_scale);

    sipoc_ra_utils::Sia20dCarSeatVelocityProfileOptimizer vel_opt;
    vel_opt.configureOptimizer(config_sip_solver.ocp.delta_t, config_sip_solver.ocp.n_hrzn);

    Eigen::Vector<double, 8> start_joints;
    start_joints << 2.22, -3.14, 0.5, 0, -1.45, 0, 0.39, -0.29;
    Eigen::Vector<double, 8> target_joints;
    target_joints << 4.06036, -0.0356958, 0.680113, 0.0250365, -1.18485, 0.0585558, 0.294533, -0.330305;

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
    vec_global_plan.push_back(target_joints);
    vel_opt.setRefPath(vec_global_plan);

    Eigen::Vector<double, 16> current_joint_state, next_joint_state;
    current_joint_state << start_joints, Eigen::Vector<double, 8>::Zero();
    std::cout << "Current Joint State: " << current_joint_state.transpose() << std::endl;
    Eigen::VectorXd u0_star(8);
    std::vector<Eigen::Vector<double, 16>> ref_trajectory;
    std::vector<Eigen::VectorXd> ref_trajectory_Xd;

    bool flag_reinitialize = true;
    unsigned int idx_sim = 0;
    unsigned int max_sim = 100;
    double thr = 1e-3;
    double still_thr = 1e-3;
    unsigned int num_step_still_termination = 5;
    unsigned int num_step_still = 0;
    double min_signed_distance = std::numeric_limits<double>::max();

    std::vector<double> vec_l_infinity2target, vec_min_distance, vec_computation_time, vec_coll_time, vec_ocp_time, vec_coll_coal_time, vec_coll_min_element_time, vec_coll_update_time;
    vec_l_infinity2target.reserve(max_sim);
    vec_min_distance.reserve(max_sim);
    vec_computation_time.reserve(max_sim);
    vec_coll_time.reserve(max_sim);
    vec_ocp_time.reserve(max_sim);
    vec_coll_coal_time.reserve(max_sim);
    vec_coll_min_element_time.reserve(max_sim);
    vec_coll_update_time.reserve(max_sim);
    std::vector<int> vec_num_iter;
    vec_num_iter.reserve(max_sim);

    while ((current_joint_state.head<8>() - target_joints).lpNorm<Eigen::Infinity>() > thr && idx_sim < max_sim)
    {
        std::cout << "Simulation step: " << idx_sim << std::endl;
        std::cout << "Current Joint State: " << current_joint_state.head<8>().transpose() << std::endl;

        solver->setOCPCurrentJointState(current_joint_state);

        vel_opt.computeReferenceTraj(current_joint_state);
        const std::vector<Eigen::Vector<double, 16>> &ref_trajectory = vel_opt.referenceTrajConstRef();
        ref_trajectory_Xd.clear();
        for (auto const &ref_state : ref_trajectory)
        {
            ref_trajectory_Xd.emplace_back(Eigen::VectorXd(ref_state));
        }
        solver->setOCPRefTrajectory(ref_trajectory_Xd, flag_reinitialize);
        sipoc_ra_utils::SolverStatus status = solver->solve();
        if (status == sipoc_ra_utils::SolverStatus::SUCCESS)
        {
            flag_reinitialize = false;
        }
        else
        {
            flag_reinitialize = true;
        }
        solver->getU0Star(u0_star);
        std::cout << "U0 Star: " << u0_star.transpose() << std::endl;
        integrate_robot_joints(u0_star, solver->getSolverConfig().ocp, current_joint_state, next_joint_state);
        // Sanity check
        std::vector<Eigen::VectorXd> vec_robot_state_sol = solver->getSolRobotJointsOverTime();
        if ((vec_robot_state_sol[1] - next_joint_state).lpNorm<Eigen::Infinity>() > 1e-6)
        {
            std::cerr << "Error: The first solution does not match the computed next joint state." << std::endl;
            return false;
        }

        // Check collision avoidance
        double temp_signed_distance = solver->evaluateConstraintSatisfaction(false);
        min_signed_distance = std::min(min_signed_distance, temp_signed_distance);

        // Terminate if the robot joints do not change for several steps
        if ((next_joint_state.head<8>() - current_joint_state.head<8>()).lpNorm<Eigen::Infinity>() < still_thr)
        {
            num_step_still++;
            if (num_step_still >= num_step_still_termination)
            {
                std::cout << "Robot joints do not change for several steps. Terminating simulation." << std::endl;
                break;
            }
        }
        else
        {
            num_step_still = 0;
        }

        current_joint_state = next_joint_state;
        double l_infinity2target = (current_joint_state.head<8>() - target_joints).lpNorm<Eigen::Infinity>();
        std::cout << "L-infinity distance to target: " << l_infinity2target << std::endl;
        sipoc_ra_utils::InfoIterSIPSolver info;
        solver->summarizeInfoIter(info);
        vec_l_infinity2target.push_back(l_infinity2target);
        vec_min_distance.push_back(temp_signed_distance);
        vec_computation_time.push_back(info.time_all_comps);
        vec_coll_time.push_back(info.time_collision_detection);
        vec_ocp_time.push_back(info.time_ocp_solver);
        vec_num_iter.push_back(info.num_iter);
        vec_coll_coal_time.push_back(info.time_detailed_collision_detection.time_coal);
        vec_coll_min_element_time.push_back(info.time_detailed_collision_detection.time_min_element);
        vec_coll_update_time.push_back(info.time_detailed_collision_detection.time_update);
        idx_sim++;

        if (!flag_reinitialize)
        {
            solver->shiftInputStateTrajectory();
            solver->gatherActiveConstraintsAndShiftOneStep();
        }
        else
        {
            solver->clearPCSubsets();
            // The x_sol will be initialized by the reference trajectory.
        }
    }
    std::cout << "Simulation finished after " << idx_sim << " steps. (" << idx_sim * config_sip_solver.ocp.delta_t << " seconds)" << std::endl;
    std::cout << "Final Joint State: " << current_joint_state.head<8>().transpose() << std::endl;

    std::map<std::string, VecVariant> results;
    results["l_infinity2target"] = vec_l_infinity2target;
    results["min_signed_distance"] = vec_min_distance;
    results["computation_time"] = vec_computation_time;
    results["num_iter"] = vec_num_iter;
    results["coll_time"] = vec_coll_time;
    results["ocp_time"] = vec_ocp_time;
    results["coll_coal_time"] = vec_coll_coal_time;
    results["coll_min_element_time"] = vec_coll_min_element_time;
    results["coll_update_time"] = vec_coll_update_time;
    sipoc_ra_utils::write_results_to_json(results, std::string("/sia20d_car_seat_mpc/"));
    return true;
}

int main()
{
    closed_loop_sim();
    return 0;
}
