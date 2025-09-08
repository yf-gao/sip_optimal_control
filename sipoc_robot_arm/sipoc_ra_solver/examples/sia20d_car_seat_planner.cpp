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
#include <sstream>

using VecVariant = std::variant<std::vector<double>, std::vector<int>>;
constexpr unsigned int num_grid_coll_eval = 100;

// joint names=["carriage_rail", "joint_s", "joint_l", "joint_e", "joint_u", "joint_r", "joint_b", "joint_t"]
sipoc_ra_utils::SolverStatus open_loop_trajectory_planning(const manif::SE3<double> &se3_target_seat, bool verbose, std::vector<double> &results_signed_distance_over_time, sipoc_ra_utils::InfoIterSIPSolver &info)
{
    // Configuration for the SIP solver
    sipoc_ra_utils::ConfigTrajectorySIPSolver config_sip_solver;
    config_sip_solver.ocp.nx = 8;
    config_sip_solver.ocp.nu = 8;
    config_sip_solver.ocp.n_hrzn = 10;
    config_sip_solver.ocp.delta_t = 0.3; // Does not really matter
    config_sip_solver.ocp.num_max_constr = 30;
    config_sip_solver.tol = 1e-6;
    config_sip_solver.ocp.joint_vel_as_state = false;
    config_sip_solver.ocp.terminal_equality_constraints = true;
    config_sip_solver.ocp.weight_joint_angle = 1e-2;
    config_sip_solver.ocp.weight_joint_vel = 1.0;
    config_sip_solver.ocp.weight_joint_angle_terminal = 1.0;
    double seat_scale = 0.8;

    std::string package_name = "sipoc_ra_support";
    std::filesystem::path env_path = ament_index_cpp::get_package_share_directory(package_name);
    env_path += "/meshes/car_seat/car_whole";
    std::filesystem::path urdf_path = ament_index_cpp::get_package_share_directory(package_name);
    urdf_path += ("/urdf/sia20d_car_seat.urdf");
    std::shared_ptr<sipoc_ra_solver::CarSeatTrajectorySolver> solver = std::make_shared<sipoc_ra_solver::CarSeatTrajectorySolver>(config_sip_solver, urdf_path.string(), env_path.string(), seat_scale);

    // Compute Target Joint Configuration
    Eigen::Vector<double, 8> target_joints_init, target_joints;
    target_joints_init << 4.05466, -0.0481182, 0.701223, 0.0189056, -1.2813, 0.0580309, 0.537218, -0.325783;
    auto pinocchio_kin = solver->getPinocchioKinematicsSharedPtr();
    manif::SE3<double> tmp_se3_seat, tmp_se3_joint_t;
    pinocchio_kin->computeForwardKinematics(Eigen::VectorXd::Zero(8));
    pinocchio_kin->getFrameSE3("seat", tmp_se3_seat);
    pinocchio_kin->getFrameSE3("joint_t", tmp_se3_joint_t);
    bool success = pinocchio_kin->solveInverseKinematics(se3_target_seat * tmp_se3_seat.inverse() * tmp_se3_joint_t, target_joints_init, "joint_t", target_joints);
    if (!success)
    {
        std::cerr << "Inverse kinematics failed for target SE3: " << se3_target_seat.translation().transpose() << std::endl;
        return sipoc_ra_utils::SolverStatus::SKIPPED_INV_KIN_FAILED;
    }
    if (verbose)
    {
        std::cout << "Target joint state: " << target_joints.transpose() << std::endl;
    }
    double sd_target = solver->evaluateMinimumSignedDistance(target_joints);
    if (sd_target < 0.0)
    {
        std::cerr << "Target joint state is in collision with the environment. Signed distance: " << sd_target << std::endl;
        return sipoc_ra_utils::SolverStatus::SKIPPED_TARGET_COLLISION;
    }

    // Reference Trajectory by linear interpolation
    Eigen::Vector<double, 8> start_joints;
    start_joints << 2.22, -3.14, 0.5, 0, -1.45, 0, 0.39, -0.29;
    std::vector<Eigen::VectorXd> ref_trajectory, vec_trajectory_init;
    for (unsigned int i = 0; i <= config_sip_solver.ocp.n_hrzn; ++i)
    {
        ref_trajectory.emplace_back(Eigen::VectorXd(target_joints));
        vec_trajectory_init.emplace_back(Eigen::VectorXd(start_joints + (target_joints - start_joints) * static_cast<double>(i) / static_cast<double>(config_sip_solver.ocp.n_hrzn)));
    }

    solver->setOCPCurrentJointState(start_joints);
    solver->setOCPTerminalJointState(target_joints);
    solver->setOCPRefTrajectory(ref_trajectory, false);
    solver->initializeSolver(vec_trajectory_init);
    sipoc_ra_utils::SolverStatus status = solver->solve();
    if (status == sipoc_ra_utils::SolverStatus::SUCCESS)
    {
        std::cout << "Open-loop trajectory planning succeeded." << std::endl;
    }
    else
    {
        std::cerr << "Open-loop trajectory planning failed." << std::endl;
    }

    // Check collision avoidance at finer resolution
    if (!solver->checkSlackVariableNonZero(1e-5))
    {
        std::cout << "Slack variables are not non-positive." << std::endl;
        status = sipoc_ra_utils::SolverStatus::SUCCESS_SLACK_NONZERO;
    }
    std::vector<Eigen::VectorXd> vec_joint_state_sol = solver->getSolRobotJointsOverTime();
    results_signed_distance_over_time.resize(num_grid_coll_eval);
    double delta_t_finer_resolution = config_sip_solver.ocp.delta_t * config_sip_solver.ocp.n_hrzn / static_cast<double>(num_grid_coll_eval);
    for (unsigned int i = 0; i < num_grid_coll_eval; ++i)
    {
        double t = i * delta_t_finer_resolution + 1e-8; // Avoid rounding errors
        int idx_hrzn = static_cast<int>(std::floor(t / config_sip_solver.ocp.delta_t));
        double gamma = (t - idx_hrzn * config_sip_solver.ocp.delta_t) / config_sip_solver.ocp.delta_t;
        Eigen::VectorXd joint_tmp = vec_joint_state_sol[idx_hrzn] * (1.0 - gamma) + vec_joint_state_sol[idx_hrzn + 1] * gamma;
        results_signed_distance_over_time[i] = solver->evaluateMinimumSignedDistance(joint_tmp);
    }
    double min_signed_distance_over_grids = solver->evaluateConstraintSatisfaction(true);
    std::cout << "min_signed_distance_over_grids = " << min_signed_distance_over_grids << std::endl;

    // Test whether the target joint state is reached
    double l_infinity_norm = (vec_joint_state_sol.back() - target_joints).lpNorm<Eigen::Infinity>();
    std::cout << "L-infinity norm between target and final joint states: " << l_infinity_norm << std::endl;
    if (status == sipoc_ra_utils::SolverStatus::SUCCESS && l_infinity_norm > 1e-4)
    {
        throw std::runtime_error("The final joint state does not reach the target joint state.");
    }

    solver->summarizeInfoIter(info);
    if (verbose)
    {
        // Timing
        std::cout << "Computation time: " << info.time_all_comps << " seconds" << std::endl;
        std::cout << "OCP solver time: " << info.time_ocp_solver << " seconds" << std::endl;
        std::cout << "Collision detection time: " << info.time_collision_detection << " seconds" << std::endl;
        std::cout << "Detailed (coal): " << info.time_detailed_collision_detection.time_coal << " seconds" << std::endl;
        std::cout << "Detailed (min element): " << info.time_detailed_collision_detection.time_min_element << " seconds" << std::endl;
        std::cout << "Detailed (update): " << info.time_detailed_collision_detection.time_update << " seconds" << std::endl;

        // Print solution. Will be used to generate the reference trajectory for mpc.
        std::vector<Eigen::VectorXd> vec_joint_state_sol = solver->getSolRobotJointsOverTime();
        for (unsigned int i = 0; i <= config_sip_solver.ocp.n_hrzn; ++i)
        {
            std::cout << "Solution at step " << i << ": " << std::endl;
            for (unsigned int j = 0; j < solver->getNumJoints(); ++j)
            {
                std::cout << vec_joint_state_sol[i](j) << ", ";
            }
            std::cout << std::endl;
        }
    }
    return status;
}

int main()
{
    unsigned int idx_test = 0;
    manif::SE3<double> se3_target_seat(-1.2, 0.15, 0.43, 0., 0., 0.);
    std::vector<double> vec_computation_time;
    std::vector<double> vec_nlp_time;
    std::vector<double> vec_coll_time;
    std::vector<double> vec_constr_lin_time;
    std::vector<std::vector<double>> vec_signed_distance_over_time;
    std::vector<int> vec_num_iter;
    std::vector<int> vec_solver_status;
    vec_signed_distance_over_time.resize(num_grid_coll_eval);

    for (double dx = -0.04; dx <= 0.2; dx += 0.02)
    {
        for (double dy = 0.; dy <= 0.04; dy += 0.01)
        {
            for (double dz = -0.02; dz <= 0.02; dz += 0.01)
            {
                std::vector<double> results_signed_distance_over_time;
                sipoc_ra_utils::InfoIterSIPSolver info;
                manif::SE3<double> se3_target_seat_perturbed = se3_target_seat * manif::SE3<double>(dx, dy, dz, 0., 0., 0.);
                sipoc_ra_utils::SolverStatus status = open_loop_trajectory_planning(se3_target_seat_perturbed, idx_test <= 30, results_signed_distance_over_time, info);
                if (status != sipoc_ra_utils::SolverStatus::SKIPPED_TARGET_COLLISION && status != sipoc_ra_utils::SolverStatus::SKIPPED_INV_KIN_FAILED)
                {
                    std::cout << "Test " << idx_test << ": Perturbed target SE3: " << se3_target_seat_perturbed.translation().transpose() << std::endl;
                    idx_test++;
                    vec_num_iter.push_back(info.num_iter);
                    vec_solver_status.push_back(static_cast<int>(status));

                    // Solve once again and take the minimum
                    sipoc_ra_utils::InfoIterSIPSolver info2;
                    open_loop_trajectory_planning(se3_target_seat_perturbed, idx_test == 0, results_signed_distance_over_time, info2);
                    vec_computation_time.push_back(std::min(info.time_all_comps, info2.time_all_comps));
                    vec_nlp_time.push_back(std::min(info.time_ocp_solver, info2.time_ocp_solver));
                    vec_coll_time.push_back(std::min(info.time_collision_detection, info2.time_collision_detection));
                    vec_constr_lin_time.push_back(std::min(info.time_constr_linearization, info2.time_constr_linearization));
                    if (info.num_iter != info2.num_iter)
                    {
                        throw std::runtime_error("Number of iterations are not the same between two runs.");
                    }
                    for (unsigned int i = 0; i < num_grid_coll_eval; ++i)
                    {
                        vec_signed_distance_over_time[i].push_back(results_signed_distance_over_time[i]);
                    }
                }
            }
        }
    }
    std::cout << "Total number of tests: " << idx_test << std::endl;
    std::map<std::string, VecVariant> results;
    results["computation_time"] = vec_computation_time;
    results["nlp_time"] = vec_nlp_time;
    results["constr_lin_time"] = vec_constr_lin_time;
    results["coll_time"] = vec_coll_time;
    results["num_iter"] = vec_num_iter;
    results["solver_status"] = vec_solver_status;

    for (unsigned int i = 0; i < num_grid_coll_eval; ++i)
    {
        std::ostringstream oss;
        oss << std::setw(3) << std::setfill('0') << i;
        results["signed_distance_over_time_" + oss.str()] = vec_signed_distance_over_time[i];
    }
    sipoc_ra_utils::write_results_to_json(results, std::string("/sia20d_car_seat_planner/"));
    return 0;
}
