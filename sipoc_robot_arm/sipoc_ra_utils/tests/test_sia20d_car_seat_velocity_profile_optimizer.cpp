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

#include <eigen3/Eigen/Dense>
#include <string>
#include <memory>
#include <filesystem>
#include <iostream>

int main()
{
    sipoc_ra_utils::Sia20dCarSeatVelocityProfileOptimizer vel_opt;
    double ocp_delta_t = 0.1;
    unsigned int ocp_n_hrzn = 30;
    vel_opt.configureOptimizer(ocp_delta_t, ocp_n_hrzn);

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

    Eigen::Vector<double, 16> current_joint_state;
    current_joint_state << start_joints, Eigen::Vector<double, 8>::Zero();
    vel_opt.computeReferenceTraj(current_joint_state);
    const std::vector<Eigen::Vector<double, 16>> &ref_trajectory = vel_opt.referenceTrajConstRef();

    std::cout << "Reference trajectory:" << std::endl;
    for (const auto &state : ref_trajectory)
    {
        std::cout << "Joint angle: " << state.head<8>().transpose() << ",\nVelocity: " << state.tail<8>().transpose() << std::endl;
    }
    std::cout << "Test Completed." << std::endl;

    return 0;
}
