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

#include "sipoc_ra_solver/pinocchio_kinematics.hpp"
#include "sipoc_ra_utils/utils.hpp"

#include <pinocchio/multibody/data.hpp>
#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/math/rpy.hpp>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <manif/manif.h>
#include <eigen3/Eigen/Dense>
#include <string>
#include <stdlib.h>
#include <time.h>
#include <filesystem>

using namespace sipoc_ra_solver;

int test_jacobian_computation(void)
{
    srand((unsigned)time(NULL));
    int num_tests = 1000;

    std::string package_name = "sipoc_ra_support";
    std::filesystem::path urdf_path = ament_index_cpp::get_package_share_directory(package_name);
    urdf_path += "/urdf/sia20d_car_seat.urdf";
    std::string link_name = "joint_t";
    std::shared_ptr<sipoc_ra_solver::PinocchioKinematics> solver = std::make_shared<sipoc_ra_solver::PinocchioKinematics>(urdf_path.string());
    manif::SE3<double> robot_pose, robot_pose2;
    Eigen::VectorXd joints;
    Eigen::VectorXd delta_robot_joints;
    delta_robot_joints.resize(solver->getModelNv());
    double delta = 1e-5;
    double max_diff = 0.;

    manif::SE3<double> tmp_se3;
    Eigen::MatrixXd J_computed(3, solver->getModelNv());
    Eigen::MatrixXd J_finite_diff(3, solver->getModelNv());

    Eigen::Vector3d old_position, new_position_by_fk;

    for (int j = 0; j < num_tests; ++j)
    {
        J_computed.setZero();
        J_finite_diff.setZero();

        solver->getRandomConfiguration(joints);
        solver->computeForwardKinematics(joints);
        solver->getFrameSE3(link_name, robot_pose);

        Eigen::Vector3d p_on_robot;
        for (unsigned int i = 0; i < 3; ++i)
        {
            p_on_robot(i) = 0.2 * (static_cast<double>(rand()) / static_cast<double>(RAND_MAX) - 0.5);
        }
        tmp_se3.quat(Eigen::Quaterniond::Identity());
        tmp_se3.translation(p_on_robot);
        solver->computeJacobians(joints, link_name, tmp_se3, J_computed);

        for (unsigned int i = 0; i < solver->getModelNv(); ++i)
        {
            delta_robot_joints.setZero();
            delta_robot_joints(i) = delta;

            solver->computeForwardKinematics(joints + delta_robot_joints);
            solver->getFrameSE3(link_name, robot_pose2);
            old_position = robot_pose.translation() + robot_pose.rotation() * p_on_robot;
            new_position_by_fk = robot_pose2.translation() + robot_pose2.rotation() * p_on_robot;
            J_finite_diff.col(i) = (new_position_by_fk - old_position) / delta;
        }
        double diff = (J_computed - J_finite_diff).array().abs().maxCoeff();
        if (diff > max_diff)
        {
            max_diff = diff;
            std::cout << "Test: " << j << " Max difference: " << max_diff << std::endl;
            std::cout << "J_computed: \n"
                      << J_computed << std::endl;
            std::cout << "J_finite_diff: \n"
                      << J_finite_diff << std::endl;
        }
    }
    std::cout << "Max difference after " << num_tests << " tests: " << max_diff << std::endl;
    return 0;
}


int main()
{
    test_jacobian_computation();
    return 0;
}
