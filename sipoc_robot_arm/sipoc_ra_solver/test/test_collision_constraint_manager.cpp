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

#include "sipoc_ra_solver/collision_constraint_manager.hpp"
#include "sipoc_ra_solver/pinocchio_kinematics.hpp"
#include "sipoc_ra_solver/coal_collision_detection.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <manif/manif.h>
#include <eigen3/Eigen/Dense>

#include <unordered_map>
#include <string>
#include <filesystem>

int main()
{
    std::string package_name = "sipoc_ra_support";
    std::filesystem::path env_path = ament_index_cpp::get_package_share_directory(package_name);
    env_path += "/meshes/car_seat/car_whole";
    std::filesystem::path urdf_path = ament_index_cpp::get_package_share_directory(package_name);
    urdf_path += "/urdf/sia20d_car_seat.urdf";
    std::cout << "Loading URDF from: " << urdf_path.string() << std::endl;
    std::filesystem::path mesh_path = ament_index_cpp::get_package_share_directory(package_name);
    mesh_path += "/meshes/car_seat/collision/seat_1.stl";
    std::string collision_element_name = "seat_1";
    sipoc_ra_utils::ConfigCollisionElementShape config_collision_element_shape;
    config_collision_element_shape.link_name = "seat";
    config_collision_element_shape.convex.mesh_path = mesh_path.string();
    config_collision_element_shape.convex.scale = 0.8;
    config_collision_element_shape.padding = 0.2f;
    std::map<std::string, sipoc_ra_utils::ConfigCollisionElementShape> map_link = {
        {collision_element_name, config_collision_element_shape}};
    sipoc_ra_solver::CollisionConstraintManager collision_constraint_manager(map_link, env_path.string(), 100, 1, 8, false);

    std::shared_ptr<sipoc_ra_solver::PinocchioKinematics> pinocchio_kin = std::make_shared<sipoc_ra_solver::PinocchioKinematics>(urdf_path.string());
    Eigen::VectorXd joints(8);
    joints << 2.99, -1.6, 0.55, 0.1, -1.3, -0.1, 0.5, -0.6;
    pinocchio_kin->computeForwardKinematics(joints);
    collision_constraint_manager.detectCollisionAndUpdatePCSubset(pinocchio_kin, joints, 0);
    std::vector<Eigen::VectorXd> vec_constr_coeffs;
    std::cout << "Call Constraint Linearization" << std::endl;
    std::vector<Eigen::Vector<double, 6>> vec_sep_planes;
    collision_constraint_manager.linearizeConstraints(joints, pinocchio_kin, 0, vec_constr_coeffs, vec_sep_planes);
    std::cout << "# End of the test. #" << std::endl;
    return 0;
}
