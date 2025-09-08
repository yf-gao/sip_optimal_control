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

#include "sipoc_ra_solver/openvdb_signed_distance_field.hpp"

#include <eigen3/Eigen/Dense>
#include <ament_index_cpp/get_package_share_directory.hpp>

#include <memory>
#include <iostream>
#include <filesystem>

int main()
{
    std::string package_name = "sipoc_ra_support";
    std::filesystem::path path = ament_index_cpp::get_package_share_directory(package_name);
    path += "/meshes/car_seat/car_whole";
    std::shared_ptr<sipoc_ra_solver::OpenVDBSignedDistanceField> sdf_ptr_ = std::make_shared<sipoc_ra_solver::OpenVDBSignedDistanceField>(path.string(), 0.01);
    Eigen::Vector3d point(0.0, 0.0, 0.0);
    Eigen::Vector3d normal;
    sdf_ptr_->ComputeNormal(point, normal);
}
