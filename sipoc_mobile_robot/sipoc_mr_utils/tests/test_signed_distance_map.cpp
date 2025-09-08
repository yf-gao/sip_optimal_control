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

#include "sipoc_mr_utils/signed_distance_map.hpp"
#include "sipoc_mr_utils/common_structs.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <manif/SE2.h>
#include <Eigen/Dense>

#include <iostream>
#include <vector>
#include <string>
#include <filesystem>

int main()
{
    std::vector<double> polygon_vertices = {-0.18, -0.11, 0.45, -0.11, 0.45, 0.11, -0.18, 0.11, -0.33, 0.};
    sipoc_mr_utils::SignedDistanceMap obj(polygon_vertices);

    std::string package_name = "sipoc_mr_support";
    std::filesystem::path map_csv_path = ament_index_cpp::get_package_share_directory(package_name);
    map_csv_path += "/maps/walkway_occupancy_map.csv";
    std::cout << map_csv_path.string() << std::endl;
    sipoc_mr_utils::SDFParam sdf_param;
    sdf_param.lb_px_grid = -7.99;
    sdf_param.lb_py_grid = -1.99;
    sdf_param.grid_size = 0.02;
    sdf_param.map_height = 200;
    sdf_param.map_width = 800;
    sdf_param.map_occ_threshold = 90;
    obj.loadOccupancyMap(map_csv_path.string(), sdf_param);
    obj.computeSignedDistanceMap();
    manif::SE2<double> current_pose(0., 0., -0.1);
    sipoc_mr_utils::MinimizerResult<double> result = {};
    obj.argMinDistanceOverGamma(current_pose, true, result);
    std::cout << "min_value = " << result.min_value << ", minimizer[0]=" << result.minimizer[0] << ", minimizer[1]=" << result.minimizer[1] << std::endl;

    Eigen::Vector2d obs_coords;
    obj.getClosestObsAtGamma(current_pose, 0.5, obs_coords);
    std::cout << "obs_coords:" << obs_coords << std::endl;

    Eigen::Vector2d robot_coords, weights;
    Eigen::Vector<int, 2> grid_indices;
    robot_coords << 0.472, 0.1054;
    obj.world2grid_indices_floor(robot_coords, grid_indices, weights);
    std::cout << "grid_indices=" << grid_indices << ", weights=" << weights << std::endl;

    std::cout << "change map shape" << std::endl;
    map_csv_path = ament_index_cpp::get_package_share_directory(package_name);
    map_csv_path += "/maps/L_corridor_occupancy_map.csv";
    sdf_param.lb_px_grid = -2.99;
    sdf_param.lb_py_grid = -2.99;
    sdf_param.grid_size = 0.02;
    sdf_param.map_height = 300;
    sdf_param.map_width = 300;
    obj.loadOccupancyMap(map_csv_path.string(), sdf_param);
    obj.computeSignedDistanceMap();
    obj.argMinDistanceOverGamma(current_pose, true, result);
    obj.getClosestObsAtGamma(current_pose, 0.5, obs_coords);
    obj.world2grid_indices_floor(robot_coords, grid_indices, weights);
    std::cout << "min_value = " << result.min_value << ", minimizer[0]=" << result.minimizer[0] << ", minimizer[1]=" << result.minimizer[1] << std::endl;
    std::cout << "Test completed successfully!" << std::endl;
    return 0;
}
