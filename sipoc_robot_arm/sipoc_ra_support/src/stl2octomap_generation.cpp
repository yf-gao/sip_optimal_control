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

#include <CGAL/Polygon_mesh_processing/IO/polygon_mesh_io.h>
#include <CGAL/Polygon_mesh_processing/distance.h>
#include <CGAL/Simple_cartesian.h>
#include <CGAL/Surface_mesh.h>
#include <CGAL/Side_of_triangle_mesh.h>
#include <octomap/octomap.h>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <iostream>
#include <string>
#include <filesystem>

int main(int argc, char *argv[])
{
    using Kernel = CGAL::Simple_cartesian<double>;

    std::string package_name = "sipoc_ra_support";
    std::filesystem::path fs_path = ament_index_cpp::get_package_share_directory(package_name);
    fs_path += "/meshes/car_seat/car_whole";
    std::string path = fs_path.string();
    double max_x = 0.;
    double min_x = 0.;
    bool flag_crop = false;

    for (int i = 1; i < argc; ++i)
    {
        if ((strcmp(argv[i], "--path") == 0))
        {
            ++i;
            path = argv[i];
        }
        else if (strcmp(argv[i], "--x-bb") == 0)
        {
            ++i;
            min_x = std::stod(argv[i]);
            ++i;
            max_x = std::stod(argv[i]);
            flag_crop = true;
        }
    }

    std::string bt_path;
    if (flag_crop)
    {
        bt_path = path + "_cropped.bt";
    }
    else
    {
        bt_path = path + ".bt";
    }
    std::string stl_path = path + ".stl";
    std::cout << "Using stl_path: " << stl_path << std::endl;

    CGAL::Surface_mesh<Kernel::Point_3> mesh;
    if (!CGAL::Polygon_mesh_processing::IO::read_polygon_mesh(stl_path, mesh))
    {
        throw std::runtime_error("Failed to read mesh from " + stl_path);
    }
    CGAL::Side_of_triangle_mesh<CGAL::Surface_mesh<Kernel::Point_3>, Kernel> tester(mesh);

    std::vector<Kernel::Point_3> mesh_samples;
    CGAL::Polygon_mesh_processing::sample_triangle_mesh(mesh, std::back_inserter(mesh_samples), CGAL::parameters::use_grid_sampling(true).grid_spacing(0.03));

    // Save mesh samples for visualization
    std::cout << "Saving output..." << std::endl;
    std::ofstream ofstream;
    ofstream.open("mesh_samples.obj", std::ios::out);
    for (auto point : mesh_samples)
    {
        ofstream << "v " << point[0] << " " << point[1] << " " << point[2] << std::endl;
    }
    ofstream.close();

    octomap::Pointcloud octomap_cloud;
    for (const auto &point : mesh_samples)
    {
        if (flag_crop && (point.x() < min_x || point.x() > max_x))
        {
            continue; // Skip points outside the x-bounding box
        }
        octomap_cloud.push_back(point.x(), point.y(), point.z());
    }
    octomap::OcTree ot(0.015);
    octomap::point3d sensor_origin(0, 0, 0);
    ot.insertPointCloud(octomap_cloud, sensor_origin);
    ot.writeBinary(bt_path);

    double octomap_resolution = ot.getResolution();
    std::cout << "Octree resolution: " << octomap_resolution << std::endl;

    return 0;
}
