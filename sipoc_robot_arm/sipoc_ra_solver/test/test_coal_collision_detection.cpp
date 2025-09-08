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

#include "sipoc_ra_solver/coal_collision_detection.hpp"
#include <coal/octree.h>
#include <coal/collision.h>
#include <coal/collision_object.h>
#include <coal/collision_data.h>
#include <coal/shape/geometric_shapes.h>

#include <octomap/octomap.h>
#include <manif/manif.h>
#include <eigen3/Eigen/Dense>
#include <ament_index_cpp/get_package_share_directory.hpp>

#include <atomic>
#include <execution>
#include <vector>
#include <string>
#include <cstdlib>
#include <memory>
#include <iostream>
#include <chrono>
#include <filesystem>

Eigen::Vector3d getrandomOccupiedNode(std::shared_ptr<const octomap::OcTree> ptr_octree)
{
    unsigned int num_occupied_nodes = 0;
    for (octomap::OcTree::leaf_iterator it = ptr_octree->begin_leafs(), end = ptr_octree->end_leafs(); it != end; ++it)
    {
        if (it->getOccupancy() > 0.5)
        {
            num_occupied_nodes++;
        }
    }
    if (num_occupied_nodes == 0)
    {
        throw std::runtime_error("No occupied nodes found in the octree.");
    }
    unsigned int random_index = rand() % num_occupied_nodes;
    unsigned int current_index = 0;
    for (octomap::OcTree::leaf_iterator it = ptr_octree->begin_leafs(), end = ptr_octree->end_leafs(); it != end; ++it)
    {
        if (it->getOccupancy() > 0.5)
        {
            current_index++;
        }
        if (current_index == random_index)
        {
            std::cout << "Randomly selected occupied node: " << it.getCoordinate() << std::endl;
            std::cout << "Occupancy: " << it->getOccupancy() << std::endl;
            octomath::Vector3 tmp = it.getCoordinate();
            Eigen::Vector3d res(tmp.x(), tmp.y(), tmp.z());
            return res;
        }
    }
}

void test_sphere_polygon()
{
    double padding_radius = 0.003;
    std::shared_ptr<coal::CollisionGeometry> robot_geometry = std::make_shared<coal::Box>(0.8, 0.2, 0.3);
    std::shared_ptr<coal::CollisionGeometry> sphere_geometry = std::make_shared<coal::Sphere>(padding_radius);
    coal::Transform3s robot_pose;
    coal::Transform3s sphere_pose = coal::Transform3s::Identity();

    manif::SE3<double> se3_robot_in_world_frame(Eigen::Vector3d(0.0, 0.0, 0.0), Eigen::Quaterniond::Identity());

    robot_pose.setQuatRotation(Eigen::Quaternion<float>(se3_robot_in_world_frame.rotation().cast<float>()));
    robot_pose.setTranslation(se3_robot_in_world_frame.translation().cast<float>());
    sphere_pose.setTranslation(Eigen::Vector3f(0.1, 0.05, 0.154));
    coal::CollisionRequest request;
    coal::CollisionResult result;
    request.num_max_contacts = 1000;
    request.enable_contact = true;
    request.security_margin = padding_radius;

    coal::collide(sphere_geometry.get(), sphere_pose, robot_geometry.get(), robot_pose, request, result);
    std::vector<coal::Contact> contacts;
    if (result.isCollision())
    {
        result.getContacts(contacts);
        std::cout << "Number of contacts: " << result.numContacts() << std::endl;
        for (const auto &contact : contacts)
        {
            std::cout << "Contact position: " << contact.pos.transpose() << std::endl;
            std::cout << "Contact normal: " << contact.normal.transpose() << std::endl;
            std::cout << "Contact penetration depth: " << contact.penetration_depth << std::endl;
            std::cout << "Contact nearest points: " << contact.nearest_points[0].transpose() << ", "
                      << contact.nearest_points[1].transpose() << std::endl;
        }
    }
    else
    {
        std::cout << "No contacts detected." << std::endl;
    }
}

void test_coal_collision_detection()
{
    std::string package_name = "sipoc_ra_support";
    std::filesystem::path path = ament_index_cpp::get_package_share_directory(package_name);
    path += "/meshes/car_seat/car_whole.bt";
    std::string bt_path = path.string();
    std::cout << "Loading octomap from: " << bt_path << std::endl;
    path = ament_index_cpp::get_package_share_directory(package_name);
    path += "/meshes/car_seat/collision/seat_1.stl";

    sipoc_ra_utils::ConfigCollisionElementShape config_collision_element_shape;
    config_collision_element_shape.link_name = "seat";
    config_collision_element_shape.convex.mesh_path = path.string();
    config_collision_element_shape.convex.scale = 0.8;
    config_collision_element_shape.padding = 0.2f;

    std::map<std::string, sipoc_ra_utils::ConfigCollisionElementShape> map_collision_element_shapes = {
        {"seat_1", config_collision_element_shape},
    };

    std::shared_ptr<octomap::OcTree> ot = std::make_shared<octomap::OcTree>(bt_path);
    sipoc_ra_solver::CoalCollisionDetection coal_collision_detection(map_collision_element_shapes, ot, 5.0, 1.5);

    double octomap_resolution = ot->getResolution();
    std::cout << "Octomap resolution: " << octomap_resolution << std::endl;
    Eigen::Vector3d tmp = getrandomOccupiedNode(ot);

    manif::SE3<double> se3_robot(tmp, Eigen::Quaterniond::Identity());
    std::cout << "Robot translation: " << se3_robot.translation().transpose() << std::endl;
    coal::Contact contact;

    std::cout << "Start timer:" << std::endl;
    auto start_box = std::chrono::high_resolution_clock::now();

    coal_collision_detection.detectCollision(se3_robot, "seat_1", contact);

    auto end_box = std::chrono::high_resolution_clock::now();
    std::cout << "Time taken for box shape collision detection: "
              << std::chrono::duration<double>(end_box - start_box).count() << " seconds." << std::endl;
    std::cout << "Contact position: " << contact.pos.transpose() << std::endl;
    std::cout << "Contact normal: " << contact.normal.transpose() << std::endl;
    std::cout << "Contact penetration depth: " << contact.penetration_depth << std::endl;
    std::cout << "Contact nearest points: " << contact.nearest_points[0].transpose() << ", " << contact.nearest_points[1].transpose() << std::endl;

    std::cout << "Test the deepest penetration point for the normal" << std::endl;
    Eigen::Vector3d res_closest_pt_in_link_frame;
    // NOTE: Use the nearest point instead of contact.pos
    coal_collision_detection.ComputeCollisionWRTPoint(se3_robot, "seat_1", contact.nearest_points[0].cast<double>(), res_closest_pt_in_link_frame);

    // Test the case without collision
    std::cout << "Testing with a far away point:" << std::endl;
    Eigen::Vector3d far_away_point(100., 100., 100.);
    Eigen::Vector3d res_closest_pt_in_link_frame2;
    double signed_distance = coal_collision_detection.ComputeCollisionWRTPoint(se3_robot, "seat_1", far_away_point, res_closest_pt_in_link_frame2);
    if (!(signed_distance >= coal_collision_detection.farApartThreshold()))
    {
        std::cout << "Collision detected for the far away point!!!" << std::endl;
    }
}

int main()
{
    srand((unsigned)time(NULL));

    std::cout << "Test Coal Collision Detection for seat_1:" << std::endl;
    test_coal_collision_detection();

    std::cout << "Test Sphere-polygon:" << std::endl;
    test_sphere_polygon();
    std::cout << "# End of the test. #" << std::endl;
    return 0;
}
