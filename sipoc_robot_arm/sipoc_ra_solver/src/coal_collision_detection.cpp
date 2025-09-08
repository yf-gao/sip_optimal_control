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

#include <coal/serialization/geometric_shapes.h>
#include <coal/collision.h>
#include <coal/mesh_loader/loader.h>
#include <coal/BVH/BVH_model.h>
#include <chrono>

using namespace sipoc_ra_solver;

CoalCollisionDetection::CoalCollisionDetection(const std::map<std::string, sipoc_ra_utils::ConfigCollisionElementShape> &map_collision_element_shapes,
                                               std::shared_ptr<octomap::OcTree> ot,
                                               double far_apart_threshold,
                                               double sphere_radius_resolution_ratio) : far_apart_threshold_(far_apart_threshold)
{
    env_ = std::make_shared<coal::OcTree>(ot);
    env_pose_ = coal::Transform3s::Identity();
    sphere_radius_ = sphere_radius_resolution_ratio * ot->getResolution();
    sphere_geometry_ = std::make_shared<coal::Sphere>(sphere_radius_);
    sphere_pose_tmp_ = coal::Transform3s::Identity();

    std::shared_ptr<coal::CollisionGeometry> unique_ptr_robot_geometry;
    map_collision_element_geometry_.clear();
    map_collision_element_padding_radius_.clear();
    for (const auto &pair : map_collision_element_shapes)
    {
        const sipoc_ra_utils::ConfigCollisionElementShape &config_shape = pair.second;
        coal::NODE_TYPE bv_type = coal::BV_AABB;
        coal::MeshLoader loader(bv_type);
        coal::BVHModelPtr_t bvh = loader.load(config_shape.convex.mesh_path, config_shape.convex.scale * coal::Vec3s::Ones());
        bvh->buildConvexHull(true, "Qt");
        unique_ptr_robot_geometry = bvh->convex;
        map_collision_element_geometry_.insert(std::make_pair(pair.first, std::move(unique_ptr_robot_geometry)));
        map_collision_element_padding_radius_.insert(std::make_pair(pair.first, config_shape.padding));
    }

    request_octree_.num_max_contacts = 150; // NOTE: hyperparameter
    request_octree_.enable_contact = true;
    request_sphere_.num_max_contacts = 1;
    request_sphere_.enable_contact = true;
    request_sphere_.security_margin = far_apart_threshold_;
}

CoalCollisionDetection::~CoalCollisionDetection() = default;

bool CoalCollisionDetection::detectCollision(const manif::SE3<double> &se3_link, const std::string &collision_element_name, coal::Contact &contact, sipoc_ra_utils::TimingCollDetection &timing)
{
    auto start_coal = std::chrono::high_resolution_clock::now();

    link_pose_tmp_.setQuatRotation(Eigen::Quaternion<float>(se3_link.rotation().cast<float>()));
    link_pose_tmp_.setTranslation(se3_link.translation().cast<float>());

    request_octree_.security_margin = map_collision_element_padding_radius_.at(collision_element_name);
    // The algorithm will terminate early if it finds that the shapes are at least `distance_upper_bound` apart.
    request_octree_.distance_upper_bound = 1.02 * request_octree_.security_margin;
    coal::CollisionResult result;
    coal::collide(env_.get(), env_pose_, map_collision_element_geometry_.at(collision_element_name).get(), link_pose_tmp_, request_octree_, result);

    std::vector<coal::Contact> contacts;
    result.getContacts(contacts);
    if (!result.isCollision())
    {
        timing.time_coal += std::chrono::duration<double>(std::chrono::high_resolution_clock::now() - start_coal).count();
        return false;
    }

    auto start_min_element = std::chrono::high_resolution_clock::now();
    contact = *min_element(contacts.begin(), contacts.end(),
                           [](const coal::Contact &i, const coal::Contact &j)
                           {
                               return i.penetration_depth < j.penetration_depth;
                           });
    auto end_min_element = std::chrono::high_resolution_clock::now();
    timing.time_coal += std::chrono::duration<double>(start_min_element - start_coal).count();
    timing.time_min_element += std::chrono::duration<double>(end_min_element - start_min_element).count();
    return true;
}

bool CoalCollisionDetection::detectCollision(const manif::SE3<double> &se3_link, const std::string &collision_element_name, coal::Contact &contact)
{
    // NOTE: The info object is not used in this function.
    sipoc_ra_utils::TimingCollDetection info;
    return detectCollision(se3_link, collision_element_name, contact, info);
}

double CoalCollisionDetection::ComputeCollisionWRTPoint(const manif::SE3<double> &se3_link, const std::string &collision_element_name, const Eigen::Vector3d &pt_position, Eigen::Vector3d &closest_pt_in_link_frame)
{
    link_pose_tmp_.setQuatRotation(Eigen::Quaternion<float>(se3_link.rotation().cast<float>()));
    link_pose_tmp_.setTranslation(se3_link.translation().cast<float>());
    sphere_pose_tmp_.setTranslation(pt_position.cast<float>());

    coal::CollisionResult result;
    coal::collide(sphere_geometry_.get(), sphere_pose_tmp_, map_collision_element_geometry_.at(collision_element_name).get(), link_pose_tmp_, request_sphere_, result);
    if (!result.isCollision())
    {
        std::cout << "No collision detected. The collision element and point is far apart." << std::endl;
        closest_pt_in_link_frame << nan(""), nan(""), nan("");
        return far_apart_threshold_ + 0.1; // Return a value larger than the far_apart_threshold_ to indicate no collision
    }
    coal::Contact contact = result.getContacts()[0];
    manif::SE3<float> se3_link_float = se3_link.cast<float>();
    closest_pt_in_link_frame = (se3_link_float.rotation().transpose() * (contact.nearest_points[1] - se3_link_float.translation())).cast<double>();
    return contact.penetration_depth;
}
