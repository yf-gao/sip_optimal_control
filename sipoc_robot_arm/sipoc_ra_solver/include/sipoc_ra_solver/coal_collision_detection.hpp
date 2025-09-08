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

#ifndef SIPOC_RA_SOLVER__COAL_COLLISION_DETECTION_HPP
#define SIPOC_RA_SOLVER__COAL_COLLISION_DETECTION_HPP

#include "sipoc_ra_utils/common_structs.hpp"
#include "sipoc_ra_utils/utils.hpp"

#include <coal/collision_object.h>
#include <coal/collision_data.h>
#include <coal/octree.h>

#include <octomap/octomap.h>
#include <manif/manif.h>
#include <eigen3/Eigen/Dense>

#include <vector>
#include <memory>
#include <unordered_map>
#include <string>

namespace sipoc_ra_solver

{
    class CoalCollisionDetection
    {
    public:
        CoalCollisionDetection(const std::map<std::string, sipoc_ra_utils::ConfigCollisionElementShape> &map_collision_element_shapes, std::shared_ptr<octomap::OcTree> ot, double far_apart_threshold, double sphere_radius_resolution_ratio);
        ~CoalCollisionDetection();

        bool detectCollision(const manif::SE3<double> &se3_link, const std::string &collision_element_name, coal::Contact &contact, sipoc_ra_utils::TimingCollDetection &timing);
        bool detectCollision(const manif::SE3<double> &se3_link, const std::string &collision_element_name, coal::Contact &contact);

        double ComputeCollisionWRTPoint(const manif::SE3<double> &se3_link, const std::string &collision_element_name, const Eigen::Vector3d &pt_position, Eigen::Vector3d &closest_pt_in_link_frame);

        inline double sphereRadius() const
        {
            return sphere_radius_;
        }

        inline double farApartThreshold() const
        {
            return far_apart_threshold_;
        }

        inline void setEnvPose(const manif::SE3<double> &se3_env)
        {
            env_pose_.setQuatRotation(Eigen::Quaternion<float>(se3_env.rotation().cast<float>()));
            env_pose_.setTranslation(se3_env.translation().cast<float>());
        }

    private:
        std::map<std::string, std::shared_ptr<coal::CollisionGeometry>> map_collision_element_geometry_;
        std::map<std::string, double> map_collision_element_padding_radius_;
        std::shared_ptr<coal::OcTree> env_;
        std::shared_ptr<coal::CollisionGeometry> sphere_geometry_;
        coal::CollisionRequest request_octree_, request_sphere_;
        coal::Transform3s env_pose_;
        coal::Transform3s link_pose_tmp_;
        coal::Transform3s sphere_pose_tmp_;
        double far_apart_threshold_, sphere_radius_;
    };
}

#endif // SIPOC_RA_SOLVER__COAL_COLLISION_DETECTION_HPP
