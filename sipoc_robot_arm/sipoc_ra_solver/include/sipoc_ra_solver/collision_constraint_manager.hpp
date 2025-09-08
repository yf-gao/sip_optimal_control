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

#ifndef SIPOC_RA_SOLVER__COLLISION_CONSTRAINT_MANAGER_HPP
#define SIPOC_RA_SOLVER__COLLISION_CONSTRAINT_MANAGER_HPP

#include "sipoc_ra_utils/utils.hpp"
#include "sipoc_ra_utils/common_structs.hpp"
#include "sipoc_ra_solver/pinocchio_kinematics.hpp"
#include "sipoc_ra_solver/coal_collision_detection.hpp"
#include "sipoc_ra_solver/openvdb_signed_distance_field.hpp"

#include <coal/collision_data.h>
#include <octomap/octomap.h>

#include <manif/manif.h>
#include <eigen3/Eigen/Dense>
#include <unordered_map>
#include <vector>
#include <string>
#include <boost/format.hpp>

namespace sipoc_ra_solver
{
    namespace collision_manager
    {
        struct CollisionPointInfo
        {
            Eigen::Vector3d position;
            std::string collision_element_name;
            double signed_distance;
        };
    }

    class CollisionConstraintManager
    {
        // Manage the contact points and the corresponding constraint linearization
    public:
        CollisionConstraintManager(const std::map<std::string, sipoc_ra_utils::ConfigCollisionElementShape> &map_collision_element_shapes, const std::string &env_path, unsigned int num_max_iter, unsigned int num_hrzn, unsigned int num_joints, bool use_cropped_octomap);
        ~CollisionConstraintManager();

        bool linearizeConstraints(const Eigen::VectorXd &joint_angles, const std::shared_ptr<PinocchioKinematics> &pinocchio_kin, unsigned int idx, std::vector<Eigen::VectorXd> &vec_constr_coeffs, std::vector<Eigen::Vector<double, 6>> &vec_sep_planes);
        void detectCollisionAndUpdatePCSubset(const std::shared_ptr<PinocchioKinematics> &pinocchio_kin, const Eigen::VectorXd &joint_angles, unsigned int idx_hrzn, sipoc_ra_utils::TimingCollDetection &timing);
        void detectCollisionAndUpdatePCSubset(const std::shared_ptr<PinocchioKinematics> &pinocchio_kin, const Eigen::VectorXd &joint_angles, unsigned int idx_hrzn);
        double evaluateMinimumSignedDistance(const std::shared_ptr<PinocchioKinematics> &pinocchio_kin, const Eigen::VectorXd &joint_angles);
        void clearPCSubsets();
        void getPCSubsetPositions(const std::string &link_name, unsigned int idx, std::vector<Eigen::Vector3d> &pc_positions);
        void updateDistanceByConstrGVals(const Eigen::VectorXd &g_values, unsigned int idx_hrzn);
        void shiftActiveConstraints(const std::vector<int> &index_active_constraints, int idx_hrzn_source, int idx_hrzn_destination, bool keep_destination_constraints);

        inline void setEnvPose(const manif::SE3<double> &se3_env)
        {
            coal_collision_detector_->setEnvPose(se3_env);
            openvdb_sdf_->setEnvTranslation(se3_env.translation());
        }

    private:
        std::unordered_map<std::string, std::vector<collision_manager::CollisionPointInfo>> map_pc_subset_;
        std::map<std::string, double> map_collision_element_padding_radius_;
        std::map<std::string, Eigen::VectorXd> map_collision_element_faces_b_;
        std::map<std::string, std::string> map_collision_element_link_name_;
        std::shared_ptr<CoalCollisionDetection> coal_collision_detector_;
        std::shared_ptr<OpenVDBSignedDistanceField> openvdb_sdf_;
        std::shared_ptr<octomap::OcTree> ot_;
        std::vector<std::string> vec_link_names_;
        coal::Transform3s link_pose_tmp_;

        manif::SE3<double> se3_link_in_world_frame_tmp_;
        std::string hash_map_key_tmp_;
        double octomap_resolution_;
        unsigned int num_max_constr_;
        unsigned int num_hrzn_;
        unsigned int num_joints_;

        void configure(const std::map<std::string, sipoc_ra_utils::ConfigCollisionElementShape> &map_collision_element_shapes);
        void updatePCSubsetOneCollisionElement(const coal::Contact &contact, const std::string &collision_element_name, unsigned int idx);
        bool linearizeConstraintsOneLink(const manif::SE3<double> &se3_link_in_world_frame, const Eigen::VectorXd &joints, const std::string &link_name, const std::shared_ptr<PinocchioKinematics> &pinocchio_kin, unsigned int idx, std::vector<Eigen::VectorXd> &vec_constr_coeffs, std::vector<Eigen::Vector<double, 6>> &vec_sep_planes);

        Eigen::VectorXd gradientFiniteDifference(const Eigen::VectorXd &joints, const Eigen::Vector3d &p_on_robot, const Eigen::Vector3d &p_obs, const std::shared_ptr<PinocchioKinematics> &pinocchio_kin, const std::string &collision_element_name) const;

        inline std::string getKeyString(const std::string &collision_element_name, unsigned int idx) const
        {
            return collision_element_name + (boost::format("%03d") % idx).str();
        }
    };
}

#endif // SIPOC_RA_SOLVER__COLLISION_CONSTRAINT_MANAGER_HPP
