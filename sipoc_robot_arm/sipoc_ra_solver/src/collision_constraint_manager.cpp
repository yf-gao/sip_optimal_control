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
#include "sipoc_ra_utils/utils.hpp"

#include <coal/shape/geometric_shapes.h>
#include <coal/collision.h>
#include <chrono>

#include <algorithm>
#include <stdexcept>
#include <iostream>
#include <string>

using namespace sipoc_ra_solver;

CollisionConstraintManager::CollisionConstraintManager(const std::map<std::string, sipoc_ra_utils::ConfigCollisionElementShape> &map_collision_element_shapes,
                                                       const std::string &env_path,
                                                       unsigned int num_max_constr,
                                                       unsigned int num_hrzn,
                                                       unsigned int num_joints,
                                                       bool use_cropped_octomap)
    : num_max_constr_(num_max_constr), num_hrzn_(num_hrzn), num_joints_(num_joints)
{
    std::string ot_path;
    if (use_cropped_octomap)
    {
        ot_path = env_path + "_cropped.bt";
    }
    else
    {
        ot_path = env_path + ".bt";
    }
    ot_ = std::make_shared<octomap::OcTree>(ot_path);
    coal_collision_detector_ = std::make_shared<CoalCollisionDetection>(map_collision_element_shapes, ot_, 5., 0.8); // NOTE: hyperparameter
    openvdb_sdf_ = std::make_shared<OpenVDBSignedDistanceField>(env_path, ot_->getResolution());

    this->configure(map_collision_element_shapes);
}

CollisionConstraintManager::~CollisionConstraintManager() = default;

void CollisionConstraintManager::configure(const std::map<std::string, sipoc_ra_utils::ConfigCollisionElementShape> &map_collision_element_shapes)
{
    map_collision_element_faces_b_.clear();
    map_collision_element_padding_radius_.clear();
    map_collision_element_link_name_.clear();
    Eigen::VectorXd polygon_faces_b(6);
    for (const auto &pair : map_collision_element_shapes)
    {
        const sipoc_ra_utils::ConfigCollisionElementShape &config_shape = pair.second;
        map_collision_element_padding_radius_.insert(std::make_pair(pair.first, config_shape.padding));
        map_collision_element_link_name_.insert(std::make_pair(pair.first, config_shape.link_name));
    }

    vec_link_names_.clear();
    map_pc_subset_.clear();
    for (const auto &pair : map_collision_element_link_name_)
    {
        if (std::find(vec_link_names_.begin(), vec_link_names_.end(), pair.second) != vec_link_names_.end())
        {
            continue; // The link has already been added. (Multiple collision elements may share the same link name)
        }
        vec_link_names_.push_back(pair.second);
        for (unsigned int i = 0; i <= num_hrzn_; ++i)
        {
            hash_map_key_tmp_ = this->getKeyString(pair.second, i);
            map_pc_subset_[hash_map_key_tmp_] = std::vector<collision_manager::CollisionPointInfo>();
        }
    }

    octomap_resolution_ = ot_->getResolution();
}

void CollisionConstraintManager::updatePCSubsetOneCollisionElement(const coal::Contact &contact, const std::string &collision_element_name, unsigned int idx_hrzn)
{
    hash_map_key_tmp_ = this->getKeyString(map_collision_element_link_name_.at(collision_element_name), idx_hrzn);

    bool contact_in_subset = std::any_of(map_pc_subset_.at(hash_map_key_tmp_).begin(), map_pc_subset_.at(hash_map_key_tmp_).end(),
                                         [&contact, this](const collision_manager::CollisionPointInfo &pc_info)
                                         {
                                             return (pc_info.position - contact.nearest_points[0].cast<double>()).norm() < this->octomap_resolution_ * 0.5; // NOTE: hyperparameter
                                         });
    if (contact_in_subset)
    {
        return;
    }
    if (map_pc_subset_.at(hash_map_key_tmp_).size() >= num_max_constr_)
    {
        std::vector<collision_manager::CollisionPointInfo>::iterator tmp_iter = std::max_element(map_pc_subset_.at(hash_map_key_tmp_).begin(), map_pc_subset_.at(hash_map_key_tmp_).end(),
                                                                                                 [](const collision_manager::CollisionPointInfo &i, const collision_manager::CollisionPointInfo &j)
                                                                                                 {
                                                                                                     return i.signed_distance < j.signed_distance;
                                                                                                 });
        map_pc_subset_.at(hash_map_key_tmp_).erase(tmp_iter);
    }
    collision_manager::CollisionPointInfo tmp_info;
    // NOTE: The nearest points of the environment in the world frame.
    tmp_info.position = contact.nearest_points[0].cast<double>();
    tmp_info.signed_distance = contact.penetration_depth + coal_collision_detector_->sphereRadius();
    tmp_info.collision_element_name = collision_element_name;
    map_pc_subset_.at(hash_map_key_tmp_).push_back(tmp_info);
}

void CollisionConstraintManager::detectCollisionAndUpdatePCSubset(const std::shared_ptr<PinocchioKinematics> &pinocchio_kin, const Eigen::VectorXd &joint_angles, unsigned int idx_hrzn, sipoc_ra_utils::TimingCollDetection &timing)
{
    if (idx_hrzn > num_hrzn_)
    {
        throw std::out_of_range("idx_hrzn out of range.");
    }
    for (const auto &pair : map_collision_element_link_name_)
    {
        pinocchio_kin->computeForwardKinematics(joint_angles);
        pinocchio_kin->getFrameSE3(pair.second, se3_link_in_world_frame_tmp_);
        coal::Contact contact;
        bool if_in_collision = coal_collision_detector_->detectCollision(se3_link_in_world_frame_tmp_, pair.first, contact, timing);

        auto start_update = std::chrono::high_resolution_clock::now();
        if (if_in_collision)
        {
            this->updatePCSubsetOneCollisionElement(contact, pair.first, idx_hrzn);
        }
        auto end_update = std::chrono::high_resolution_clock::now();
        timing.time_update += std::chrono::duration<double>(end_update - start_update).count();
    }
}

void CollisionConstraintManager::detectCollisionAndUpdatePCSubset(const std::shared_ptr<PinocchioKinematics> &pinocchio_kin, const Eigen::VectorXd &joint_angles, unsigned int idx_hrzn)
{
    sipoc_ra_utils::TimingCollDetection timing;
    this->detectCollisionAndUpdatePCSubset(pinocchio_kin, joint_angles, idx_hrzn, timing);
}

double CollisionConstraintManager::evaluateMinimumSignedDistance(const std::shared_ptr<PinocchioKinematics> &pinocchio_kin, const Eigen::VectorXd &joint_angles)
{
    double min_signed_distance = 0.0; // NOTE: The collision will not be detected once the distance is greater than the padding radius.
    for (const auto &pair : map_collision_element_link_name_)
    {
        pinocchio_kin->computeForwardKinematics(joint_angles);
        pinocchio_kin->getFrameSE3(pair.second, se3_link_in_world_frame_tmp_);
        coal::Contact contact;
        bool if_in_collision = coal_collision_detector_->detectCollision(se3_link_in_world_frame_tmp_, pair.first, contact);
        if (if_in_collision)
        {
            min_signed_distance = std::min(min_signed_distance, contact.penetration_depth - map_collision_element_padding_radius_.at(pair.first));
        }
    }
    return min_signed_distance;
}

bool CollisionConstraintManager::linearizeConstraintsOneLink(const manif::SE3<double> &se3_link_in_world_frame,
                                                             const Eigen::VectorXd &joints,
                                                             const std::string &link_name,
                                                             const std::shared_ptr<PinocchioKinematics> &pinocchio_kin,
                                                             unsigned int idx_hrzn,
                                                             std::vector<Eigen::VectorXd> &vec_constr_coeffs,
                                                             std::vector<Eigen::Vector<double, 6>> &vec_sep_planes)
{
    hash_map_key_tmp_ = this->getKeyString(link_name, idx_hrzn);

    Eigen::MatrixXd J_pinocchio(3, pinocchio_kin->getModelNv());
    Eigen::Vector3d pt_in_link_frame, p_pt2obs, separating_normal;
    Eigen::Vector<double, 6> separating_plane; // NOTE: Only for debug
    manif::SE3<double> se3_pt_in_link_frame, se3_pt_in_world_frame;
    se3_pt_in_link_frame.quat(Eigen::Quaterniond::Identity());
    double signed_distance_coal;

    for (auto iter = map_pc_subset_.at(hash_map_key_tmp_).begin(); iter != map_pc_subset_.at(hash_map_key_tmp_).end(); ++iter)
    {
        signed_distance_coal = coal_collision_detector_->ComputeCollisionWRTPoint(se3_link_in_world_frame, iter->collision_element_name, iter->position, pt_in_link_frame);
        se3_pt_in_link_frame.translation(pt_in_link_frame);
        // Compute the coefficients for the linearized constraints
        pinocchio_kin->computeJacobians(joints, map_collision_element_link_name_.at(iter->collision_element_name), se3_pt_in_link_frame, J_pinocchio);
        Eigen::VectorXd constr_coeffs(num_joints_ + 1);

        se3_pt_in_world_frame = se3_link_in_world_frame * se3_pt_in_link_frame;
        separating_plane.head<3>() = iter->position;

        if (signed_distance_coal >= 0.0) // Separate by at least a radius of sphereRadius
        {
            p_pt2obs = se3_pt_in_world_frame.translation() - iter->position;
            constr_coeffs.head(num_joints_) = J_pinocchio.transpose() * p_pt2obs.normalized();
            constr_coeffs(num_joints_) = p_pt2obs.norm() - constr_coeffs.head(num_joints_).dot(joints) - map_collision_element_padding_radius_.at(iter->collision_element_name);
            vec_constr_coeffs.push_back(constr_coeffs);
            separating_plane.tail<3>() = p_pt2obs.normalized();
        }
        else
        {
            openvdb_sdf_->ComputeNormal(iter->position, separating_normal);
            constr_coeffs.head(num_joints_) = J_pinocchio.transpose() * separating_normal;
            // J.T (q_new - q_cur) >= -penetration + 1.0 * sphere_radius. (NOTE: Hyperparameter)
            constr_coeffs(num_joints_) = signed_distance_coal - constr_coeffs.head(num_joints_).dot(joints) - coal_collision_detector_->sphereRadius();
            vec_constr_coeffs.push_back(constr_coeffs);
            separating_plane.tail<3>() = separating_normal;
        }
        vec_sep_planes.push_back(separating_plane); // NOTE: Only for debug
    }
    return true;
}

bool CollisionConstraintManager::linearizeConstraints(const Eigen::VectorXd &joint_angles,
                                                      const std::shared_ptr<PinocchioKinematics> &pinocchio_kin,
                                                      unsigned int idx_hrzn,
                                                      std::vector<Eigen::VectorXd> &vec_constr_coeffs,
                                                      std::vector<Eigen::Vector<double, 6>> &vec_sep_planes)
{
    if (idx_hrzn > num_hrzn_)
    {
        throw std::out_of_range("idx_hrzn out of range.");
    }
    vec_constr_coeffs.clear();
    vec_sep_planes.clear();
    for (const std::string &link_name : vec_link_names_)
    {
        pinocchio_kin->computeForwardKinematics(joint_angles);
        pinocchio_kin->getFrameSE3(link_name, se3_link_in_world_frame_tmp_);
        bool success_lin_constr = this->linearizeConstraintsOneLink(se3_link_in_world_frame_tmp_, joint_angles, link_name, pinocchio_kin, idx_hrzn, vec_constr_coeffs, vec_sep_planes);
        if (!success_lin_constr)
        {
            return false;
        }
    }
    return true;
}

Eigen::VectorXd CollisionConstraintManager::gradientFiniteDifference(const Eigen::VectorXd &joints,
                                                                     const Eigen::Vector3d &p_on_robot,
                                                                     const Eigen::Vector3d &p_obs,
                                                                     const std::shared_ptr<PinocchioKinematics> &pinocchio_kin,
                                                                     const std::string &collision_element_name) const
{
    double delta = 1e-5;
    manif::SE3<double> robot_pose, robot_pose2;
    Eigen::VectorXd J_finite_diff(6);
    Eigen::Vector3d old_offset, new_offset_by_fk;
    pinocchio_kin->computeForwardKinematics(joints);
    pinocchio_kin->getFrameSE3(map_collision_element_link_name_.at(collision_element_name), robot_pose);
    old_offset = robot_pose.translation() + robot_pose.rotation() * p_on_robot - p_obs;

    Eigen::VectorXd delta_robot_joints(pinocchio_kin->getModelNv());
    for (unsigned int i = 0; i < pinocchio_kin->getModelNv(); ++i)
    {
        delta_robot_joints.setZero();
        delta_robot_joints(i) = delta;

        pinocchio_kin->computeForwardKinematics(joints + delta_robot_joints);
        pinocchio_kin->getFrameSE3(map_collision_element_link_name_.at(collision_element_name), robot_pose2);
        new_offset_by_fk = robot_pose2.translation() + robot_pose2.rotation() * p_on_robot - p_obs;
        J_finite_diff(i) = (new_offset_by_fk.norm() - old_offset.norm()) / delta;
    }
    return J_finite_diff;
}

void CollisionConstraintManager::clearPCSubsets()
{
    for (const std::string &link_name : vec_link_names_)
    {
        for (unsigned int i = 0; i <= num_hrzn_; ++i)
        {
            hash_map_key_tmp_ = this->getKeyString(link_name, i);
            map_pc_subset_.at(hash_map_key_tmp_).clear();
        }
    }
}

void CollisionConstraintManager::getPCSubsetPositions(const std::string &link_name, unsigned int idx, std::vector<Eigen::Vector3d> &pc_positions)
{
    hash_map_key_tmp_ = this->getKeyString(link_name, idx);
    pc_positions.clear();
    for (auto it = map_pc_subset_.at(hash_map_key_tmp_).begin(); it != map_pc_subset_.at(hash_map_key_tmp_).end(); ++it)
    {
        pc_positions.push_back(it->position);
    }
}

void CollisionConstraintManager::updateDistanceByConstrGVals(const Eigen::VectorXd &g_values, unsigned int idx_hrzn)
{
    if (g_values.size() != static_cast<int>(num_max_constr_ * vec_link_names_.size()))
    {
        std::cerr << "g_values size: " << g_values.size() << ", num_max_constr_: " << num_max_constr_ << ", vec_link_names_.size(): " << vec_link_names_.size() << std::endl;
        throw std::invalid_argument("g_values size does not match the number of `num_max_constr_ * vec_link_names_.size()`.");
    }
    unsigned int idx_link = 0, idx_constr = 0;
    for (const std::string &link_name : vec_link_names_)
    {
        hash_map_key_tmp_ = this->getKeyString(link_name, idx_hrzn);
        std::vector<collision_manager::CollisionPointInfo> &pc_subset = map_pc_subset_.at(hash_map_key_tmp_);
        idx_constr = 0;
        for (auto collision_point : pc_subset)
        {
            // NOTE: The constraints are ordered by the links in `vec_link_names_`
            pc_subset[idx_constr].signed_distance = g_values(idx_link * num_max_constr_ + idx_constr);
            idx_constr++;
        }
        idx_link++;
    }
    return;
}

void CollisionConstraintManager::shiftActiveConstraints(const std::vector<int> &index_active_constraints,
                                                        int idx_hrzn_source,
                                                        int idx_hrzn_destination,
                                                        bool keep_destination_constraints)
{
    if (!keep_destination_constraints)
    {
        for (const std::string &link_name : vec_link_names_)
        {
            hash_map_key_tmp_ = this->getKeyString(link_name, idx_hrzn_destination);
            map_pc_subset_.at(hash_map_key_tmp_).clear();
        }
    }
    int idx_constr = 0;
    int idx_vector = 0;
    std::string hash_map_key_source, hash_map_key_destination;
    for (const std::string &link_name : vec_link_names_)
    {
        hash_map_key_source = this->getKeyString(link_name, idx_hrzn_source);
        hash_map_key_destination = this->getKeyString(link_name, idx_hrzn_destination);
        for (auto iter = map_pc_subset_.at(hash_map_key_source).begin(); iter != map_pc_subset_.at(hash_map_key_source).end(); ++iter)
        {
            if (idx_vector >= static_cast<int>(index_active_constraints.size()))
            {
                break; // No more active constraints to shift
            }
            if (map_pc_subset_.at(hash_map_key_destination).size() >= num_max_constr_)
            {
                break; // The destination subset has reached the maximum number
            }
            if (idx_constr == index_active_constraints[idx_vector])
            {
                ++idx_vector;
                map_pc_subset_.at(hash_map_key_destination).push_back(*iter);
            }
            ++idx_constr;
        }
        if (idx_vector >= static_cast<int>(index_active_constraints.size()) || map_pc_subset_.at(hash_map_key_destination).size() >= num_max_constr_)
        {
            break;
        }
    }
}
