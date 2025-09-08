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

#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/jacobian.hpp>

using namespace sipoc_ra_solver;

PinocchioKinematics::PinocchioKinematics(const std::string &urdf_path)
{
    // Load the URDF model
    pinocchio::urdf::buildModel(urdf_path, model_);
    data_ptr_ = std::make_shared<pinocchio::Data>(model_);
    J_pinocchio_tmp_.resize(6, model_.nv);
}

void PinocchioKinematics::getFrameSE3(const std::string &frame_name, manif::SE3<double> &se3_pose)
{
    pinocchio::SE3 tmp_pose;
    if (!model_.existFrame(frame_name))
    {
        throw std::invalid_argument("Frame name '" + frame_name + "' not found in the model.");
    }
    pinocchio::FrameIndex frame_id = model_.getFrameId(frame_name);
    tmp_pose = (*data_ptr_).oMf[frame_id];
    sipoc_ra_utils::SE3Pinocchio2Manif(tmp_pose, se3_pose);
}

void PinocchioKinematics::getModelName(std::string &model_name) const
{
    model_name = model_.name;
}
