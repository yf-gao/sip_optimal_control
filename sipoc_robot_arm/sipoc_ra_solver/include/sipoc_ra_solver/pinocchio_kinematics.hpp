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

#ifndef SIPOC_RA_SOLVER__PINOCCHIO_KINEMATICS_HPP
#define SIPOC_RA_SOLVER__PINOCCHIO_KINEMATICS_HPP

#include <pinocchio/multibody/data.hpp>
#include <pinocchio/parsers/urdf.hpp>

#include <manif/manif.h>
#include <eigen3/Eigen/Dense>
#include <string>
#include <memory>

namespace sipoc_ra_solver
{
    class PinocchioKinematics
    {
    public:
        PinocchioKinematics(const std::string &urdf_path);

        template <typename Derived>
        void computeForwardKinematics(const Eigen::MatrixBase<Derived> &joints);
        template <typename Derived>
        bool solveInverseKinematics(const manif::SE3<double> &se3_pose, const Eigen::MatrixBase<Derived> &joints_init, const std::string &frame_name, Eigen::MatrixBase<Derived> &joints);
        template <typename Derived>
        void computeJacobians(const Eigen::MatrixBase<Derived> &joints, const std::string &frame_name, const manif::SE3<double> &placement, Eigen::MatrixXd &J);
        template <typename Derived>
        void getFrameJacobian(const Eigen::MatrixBase<Derived> &joints, const std::string &frame_name, Eigen::MatrixXd &J);

        void getFrameSE3(const std::string &frame_name, manif::SE3<double> &se3_pose);
        void getModelName(std::string &model_name) const;

        inline unsigned int getModelNv() const { return model_.nv; }
        inline void getRandomConfiguration(Eigen::VectorXd &joints) const
        {
            joints = pinocchio::randomConfiguration(model_);
        }

    private:
        pinocchio::Model model_;
        std::shared_ptr<pinocchio::Data> data_ptr_;
        Eigen::MatrixXd J_pinocchio_tmp_;
        Eigen::Matrix3d rot_mat_tmp_;
        Eigen::Matrix3d skew_tmp_;
    };
} // namespace sipoc_ra_solver

#include "sipoc_ra_solver/pinocchio_kinematics-impl.hpp"

#endif // SIPOC_RA_SOLVER__PINOCCHIO_KINEMATICS_HPP
