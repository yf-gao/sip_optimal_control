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

#ifndef SIPOC_RA_SOLVER__PINOCCHIO_KINEMATICS_IMPL_HPP
#define SIPOC_RA_SOLVER__PINOCCHIO_KINEMATICS_IMPL_HPP

#include "sipoc_ra_solver/pinocchio_kinematics.hpp"
#include "sipoc_ra_utils/utils.hpp"

#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/jacobian.hpp>

namespace sipoc_ra_solver
{
    template <typename Derived>
    void PinocchioKinematics::computeForwardKinematics(const Eigen::MatrixBase<Derived> &joints)
    {
        pinocchio::forwardKinematics(model_, *data_ptr_, joints);
        pinocchio::updateFramePlacements(model_, *data_ptr_);
    }

    template <typename Derived>
    bool PinocchioKinematics::solveInverseKinematics(const manif::SE3<double> &joint_pose, const Eigen::MatrixBase<Derived> &joints_init, const std::string &joint_name, Eigen::MatrixBase<Derived> &joints)
    {
        // The code is from https://gepettoweb.laas.fr/doc/stack-of-tasks/pinocchio/devel/doxygen-html/md_doc_b-examples_i-inverse-kinematics.html

        if (joints.size() != model_.nv || joints_init.size() != model_.nv)
        {
            // Print the size of joints and model nv for debugging
            std::cout << "init joints size=" << joints_init.size() << ", model nv=" << model_.nv << std::endl;
            std::cout << "joints size=" << joints.size() << ", model nv=" << model_.nv << std::endl;
            throw std::invalid_argument("The size of joints vector must match the number of degrees of freedom in the model.");
        }

        int joint_id = model_.getJointId(joint_name);
        if (joint_id < 0 || joint_id >= static_cast<int>(model_.joints.size()))
        {
            throw std::invalid_argument("Invalid Joint name '" + joint_name + "'.");
        }

        pinocchio::SE3 oMdes;
        sipoc_ra_utils::SE3Manif2Pinocchio(joint_pose, oMdes);

        joints = joints_init;
        const double eps = 1e-4;
        const int IT_MAX = 1000;
        const double DT = 1e-1;
        const double damp = 1e-6;

        pinocchio::Data::Matrix6x J(6, model_.nv);
        J.setZero();

        bool success = false;
        Eigen::Matrix<double, 6, 1> err = Eigen::Matrix<double, 6, 1>::Ones();
        Eigen::VectorXd v(model_.nv);
        for (int i = 0;; i++)
        {
            pinocchio::forwardKinematics(model_, *data_ptr_, joints);
            const pinocchio::SE3 dMi = oMdes.actInv(data_ptr_->oMi[joint_id]);
            err = pinocchio::log6(dMi).toVector();
            if (err.norm() < eps)
            {
                success = true;
                break;
            }
            if (i >= IT_MAX)
            {
                success = false;
                break;
            }
            pinocchio::computeJointJacobian(model_, *data_ptr_, joints, joint_id, J); // NOTE: Local frame
            pinocchio::Data::Matrix6 JJt;
            JJt.noalias() = J * J.transpose();
            JJt.diagonal().array() += damp;
            v.noalias() = -J.transpose() * JJt.ldlt().solve(err);
            joints = pinocchio::integrate(model_, joints, v * DT);
        }
        return success;
    }

    template <typename Derived>
    void PinocchioKinematics::computeJacobians(const Eigen::MatrixBase<Derived> &joints, const std::string &frame_name, const manif::SE3<double> &placement, Eigen::MatrixXd &J)
    {
        if (J.rows() != 3 || J.cols() != model_.nv)
        {
            throw std::invalid_argument("Jacobian matrix J must have 3 rows and model_.nv columns.");
        }
        pinocchio::FrameIndex frame_id = model_.getFrameId(frame_name);
        pinocchio::computeJointJacobians(model_, *data_ptr_, joints);
        // ("model", "data", "frame_id", "reference_frame")
        pinocchio::getFrameJacobian(model_, *data_ptr_, frame_id, pinocchio::LOCAL, J_pinocchio_tmp_);

        rot_mat_tmp_ = (*data_ptr_).oMf[frame_id].rotation();
        skew_tmp_ = sipoc_ra_utils::skew(placement.translation());
        J = rot_mat_tmp_ * J_pinocchio_tmp_.block(0, 0, 3, model_.nv) -
            rot_mat_tmp_ * skew_tmp_ * J_pinocchio_tmp_.block(3, 0, 3, model_.nv);

        return;
    }

    template <typename Derived>
    void PinocchioKinematics::getFrameJacobian(const Eigen::MatrixBase<Derived> &joints, const std::string &frame_name, Eigen::MatrixXd &J)
    {
        if (J.rows() != 6 || J.cols() != model_.nv)
        {
            throw std::invalid_argument("Jacobian matrix J must have 6 rows and model_.nv columns.");
        }
        pinocchio::FrameIndex frame_id = model_.getFrameId(frame_name);
        pinocchio::computeJointJacobians(model_, *data_ptr_, joints);
        pinocchio::getFrameJacobian(model_, *data_ptr_, frame_id, pinocchio::LOCAL, J);
    }
}

#endif // SIPOC_RA_SOLVER__PINOCCHIO_KINEMATICS_IMPL_HPP
