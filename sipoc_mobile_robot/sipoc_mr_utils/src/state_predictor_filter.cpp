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
// Authors: Yunfan Gao, Niels van Duijkeren
//

#include "sipoc_mr_utils/state_predictor_filter.hpp"

#include <unsupported/Eigen/MatrixFunctions>

#include <iostream>
#include <stdexcept>

namespace sipoc_mr_utils
{

    StatePredictorFilter::StatePredictorFilter(
        const Eigen::Matrix<double, NX_SID, NX_SID> &A,
        const Eigen::Matrix<double, NX_SID, NX_VREF> &B,
        const Eigen::Matrix<double, NX_VREF, NX_SID> &C,
        const Eigen::Matrix<double, NX_VREF, NX_VREF> &D,
        const Eigen::Matrix<double, NX_SID, NX_SID> &Q,
        const Eigen::Matrix<double, NX_VREF, NX_VREF> &R,
        const Eigen::Matrix<double, NX_SID, NX_SID> &P0,
        double dt)
        : A_(A), B_(B), C_(C), D_(D), Q_(Q), R_(R), P0_(P0), dt_(dt), initialized_(false)
    {
        Eigen::Matrix<double, NX_SID + NX_VREF, NX_SID + NX_VREF> discretizer_matrix;
        discretizer_matrix.setIdentity();
        discretizer_matrix.topLeftCorner<NX_SID, NX_SID>() = A;
        discretizer_matrix.topRightCorner<NX_SID, NX_VREF>() = B;
        discretizer_matrix = discretizer_matrix.log() / dt;

        Ac_ = discretizer_matrix.topLeftCorner<NX_SID, NX_SID>();
        Bc_ = discretizer_matrix.topRightCorner<NX_SID, NX_VREF>();
    }

    void StatePredictorFilter::init()
    {
        this->state_tm_delay_meas_.x_sid.setZero();
        this->state_tm_delay_meas_.x_vref.setZero();
        this->P_ = this->P0_;
        this->initialized_ = true;
    }

    void StatePredictorFilter::init(const Eigen::Vector<double, NX_SID> &x_sid, const Eigen::Vector<double, NX_VREF> &x_vref)
    {
        this->state_tm_delay_meas_.x_sid = x_sid;
        this->state_tm_delay_meas_.x_vref = x_vref;
        this->P_ = this->P0_;
        this->initialized_ = true;
    }

    void StatePredictorFilter::update(const Eigen::Vector<double, NX_POSE> &pose_measured, const Eigen::Vector<double, NX_VREF> &v_measured, const Eigen::Vector<double, NX_VREF> &vref)
    {

        if (!initialized_)
            throw std::runtime_error("Filter is not initialized!");

        state_tm_delay_meas_.x_pose = pose_measured;
        state_tm_delay_meas_.x_vref = vref;

        P_ = A_ * P_ * A_.transpose() + Q_;
        K_ = P_ * C_.transpose() * (C_ * P_ * C_.transpose() + R_).inverse();
        x_sid_filtered_tmp_ = state_tm_delay_meas_.x_sid + K_ * (v_measured - D_ * state_tm_delay_meas_.x_vref - C_ * state_tm_delay_meas_.x_sid);
        P_ = (Eigen::Matrix<double, NX_SID, NX_SID>::Identity() - K_ * C_) * P_;
        state_tm_delay_meas_.x_sid = x_sid_filtered_tmp_;
    }

    void StatePredictorFilter::predictOCPInitialState(const boost::circular_buffer<Eigen::Vector<double, NU_A>> &cb_u, Eigen::Vector<double, NX_POSE + NX_SID + NX_VREF> &ocp_init_state_concat, bool if_incorporate_sys_id)
    {
        unsigned int num_steps = n_step_delay_meas_ + n_step_delay_comp_;
        if (cb_u.size() < num_steps)
        {
            throw std::runtime_error("Insufficient number of control inputs provided for OCP initial state computation.");
        }

        state_tmp_ = state_tm_delay_meas_;
        if (if_incorporate_sys_id)
        {
            predictOneStep(cb_u[0]);
        }
        else
        {
            predictOneStepWoSysID(cb_u[0]);
        }
        state_tm_delay_meas_ = state_tmp_; // Will be used for the next call of update() and predictOCPInitialState()

        if (if_incorporate_sys_id)
        {
            for (unsigned int i = 1; i < num_steps; ++i)
            {
                predictOneStep(cb_u[i]);
            }
        }
        else
        {
            for (unsigned int i = 1; i < num_steps; ++i)
            {
                predictOneStepWoSysID(cb_u[i]);
            }
        }

        ocp_init_state_concat.head<NX_POSE>() = state_tmp_.x_pose;
        ocp_init_state_concat.segment<NX_VREF>(NX_POSE) = state_tmp_.x_vref;
        ocp_init_state_concat.tail<NX_SID>() = state_tmp_.x_sid;
    }

    void StatePredictorFilter::predictOCPInitialUncertainty(Eigen::Matrix<double, NX_SID, NX_SID> &Sigma0)
    {
        Sigma0 = P_;
        unsigned int num_steps = n_step_delay_meas_ + n_step_delay_comp_;
        for (unsigned int i = 0; i < num_steps; ++i)
        {
            Sigma0 = A_ * Sigma0 * A_.transpose() + Q_;
        }
    }

    void StatePredictorFilter::predictOneStep(const Eigen::Vector<double, NU_A> &u)
    {
        Eigen::Vector2d vel_tmp;
        Eigen::Vector<double, NX_SID> x_sid_dot;

        initial_state_ << state_tmp_.x_pose, state_tmp_.x_vref, state_tmp_.x_sid;

        rk4_tmp_ = initial_state_;
        vel_tmp = C_ * rk4_tmp_.tail<NX_SID>() + D_ * rk4_tmp_.segment<NX_VREF>(3);
        x_sid_dot = Ac_ * rk4_tmp_.tail<NX_SID>() + Bc_ * rk4_tmp_.segment<NX_VREF>(3);
        rk4_k0_ << vel_tmp.coeff(0) * cos(rk4_tmp_.coeff(2)), vel_tmp.coeff(0) * sin(rk4_tmp_.coeff(2)), vel_tmp.coeff(1), u, x_sid_dot;

        rk4_tmp_ = initial_state_ + rk4_k0_ * dt_ / 2.0;
        vel_tmp = C_ * rk4_tmp_.tail<NX_SID>() + D_ * rk4_tmp_.segment<NX_VREF>(3);
        x_sid_dot = Ac_ * rk4_tmp_.tail<NX_SID>() + Bc_ * rk4_tmp_.segment<NX_VREF>(3);
        rk4_k1_ << vel_tmp.coeff(0) * cos(rk4_tmp_.coeff(2)), vel_tmp.coeff(0) * sin(rk4_tmp_.coeff(2)), vel_tmp.coeff(1), u, x_sid_dot;

        rk4_tmp_ = initial_state_ + rk4_k1_ * dt_ / 2.0;
        vel_tmp = C_ * rk4_tmp_.tail<NX_SID>() + D_ * rk4_tmp_.segment<NX_VREF>(3);
        x_sid_dot = Ac_ * rk4_tmp_.tail<NX_SID>() + Bc_ * rk4_tmp_.segment<NX_VREF>(3);
        rk4_k2_ << vel_tmp.coeff(0) * cos(rk4_tmp_.coeff(2)), vel_tmp.coeff(0) * sin(rk4_tmp_.coeff(2)), vel_tmp.coeff(1), u, x_sid_dot;

        rk4_tmp_ = initial_state_ + rk4_k2_ * dt_;
        vel_tmp = C_ * rk4_tmp_.tail<NX_SID>() + D_ * rk4_tmp_.segment<NX_VREF>(3);
        x_sid_dot = Ac_ * rk4_tmp_.tail<NX_SID>() + Bc_ * rk4_tmp_.segment<NX_VREF>(3);
        rk4_k3_ << vel_tmp.coeff(0) * cos(rk4_tmp_.coeff(2)), vel_tmp.coeff(0) * sin(rk4_tmp_.coeff(2)), vel_tmp.coeff(1), u, x_sid_dot;

        state_next_ = initial_state_ + 1.0 / 6.0 * dt_ * (rk4_k0_ + 2 * rk4_k1_ + 2 * rk4_k2_ + rk4_k3_);
        state_next_tmp_.x_pose = state_next_.head<NX_POSE>();
        state_next_tmp_.x_vref = state_next_.segment<NX_VREF>(NX_POSE);
        state_next_tmp_.x_sid = state_next_.tail<NX_SID>();

        state_tmp_ = state_next_tmp_;
    }

    void StatePredictorFilter::predictOneStepWoSysID(const Eigen::Vector<double, NU_A> &u)
    {
        Eigen::Vector<double, NX_POSE + NX_VREF> rk4_tmp, rk4_k0, rk4_k1, rk4_k2, rk4_k3, initial_state;

        initial_state << state_tmp_.x_pose, state_tmp_.x_vref;

        rk4_tmp = initial_state;
        rk4_k0 << rk4_tmp.coeff(3) * cos(rk4_tmp.coeff(2)), rk4_tmp.coeff(3) * sin(rk4_tmp.coeff(2)), rk4_tmp.coeff(4), u;

        rk4_tmp = initial_state + rk4_k0 * dt_ / 2.0;
        rk4_k1 << rk4_tmp.coeff(3) * cos(rk4_tmp.coeff(2)), rk4_tmp.coeff(3) * sin(rk4_tmp.coeff(2)), rk4_tmp.coeff(4), u;

        rk4_tmp = initial_state + rk4_k1 * dt_ / 2.0;
        rk4_k2 << rk4_tmp.coeff(3) * cos(rk4_tmp.coeff(2)), rk4_tmp.coeff(3) * sin(rk4_tmp.coeff(2)), rk4_tmp.coeff(4), u;

        rk4_tmp = initial_state + rk4_k2 * dt_;
        rk4_k3 << rk4_tmp.coeff(3) * cos(rk4_tmp.coeff(2)), rk4_tmp.coeff(3) * sin(rk4_tmp.coeff(2)), rk4_tmp.coeff(4), u;

        state_next_.head<NX_POSE + NX_VREF>() = initial_state + 1.0 / 6.0 * dt_ * (rk4_k0 + 2 * rk4_k1 + 2 * rk4_k2 + rk4_k3);
        state_next_.tail<NX_SID>() = A_ * state_tmp_.x_sid + B_ * state_tmp_.x_vref;
        state_next_tmp_.x_pose = state_next_.head<NX_POSE>();
        state_next_tmp_.x_vref = state_next_.segment<NX_VREF>(NX_POSE);
        state_next_tmp_.x_sid = state_next_.tail<NX_SID>();

        state_tmp_ = state_next_tmp_;
    }
}
