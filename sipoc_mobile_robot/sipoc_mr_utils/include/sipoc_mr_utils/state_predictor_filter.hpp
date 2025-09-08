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

#ifndef SIPOC_MR_UTILS__STATE_PREDICTOR_FILTER_HPP_
#define SIPOC_MR_UTILS__STATE_PREDICTOR_FILTER_HPP_

#include <Eigen/Dense>
#include <boost/circular_buffer.hpp>

namespace sipoc_mr_utils
{

    class StatePredictorFilter
    {

    private:
        static constexpr int NX_POSE = 3;
        static constexpr int NX_VREF = 2;
        static constexpr int NX_SID = 4;
        static constexpr int NU_A = 2;

    public:
        /**
         *   P - Estimate error covariance
         *
         ***********************************************************
         *   Prediction:
         *   A - System dynamics matrix
         *   C - Output matrix
         *   D - Feedforward matrix
         *   x-pose_{k+1} = \phi(x-pose_{k}, C @ x-sid_{k} + D @ x-vref_{k})
         *   x-sid_{k+1}  = A @ x-sid _k + B @ x-vref_k (+ \omega_k)
         *   x-vref_{k+1} = x-vref_k + a * dt
         *
         ***********************************************************
         *   Filtering:
         *   Q - Process noise covariance
         *   R - Measurement noise covariance
         *   The measurement delay is one time step.
         *   \omega_k    \sim \mathcal{N}(0, Q)
         *   \nu_k       \sim \mathcal{N}(0, R)
         *
         *   v-meas_k    = C x-sid_{k-1} + D v-ref_{k-1} + \nu_k
         */

        struct FullState
        {
            Eigen::Vector<double, NX_POSE> x_pose;
            Eigen::Vector<double, NX_SID> x_sid;
            Eigen::Vector<double, NX_VREF> x_vref;
        };

        StatePredictorFilter(
            const Eigen::Matrix<double, NX_SID, NX_SID> &A,
            const Eigen::Matrix<double, NX_SID, NX_VREF> &B,
            const Eigen::Matrix<double, NX_VREF, NX_SID> &C,
            const Eigen::Matrix<double, NX_VREF, NX_VREF> &D,
            const Eigen::Matrix<double, NX_SID, NX_SID> &Q,
            const Eigen::Matrix<double, NX_VREF, NX_VREF> &R,
            const Eigen::Matrix<double, NX_SID, NX_SID> &P0,
            double dt);

        /**
         * Initialize the filter with initial states as zero.
         */
        void init();

        /**
         * Initialize the filter with a guess for initial states.
         */
        void init(const Eigen::Vector<double, NX_SID> &x_sid, const Eigen::Vector<double, NX_VREF> &x_vref);

        void update(const Eigen::Vector<double, NX_POSE> &pose_measured, const Eigen::Vector<double, NX_VREF> &v_measured, const Eigen::Vector<double, NX_VREF> &vref);

        void predictOneStepWoSysID(const Eigen::Vector<double, NU_A> &u);
        void predictOneStep(const Eigen::Vector<double, NU_A> &u);

        void predictOCPInitialUncertainty(Eigen::Matrix<double, NX_SID, NX_SID> &Sigma0);
        void predictOCPInitialState(const boost::circular_buffer<Eigen::Vector<double, NU_A>> &cb_u, Eigen::Vector<double, NX_POSE + NX_SID + NX_VREF> &ocp_init_state_concat, bool if_incorporate_sys_id);

        inline unsigned int NumStepDelayComp() const
        {
            return n_step_delay_comp_;
        }

        inline unsigned int NumStepDelayMeas() const
        {
            return n_step_delay_meas_;
        }

    private:
        // Matrices for the Kalman filter
        Eigen::Matrix<double, NX_SID, NX_SID> A_;
        Eigen::Matrix<double, NX_SID, NX_SID> Ac_;
        Eigen::Matrix<double, NX_SID, NX_VREF> B_;
        Eigen::Matrix<double, NX_SID, NX_VREF> Bc_;
        Eigen::Matrix<double, NX_VREF, NX_SID> C_;
        Eigen::Matrix<double, NX_VREF, NX_VREF> D_;
        Eigen::Matrix<double, NX_SID, NX_SID> Q_;
        Eigen::Matrix<double, NX_VREF, NX_VREF> R_;
        Eigen::Matrix<double, NX_SID, NX_SID> P_;
        Eigen::Matrix<double, NX_SID, NX_SID> P0_;
        Eigen::Matrix<double, NX_SID, NX_VREF> K_;
        double dt_;

        unsigned int n_step_delay_comp_ = 1;
        unsigned int n_step_delay_meas_ = 1;

        bool initialized_;

        FullState state_tm_delay_meas_;
        FullState state_tmp_, state_next_tmp_;
        Eigen::Vector<double, NX_SID> x_sid_filtered_tmp_;
        Eigen::Vector<double, NX_POSE + NX_VREF + NX_SID> rk4_k0_, rk4_k1_, rk4_k2_, rk4_k3_, rk4_tmp_, state_next_, initial_state_;
    };
}

#endif // SIPOC_MR_UTILS__STATE_PREDICTOR_FILTER_HPP_
