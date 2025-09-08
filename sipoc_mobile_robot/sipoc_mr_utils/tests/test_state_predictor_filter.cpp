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
// Authors: Yunfan Gao
//

#include "sipoc_mr_utils/state_predictor_filter.hpp"

#include <boost/circular_buffer.hpp>
#include <Eigen/Dense>

#include <iostream>
#include <vector>
#include <random>

using namespace sipoc_mr_utils;

int main(int argc, char *argv[])
{

    constexpr int NX_SID = 4;
    constexpr int NX_VREF = 2;

    Eigen::Matrix<double, NX_SID, NX_SID> A;
    Eigen::Matrix<double, NX_SID, NX_VREF> B;
    Eigen::Matrix<double, NX_VREF, NX_SID> C;
    Eigen::Matrix<double, NX_VREF, NX_VREF> D;
    Eigen::Matrix<double, NX_SID, NX_SID> Q;
    Eigen::Matrix<double, NX_VREF, NX_VREF> R;
    Eigen::Matrix<double, NX_SID, NX_SID> P;

    A << 5.47156136e-01, 2.94260985e-02, 1.18485416e-01, 4.03549794e-02,
        -1.77330039e-01, 6.76329923e-01, -6.83086489e-04, -1.33241585e-01,
        -1.00597976e+00, -6.42665586e-02, 9.45021492e-01, 1.08375645e-01,
        -2.75956314e-01, 5.80909335e-01, -1.07056137e-01, 9.21563495e-01;
    B << -0.10598989, 0.02866782,
        -0.0474502, -0.0839996,
        -0.23548087, 0.02802434,
        -0.05453232, 0.16877108;
    C << -3.93718367, -0.6468206, -0.59703854, 0.27526566,
        0.68802978, -2.56421156, 0.11664315, 0.61027594;
    D << 0.06895757, 0.00225147,
        0.12305827, 0.27739907;

    R << 1.42512526e-03, 2.07801177e-04,
        2.07801177e-04, 5.55811136e-03;
    Q << 7.52924843e-05, -4.88510754e-06, 4.96664615e-06, 1.45580301e-04,
        -4.88510754e-06, 2.37377547e-04, 6.77252242e-05, -1.36925064e-04,
        4.96664615e-06, 6.77252242e-05, 1.95001551e-04, 4.33295159e-05,
        1.45580301e-04, -1.36925064e-04, 4.33295159e-05, 9.13550832e-04;

    P = Eigen::MatrixXd::Identity(NX_SID, NX_SID); // Initial estimate error covariance
    std::cout << "A: \n"
              << A << std::endl;
    std::cout << "B: \n"
              << B << std::endl;
    std::cout << "C: \n"
              << C << std::endl;
    std::cout << "D: \n"
              << D << std::endl;
    std::cout << "Q: \n"
              << Q << std::endl;
    std::cout << "R: \n"
              << R << std::endl;
    std::cout << "P: \n"
              << P << std::endl;

    // Construct the filter
    double dt = 0.05;
    StatePredictorFilter filter(A, B, C, D, Q, R, P, dt);

    filter.init();

    Eigen::Vector2d u;
    u << 0.2, -0.4;
    boost::circular_buffer<Eigen::Vector2d> cb_u(5);
    Eigen::Vector<double, 3> pose, pose_measured;
    pose << 0.1, 0.2, 0.3;
    cb_u.push_back(u);
    cb_u.push_back(u);
    std::cout << "size=" << cb_u.size() << ", capacity=" << cb_u.capacity() << std::endl;

    unsigned int num_steps = 30;
    std::vector<Eigen::Vector2d> vec_velocities(num_steps);
    vec_velocities[0] << 0.2, 0.5;
    for (unsigned int i = 1; i < num_steps; ++i)
    {
        vec_velocities[i] = vec_velocities[i - 1] + u * dt;
    }

    std::random_device rd;
    std::default_random_engine e2(rd());
    std::uniform_real_distribution<> dist(-0.05, 0.05);

    // Feed measurements into filter, output estimated states
    Eigen::Vector2d vel_measured;
    Eigen::Vector2d vel_measured_last = Eigen::Vector2d::Zero();
    Eigen::Vector<double, 9> ocp_init_state;

    for (long unsigned int i = 0; i < num_steps; i++)
    {
        vel_measured << vec_velocities[i].x() + dist(e2), vec_velocities[i].y() + dist(e2);
        pose_measured << pose.x() + dist(e2), pose.y() + dist(e2), pose.z() + dist(e2);
        filter.update(pose_measured, vel_measured, vel_measured_last);
        filter.predictOCPInitialState(cb_u, ocp_init_state, true);
        vel_measured_last = vel_measured;
        std::cout << "i = " << i << ", ocp_init_state: " << ocp_init_state.transpose() << std::endl;
    }

    return 0;
}
