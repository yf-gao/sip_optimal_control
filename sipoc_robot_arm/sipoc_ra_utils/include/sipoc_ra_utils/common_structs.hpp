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

#ifndef SIPOC_RA_SUPPORT__COMMON_STRUCTS_HPP_
#define SIPOC_RA_SUPPORT__COMMON_STRUCTS_HPP_

#include <string>

namespace sipoc_ra_utils
{
    enum SolverStatus
    {
        SUCCESS = 0,
        UL_MAX_ITERATIONS_REACHED = 1,
        UL_FAILED = 2,
        LL_FAILED = 3,
        SKIPPED_TARGET_COLLISION = 4,
        SKIPPED_INV_KIN_FAILED = 5,
        SUCCESS_SLACK_NONZERO = 6,
    };

    struct TimingCollDetection
    {
        double time_coal = 0.;
        double time_min_element = 0.;
        double time_update = 0.;
    };

    struct InfoIterSIPSolver
    {
        struct TimingCollDetection time_detailed_collision_detection;
        double l_infinity_step = 0.0;
        double time_all_comps = 0.0;
        double time_collision_detection = 0.0;
        double time_ocp_solver = 0.0;
        double time_constr_linearization = 0.0;
        int num_iter = -1;
    };

    struct ConfigCollisionElementShape
    {
        // NOTE: The collision element is at the origin of the link frame.
        double padding = 0.08f;
        std::string link_name = "default_link";
        struct ConfigConvexHullShape
        {
            std::string mesh_path;
            double scale = 1.0f;
        } convex;
    };

    struct ConfigOCP
    {
        double weight_joint_angle = 1.0;
        double weight_joint_vel = 1.0;
        double weight_joint_acc = 1e-4;
        double weight_joint_angle_terminal = 10.0;
        double weight_joint_vel_terminal = 10.0;
        double delta_t = 0.1;
        unsigned int n_hrzn = 30;
        unsigned int num_max_constr = 30;
        unsigned int nx = 8;
        unsigned int nu = 8;
        bool joint_vel_as_state = true;             // If true, the joint velocity is included in the state vector.
        bool terminal_equality_constraints = false; // If true, the terminal state is subject to equality constraints.
    };

    struct ConfigVelAccBounds
    {
        double max_joint_vel = 3.0;
        double max_joint_acc = 10.0;
        double max_lin_vel = 1.0;
        double max_lin_acc = 3.0;
        double max_spline_acc = 1.0;
        double velopt_factor = 0.95;
    };

    struct ConfigTrajectorySIPSolver
    {
        double tol = 1e-6;
        unsigned int num_iterations = 100;
        bool use_cropped_octomap = false;
        struct ConfigOCP ocp;
        struct ConfigVelAccBounds bounds;
    };
}

#endif // SIPOC_RA_SUPPORT__COMMON_STRUCTS_HPP_
