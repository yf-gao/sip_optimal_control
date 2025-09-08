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

#include "sipoc_mr_solver/base_controller.hpp"

#include <Eigen/Dense>

#include <cmath>
#include <algorithm>
#include <iostream>
#include <limits>
#include <stdexcept>

namespace sipoc_mr_solver
{
    void BaseController::allocateMemory()
    {
        num_constr_each_time_step_ = (int *)malloc((params_.ocp_dims.n_hrzn + 1) * sizeof(int));
        lubx0_ = (double *)malloc(params_.ocp_dims.nx * sizeof(double));
        yref_ = (double *)malloc(params_.ocp_dims.n_hrzn * params_.ocp_dims.ny * sizeof(double));
        yref_e_ = (double *)malloc(params_.ocp_dims.nyn * sizeof(double));
        x_sol_ = (double *)malloc((params_.ocp_dims.n_hrzn + 1) * params_.ocp_dims.nx * sizeof(double));
        x_sol_new_iter_ = (double *)malloc((params_.ocp_dims.n_hrzn + 1) * params_.ocp_dims.nx * sizeof(double));
        u_sol_ = (double *)malloc(params_.ocp_dims.n_hrzn * params_.ocp_dims.nu * sizeof(double));
        u_sol_new_iter_ = (double *)malloc(params_.ocp_dims.n_hrzn * params_.ocp_dims.nu * sizeof(double));
        sl_sol_ = (double *)malloc((params_.ocp_dims.n_hrzn + 1) * params_.ocp_iters.max_constr_num * sizeof(double));
        lg_ = (double *)malloc(params_.ocp_iters.max_constr_num * sizeof(double));
        Cg_ = (double *)malloc(params_.ocp_dims.nx * params_.ocp_iters.max_constr_num * sizeof(double));
        // Lam: state-input constraint, collision-avoidance constraint, slack variable constraints
        lam_ = (double *)malloc((params_.ocp_dims.nbx + params_.ocp_dims.nbu + 2 * params_.ocp_iters.max_constr_num) * 2 * sizeof(double));
        setZerosAllArrays();
        obs_gamma_star_sol_.resize(params_.ocp_dims.n_hrzn + 1);
        for (int idx_hrzn = 0; idx_hrzn < params_.ocp_dims.n_hrzn + 1; ++idx_hrzn)
        {
            obs_gamma_star_sol_[idx_hrzn].resize(params_.ocp_iters.max_iter_num);
        }
    }

    void BaseController::setZerosAllArrays()
    {
        memset(num_constr_each_time_step_, 0, (params_.ocp_dims.n_hrzn + 1) * sizeof(int));
        memset(lubx0_, 0, params_.ocp_dims.nx * sizeof(double));
        memset(yref_, 0, params_.ocp_dims.n_hrzn * params_.ocp_dims.ny * sizeof(double));
        memset(yref_e_, 0, params_.ocp_dims.nyn * sizeof(double));
        memset(x_sol_, 0, (params_.ocp_dims.n_hrzn + 1) * params_.ocp_dims.nx * sizeof(double));
        memset(x_sol_new_iter_, 0, (params_.ocp_dims.n_hrzn + 1) * params_.ocp_dims.nx * sizeof(double));
        memset(u_sol_, 0, params_.ocp_dims.n_hrzn * params_.ocp_dims.nu * sizeof(double));
        memset(u_sol_new_iter_, 0, params_.ocp_dims.n_hrzn * params_.ocp_dims.nu * sizeof(double));
        memset(sl_sol_, 0, (params_.ocp_dims.n_hrzn + 1) * params_.ocp_iters.max_constr_num * sizeof(double));
        memset(lg_, 0, params_.ocp_iters.max_constr_num * sizeof(double));
        memset(Cg_, 0, params_.ocp_dims.nx * params_.ocp_iters.max_constr_num * sizeof(double));
        memset(lam_, 0, (params_.ocp_dims.nbx + params_.ocp_dims.nbu + 2 * params_.ocp_iters.max_constr_num) * 2 * sizeof(double));
    }

    void BaseController::deallocateMemory()
    {
        free(num_constr_each_time_step_);
        free(lubx0_);
        free(yref_);
        free(yref_e_);
        free(x_sol_);
        free(x_sol_new_iter_);
        free(u_sol_);
        free(u_sol_new_iter_);
        free(sl_sol_);
        free(lg_);
        free(Cg_);
        free(lam_);
    }

    void BaseController::checkOCPSameTimeStampsAsExported()
    {
        for (unsigned int idx = 0; idx < params_.shooting_nodes.size() - 1; ++idx)
        {
            if (std::abs(params_.shooting_nodes[idx + 1] - params_.shooting_nodes[idx] - nlp_in_->Ts[idx]) >= 1e-5)
            {
                throw std::runtime_error("Ts[idx] is not the same as params_.shooting_nodes[idx+1] - params_.shooting_nodes[idx], idx=" + std::to_string(idx));
            }
        }
    }

    void BaseController::configureOCPWeights()
    {
        double *W_0 = (double *)malloc(params_.ocp_dims.ny * params_.ocp_dims.ny * sizeof(double));
        memset(W_0, 0, params_.ocp_dims.ny * params_.ocp_dims.ny * sizeof(double));
        W_0[0 + (params_.ocp_dims.ny) * 0] = params_.ocp_weights.position;
        W_0[1 + (params_.ocp_dims.ny) * 1] = params_.ocp_weights.position;
        W_0[2 + (params_.ocp_dims.ny) * 2] = params_.ocp_weights.heading;
        W_0[3 + (params_.ocp_dims.ny) * 3] = params_.ocp_weights.forward_velocity;
        W_0[4 + (params_.ocp_dims.ny) * 4] = params_.ocp_weights.angular_velocity;
        W_0[5 + (params_.ocp_dims.ny) * 5] = params_.ocp_weights.forward_acceleration;
        W_0[6 + (params_.ocp_dims.ny) * 6] = params_.ocp_weights.angular_acceleration;
        ocp_nlp_cost_model_set(nlp_config_, nlp_dims_, nlp_in_, 0, "W", W_0);
        free(W_0);

        double *W = (double *)malloc(params_.ocp_dims.ny * params_.ocp_dims.ny * sizeof(double));
        memset(W, 0, params_.ocp_dims.ny * params_.ocp_dims.ny * sizeof(double));
        W[0 + (params_.ocp_dims.ny) * 0] = params_.ocp_weights.position;
        W[1 + (params_.ocp_dims.ny) * 1] = params_.ocp_weights.position;
        W[2 + (params_.ocp_dims.ny) * 2] = params_.ocp_weights.heading;
        W[3 + (params_.ocp_dims.ny) * 3] = params_.ocp_weights.forward_velocity;
        W[4 + (params_.ocp_dims.ny) * 4] = params_.ocp_weights.angular_velocity;
        W[5 + (params_.ocp_dims.ny) * 5] = params_.ocp_weights.forward_acceleration;
        W[6 + (params_.ocp_dims.ny) * 6] = params_.ocp_weights.angular_acceleration;
        for (int idx_hrzn = 1; idx_hrzn < params_.ocp_dims.n_hrzn; ++idx_hrzn)
        {
            ocp_nlp_cost_model_set(nlp_config_, nlp_dims_, nlp_in_, idx_hrzn, "W", W);
        }
        free(W);

        double *W_e = (double *)malloc(params_.ocp_dims.nyn * params_.ocp_dims.nyn * sizeof(double));
        memset(W_e, 0, params_.ocp_dims.nyn * params_.ocp_dims.nyn * sizeof(double));
        W_e[0 + (params_.ocp_dims.nyn) * 0] = params_.ocp_weights.position * params_.shooting_nodes.back();
        W_e[1 + (params_.ocp_dims.nyn) * 1] = params_.ocp_weights.position * params_.shooting_nodes.back();
        W_e[2 + (params_.ocp_dims.nyn) * 2] = params_.ocp_weights.heading * params_.shooting_nodes.back();
        W_e[3 + (params_.ocp_dims.nyn) * 3] = params_.ocp_weights.forward_velocity * params_.shooting_nodes.back();
        W_e[4 + (params_.ocp_dims.nyn) * 4] = params_.ocp_weights.angular_velocity * params_.shooting_nodes.back();
        ocp_nlp_cost_model_set(nlp_config_, nlp_dims_, nlp_in_, params_.ocp_dims.n_hrzn, "W", W_e);
        free(W_e);

        double *l1_cost = (double *)malloc(params_.ocp_iters.max_constr_num * sizeof(double));
        double *l2_cost = (double *)malloc(params_.ocp_iters.max_constr_num * sizeof(double));
        std::fill_n(l1_cost, params_.ocp_iters.max_constr_num, params_.ocp_weights.slack_l1_cost);
        std::fill_n(l2_cost, params_.ocp_iters.max_constr_num, params_.ocp_weights.slack_l2_cost);
        for (int idx_hrzn = 1; idx_hrzn <= params_.ocp_dims.n_hrzn; ++idx_hrzn)
        {
            ocp_nlp_cost_model_set(nlp_config_, nlp_dims_, nlp_in_, idx_hrzn, "zl", l1_cost);
            ocp_nlp_cost_model_set(nlp_config_, nlp_dims_, nlp_in_, idx_hrzn, "zu", l1_cost);
            ocp_nlp_cost_model_set(nlp_config_, nlp_dims_, nlp_in_, idx_hrzn, "Zl", l2_cost);
            ocp_nlp_cost_model_set(nlp_config_, nlp_dims_, nlp_in_, idx_hrzn, "Zu", l2_cost);
        }
        free(l1_cost);
        free(l2_cost);
    }

    void BaseController::configureOCPVelAccBounds()
    {
        // NOTE: The constraints are set on the command velocities, the system of which is certain even for the robust controller.
        // Control input
        double *lubu = (double *)malloc(2 * params_.ocp_dims.nbu * sizeof(double));
        double *lbu = lubu;
        double *ubu = lubu + params_.ocp_dims.nbu;
        lbu[0] = params_.ocp_bounds.min_forward_acceleration;
        ubu[0] = params_.ocp_bounds.max_forward_acceleration;
        lbu[1] = -params_.ocp_bounds.max_angular_acceleration;
        ubu[1] = params_.ocp_bounds.max_angular_acceleration;
        ocp_nlp_constraints_model_set(nlp_config_, nlp_dims_, nlp_in_, nlp_out_, 0, "lbu", lbu);
        ocp_nlp_constraints_model_set(nlp_config_, nlp_dims_, nlp_in_, nlp_out_, 0, "ubu", ubu);

        // Middle stages
        double *lubx = (double *)malloc(2 * params_.ocp_dims.nbx * sizeof(double));
        double *lbx = lubx;
        double *ubx = lubx + params_.ocp_dims.nbx;
        lbu[0] = params_.ocp_bounds.min_forward_acceleration;
        ubu[0] = params_.ocp_bounds.max_forward_acceleration;
        lbu[1] = -params_.ocp_bounds.max_angular_acceleration;
        ubu[1] = params_.ocp_bounds.max_angular_acceleration;

        lbx[0] = params_.ocp_bounds.min_forward_velocity;
        ubx[0] = params_.ocp_bounds.max_forward_velocity;
        lbx[1] = -params_.ocp_bounds.max_angular_velocity;
        ubx[1] = params_.ocp_bounds.max_angular_velocity;

        for (int idx_hrzn = 1; idx_hrzn < params_.ocp_dims.n_hrzn; ++idx_hrzn)
        {
            ocp_nlp_constraints_model_set(nlp_config_, nlp_dims_, nlp_in_, nlp_out_, idx_hrzn, "lbu", lbu);
            ocp_nlp_constraints_model_set(nlp_config_, nlp_dims_, nlp_in_, nlp_out_, idx_hrzn, "ubu", ubu);
            ocp_nlp_constraints_model_set(nlp_config_, nlp_dims_, nlp_in_, nlp_out_, idx_hrzn, "lbx", lbx);
            ocp_nlp_constraints_model_set(nlp_config_, nlp_dims_, nlp_in_, nlp_out_, idx_hrzn, "ubx", ubx);
        }
        free(lubu);
        free(lubx);

        // Last stage
        double *lubx_e = (double *)malloc(2 * params_.ocp_dims.nbxe * sizeof(double));
        double *lbx_e = lubx_e;
        double *ubx_e = lubx_e + params_.ocp_dims.nbxe;
        lbx_e[0] = -params_.ocp_bounds.max_forward_velocity_e;
        lbx_e[1] = -params_.ocp_bounds.max_angular_velocity_e;
        ubx_e[0] = params_.ocp_bounds.max_forward_velocity_e;
        ubx_e[1] = params_.ocp_bounds.max_angular_velocity_e;

        ocp_nlp_constraints_model_set(nlp_config_, nlp_dims_, nlp_in_, nlp_out_, params_.ocp_dims.n_hrzn, "lbx", lbx_e);
        ocp_nlp_constraints_model_set(nlp_config_, nlp_dims_, nlp_in_, nlp_out_, params_.ocp_dims.n_hrzn, "ubx", ubx_e);
        free(lubx_e);
    }

    void BaseController::updateReferenceTrajectory(const double *yref, const double *yref_e)
    {
        std::memcpy(yref_, yref, params_.ocp_dims.n_hrzn * params_.ocp_dims.ny * sizeof(double));
        std::memcpy(yref_e_, yref_e, params_.ocp_dims.nyn * sizeof(double));
        ocp_nlp_cost_model_set(nlp_config_, nlp_dims_, nlp_in_, 0, "yref", yref_);
        for (int idx_hrzn = 1; idx_hrzn < params_.ocp_dims.n_hrzn; idx_hrzn++)
        {
            ocp_nlp_cost_model_set(nlp_config_, nlp_dims_, nlp_in_, idx_hrzn, "yref", yref_ + idx_hrzn * params_.ocp_dims.ny);
        }
        ocp_nlp_cost_model_set(nlp_config_, nlp_dims_, nlp_in_, params_.ocp_dims.n_hrzn, "yref", yref_e_);
        return;
    }

    void BaseController::updateInitialState(const manif::SE2<double> &current_pose, const manif::SE2Tangent<double> &current_ref_vel)
    {
        // x0
        lubx0_[0] = current_pose.x();
        lubx0_[1] = current_pose.y();
        lubx0_[2] = current_pose.angle();
        lubx0_[3] = current_ref_vel.x();
        lubx0_[4] = current_ref_vel.angle();

        ocp_nlp_constraints_model_set(nlp_config_, nlp_dims_, nlp_in_, nlp_out_, 0, "lbx", lubx0_);
        ocp_nlp_constraints_model_set(nlp_config_, nlp_dims_, nlp_in_, nlp_out_, 0, "ubx", lubx0_);
    }

    void BaseController::updateInitialState(const Eigen::Vector<double, 9> &full_state)
    {
        // x0
        std::copy(full_state.data(), full_state.data() + 9, lubx0_);
        ocp_nlp_constraints_model_set(nlp_config_, nlp_dims_, nlp_in_, nlp_out_, 0, "lbx", lubx0_);
        ocp_nlp_constraints_model_set(nlp_config_, nlp_dims_, nlp_in_, nlp_out_, 0, "ubx", lubx0_);
    }

    void BaseController::initializeOCPByReference()
    {
        // Initialize with values in yref_
        // NOTE: The state is given by [x, x-sid] and the yref is given by [x, u]
        unsigned int nx_in_yref = params_.ocp_dims.ny - params_.ocp_dims.nu;
        memset(x_sol_, 0, (params_.ocp_dims.n_hrzn + 1) * params_.ocp_dims.nx * sizeof(double));
        unsigned int size_x_memcpy = sizeof(double) * nx_in_yref;
        unsigned int size_u_memcpy = sizeof(double) * params_.ocp_dims.nu;
        for (int idx_hrzn = 0; idx_hrzn < params_.ocp_dims.n_hrzn; ++idx_hrzn)
        {
            std::memcpy(x_sol_ + idx_hrzn * params_.ocp_dims.nx, yref_ + idx_hrzn * params_.ocp_dims.ny, size_x_memcpy);
            std::memcpy(u_sol_ + idx_hrzn * params_.ocp_dims.nu, yref_ + idx_hrzn * params_.ocp_dims.ny + nx_in_yref, size_u_memcpy);
            ocp_nlp_out_set(nlp_config_, nlp_dims_, nlp_out_, nlp_in_, idx_hrzn, "x", x_sol_ + idx_hrzn * params_.ocp_dims.nx);
            ocp_nlp_out_set(nlp_config_, nlp_dims_, nlp_out_, nlp_in_, idx_hrzn, "u", u_sol_ + idx_hrzn * params_.ocp_dims.nu);
        }
        std::memcpy(x_sol_ + params_.ocp_dims.n_hrzn * params_.ocp_dims.nx, yref_e_, size_x_memcpy);
        ocp_nlp_out_set(nlp_config_, nlp_dims_, nlp_out_, nlp_in_, params_.ocp_dims.n_hrzn, "x", x_sol_ + params_.ocp_dims.n_hrzn * params_.ocp_dims.nx);
    }

    void BaseController::updateNominalOCPLinearConstraints(int idx_hrzn, double padding_radius)
    {
        unsigned int max_constr_num = params_.ocp_iters.max_constr_num;
        memset(Cg_, 0, params_.ocp_dims.nx * params_.ocp_iters.max_constr_num * sizeof(double));
        std::fill(lg_, lg_ + max_constr_num, -1e3);
        Eigen::Matrix2d rot_mat, rot_mat_derivative;
        rot_mat << std::cos(x_sol_[idx_hrzn * params_.ocp_dims.nx + 2]), -std::sin(x_sol_[idx_hrzn * params_.ocp_dims.nx + 2]),
            std::sin(x_sol_[idx_hrzn * params_.ocp_dims.nx + 2]), std::cos(x_sol_[idx_hrzn * params_.ocp_dims.nx + 2]);
        rot_mat_derivative << -std::sin(x_sol_[idx_hrzn * params_.ocp_dims.nx + 2]), -std::cos(x_sol_[idx_hrzn * params_.ocp_dims.nx + 2]),
            std::cos(x_sol_[idx_hrzn * params_.ocp_dims.nx + 2]), -std::sin(x_sol_[idx_hrzn * params_.ocp_dims.nx + 2]);
        for (int idx_constr = 0; idx_constr < num_constr_each_time_step_[idx_hrzn]; ++idx_constr)
        {
            const auto &sol = obs_gamma_star_sol_[idx_hrzn][idx_constr];
            Cg_[idx_constr + max_constr_num * 0] = sol.sep_plane.x();
            Cg_[idx_constr + max_constr_num * 1] = sol.sep_plane.y();
            Cg_[idx_constr + max_constr_num * 2] = sol.sep_plane.dot(rot_mat_derivative * sol.gamma_polygon);
            lg_[idx_constr] = -sol.sep_plane.dot((rot_mat - x_sol_[idx_hrzn * params_.ocp_dims.nx + 2] * rot_mat_derivative) * sol.gamma_polygon - sol.obs_coords) + padding_radius;
        }
        ocp_nlp_constraints_model_set(nlp_config_, nlp_dims_, nlp_in_, nlp_out_, idx_hrzn, "lg", lg_);
        ocp_nlp_constraints_model_set(nlp_config_, nlp_dims_, nlp_in_, nlp_out_, idx_hrzn, "C", Cg_);
    }

    sipoc_mr_utils::SIPSolverStatus BaseController::solveOCP(bool if_reset_num_constr)
    {
        if (if_reset_num_constr)
        {
            std::fill_n(num_constr_each_time_step_, params_.ocp_dims.n_hrzn + 1, 0);
        }
        bool status_subproblem;
        for (int idx_iter = 0; idx_iter < params_.ocp_iters.max_iter_num; ++idx_iter)
        {
            status_subproblem = solveOCPSubproblem(idx_iter);
            if (!status_subproblem)
            {
                DEBUG_MSG("Failed at #iter= " << idx_iter);
                num_iter_converg_ = std::numeric_limits<int>::max();
                return sipoc_mr_utils::SIPSolverStatus::acadosFail;
            }
            if (ifConverged())
            {
                DEBUG_MSG("Converge at #iter= " << idx_iter);
                num_iter_converg_ = idx_iter;
                return sipoc_mr_utils::SIPSolverStatus::success;
            }
            std::memcpy(x_sol_, x_sol_new_iter_, (params_.ocp_dims.n_hrzn + 1) * params_.ocp_dims.nx * sizeof(double));
            std::memcpy(u_sol_, u_sol_new_iter_, params_.ocp_dims.n_hrzn * params_.ocp_dims.nu * sizeof(double));
        }
        DEBUG_MSG("Max iteration reached.");
        num_iter_converg_ = params_.ocp_iters.max_iter_num + 1;
        return sipoc_mr_utils::SIPSolverStatus::reachMaxIters;
    }

    void BaseController::findClosestObsGivenGamma(double gamma_val, unsigned int idx_constr, unsigned int idx_hrzn)
    {
        manif::SE2<double> pose_at_tk(solPxAtTk(idx_hrzn), solPyAtTk(idx_hrzn), solYawAtTk(idx_hrzn));
        unsigned int index_array = idx_hrzn * params_.ocp_iters.max_constr_num + idx_constr;

        Eigen::Vector2d temp_coords;
        double gradient_x, gradient_y;

        obs_gamma_star_sol_[idx_hrzn][idx_constr].gamma_polygon.x() = gamma_val;
        obs_gamma_star_sol_[idx_hrzn][idx_constr].gamma_polygon.y() = 0.0;

        ptr_sd_map_->getClosestObsAtGamma(pose_at_tk, gamma_val, temp_coords);
        gradient_x = solPxAtTk(idx_hrzn) + gamma_val * pose_at_tk.real() - temp_coords.coeff(0);
        gradient_y = solPyAtTk(idx_hrzn) + gamma_val * pose_at_tk.imag() - temp_coords.coeff(1);
        obs_gamma_star_sol_[idx_hrzn][idx_constr].obs_coords = temp_coords;
        obs_gamma_star_sol_[idx_hrzn][idx_constr].sep_plane.x() = gradient_x;
        obs_gamma_star_sol_[idx_hrzn][idx_constr].sep_plane.y() = gradient_y;
        obs_gamma_star_sol_[idx_hrzn][idx_constr].sep_plane.normalize();
    }

    bool BaseController::ifConverged()
    {
        double max_diff = 0.;
        for (int i = 0; i < (params_.ocp_dims.n_hrzn + 1) * params_.ocp_dims.nx; ++i)
        {
            max_diff = std::max(max_diff, std::abs(x_sol_[i] - x_sol_new_iter_[i]));
        }
        for (int i = 0; i < (params_.ocp_dims.n_hrzn) * params_.ocp_dims.nu; ++i)
        {
            max_diff = std::max(max_diff, std::abs(u_sol_[i] - u_sol_new_iter_[i]));
        }
        DEBUG_MSG("max_diff = " << max_diff);
        return max_diff <= params_.ocp_iters.eps_converg;
    }

    void BaseController::shiftInputStateTrajectory(unsigned int num_steps)
    {
        // NOTE: the solution in acados internal is NOT shifted.
        if (num_steps >= params_.ocp_dims.n_hrzn)
        {
            throw std::invalid_argument("num_steps should be no greater than params_.ocp_dims.n_hrzn");
        }
        // memmove allows the objects to overlap
        std::memmove(x_sol_, x_sol_ + num_steps * params_.ocp_dims.nx, params_.ocp_dims.nx * (params_.ocp_dims.n_hrzn + 1 - num_steps) * sizeof(double));
        std::memmove(u_sol_, u_sol_ + num_steps * params_.ocp_dims.nu, params_.ocp_dims.nu * (params_.ocp_dims.n_hrzn - num_steps) * sizeof(double));
    }

    void BaseController::gatherActiveConstraintsAndShiftOneStep()
    {
        int number_active_constr;
        for (int idx_hrzn = 2; idx_hrzn < params_.ocp_dims.n_hrzn; ++idx_hrzn)
        {
            number_active_constr = 0;
            ocp_nlp_out_get(nlp_config_, nlp_dims_, nlp_out_, idx_hrzn, "lam", lam_);
            for (unsigned int idx_constr = 4; idx_constr < 4 + params_.ocp_iters.max_constr_num; ++idx_constr)
            {
                if (std::abs(lam_[idx_constr]) >= 1e-3)
                {
                    obs_gamma_star_sol_[idx_hrzn - 1][number_active_constr] = obs_gamma_star_sol_[idx_hrzn][idx_constr - 4];
                    number_active_constr += 1;
                }
            }
            num_constr_each_time_step_[idx_hrzn - 1] = number_active_constr;
        }
        // Last stage
        number_active_constr = 0;
        ocp_nlp_out_get(nlp_config_, nlp_dims_, nlp_out_, params_.ocp_dims.n_hrzn, "lam", lam_);
        for (unsigned int idx_constr = 2; idx_constr < 2 + params_.ocp_iters.max_constr_num; ++idx_constr)
        {
            if (std::abs(lam_[idx_constr]) >= 1e-3)
            {
                obs_gamma_star_sol_[params_.ocp_dims.n_hrzn - 1][number_active_constr] = obs_gamma_star_sol_[params_.ocp_dims.n_hrzn][idx_constr - 2];
                number_active_constr += 1;
            }
        }
        num_constr_each_time_step_[params_.ocp_dims.n_hrzn - 1] = number_active_constr;
        num_constr_each_time_step_[params_.ocp_dims.n_hrzn] = number_active_constr;
        std::copy(obs_gamma_star_sol_[params_.ocp_dims.n_hrzn - 1].begin(),
                  obs_gamma_star_sol_[params_.ocp_dims.n_hrzn - 1].begin() + number_active_constr,
                  obs_gamma_star_sol_[params_.ocp_dims.n_hrzn].begin());
    }

    void BaseController::obsSolAtTk(int idx_hrzn, std::vector<Eigen::Vector2d> &vec_obs)
    {
        vec_obs.clear();
        for (int idx_obs = 0; idx_obs < num_constr_each_time_step_[idx_hrzn]; ++idx_obs)
        {
            vec_obs.push_back(obs_gamma_star_sol_[idx_hrzn][idx_obs].obs_coords);
        }
        vec_obs.push_back(Eigen::Vector2d(1e3, 1e3));
    }
}
