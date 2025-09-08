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

#include "sipoc_mr_solver/capsule_nominal_controller.hpp"

#include <cmath>

namespace sipoc_mr_solver
{

    void CapsuleNominalController::checkOCPSameDimsAsExported()
    {
        if (params_.ocp_dims.n_hrzn != DIFF_DRIVE_ROBOT_N)
        {
            std::cout << "params_.ocp_dims.n_hrzn: " << params_.ocp_dims.n_hrzn << std::endl;
            std::cout << "DIFF_DRIVE_ROBOT_N: " << DIFF_DRIVE_ROBOT_N << std::endl;
            throw std::runtime_error("not same number of shooting nodes");
        }
        if (params_.shooting_nodes.size() != params_.ocp_dims.n_hrzn + 1)
        {
            throw std::runtime_error("size of shooting nodes != params_.ocp_dims.n_hrzn+1");
        }
        if (params_.ocp_iters.max_constr_num != DIFF_DRIVE_ROBOT_NG)
        {
            std::cout << "params_.ocp_iters.max_constr_num: " << params_.ocp_iters.max_constr_num << std::endl;
            std::cout << "DIFF_DRIVE_ROBOT_NG: " << DIFF_DRIVE_ROBOT_NG << std::endl;
            throw std::runtime_error("not same number of inequality constraints");
        }
        if (params_.ocp_dims.nx != DIFF_DRIVE_ROBOT_NX)
        {
            std::cout << "params_.ocp_dims.nx: " << params_.ocp_dims.nx << std::endl;
            std::cout << "DIFF_DRIVE_ROBOT_NX: " << DIFF_DRIVE_ROBOT_NX << std::endl;
            throw std::runtime_error("not same number of states");
        }
    }

    void CapsuleNominalController::createAcadosSolver()
    {
        acados_ocp_capsule_ = diff_drive_robot_acados_create_capsule();
        double *new_time_steps = NULL;
        int status = diff_drive_robot_acados_create_with_discretization(
            acados_ocp_capsule_, DIFF_DRIVE_ROBOT_N, new_time_steps);
        if (status)
        {
            exit(1);
        }
        nlp_config_ = diff_drive_robot_acados_get_nlp_config(acados_ocp_capsule_);
        nlp_dims_ = diff_drive_robot_acados_get_nlp_dims(acados_ocp_capsule_);
        nlp_in_ = diff_drive_robot_acados_get_nlp_in(acados_ocp_capsule_);
        nlp_out_ = diff_drive_robot_acados_get_nlp_out(acados_ocp_capsule_);
        nlp_solver_ = diff_drive_robot_acados_get_nlp_solver(acados_ocp_capsule_);
        nlp_opts_ = diff_drive_robot_acados_get_nlp_opts(acados_ocp_capsule_);
    }

    void CapsuleNominalController::configureLowerLevelSolver()
    {
        ptr_sd_map_ = std::make_unique<sipoc_mr_utils::SignedDistanceMap>(params_.sdf, params_.robot);
        ptr_ll_solver_ = std::make_unique<sipoc_mr_utils::DistanceMinimizerPointToLine<double>>(params_.robot);
    }

    bool CapsuleNominalController::solveOCPSubproblem(int idx_iter)
    {
        int nx = params_.ocp_dims.nx;
        int max_constr_num = params_.ocp_iters.max_constr_num;
        int solver_status = true;
        int constr_update_idx, num_imposed_constr;
        // NOTE: Hard-coded parameter: threshold for replacing / adding the constraints
        double thr_obs_replace = std::max(0.5, static_cast<double>(20 - idx_iter)) * ptr_sd_map_->get_grid_size();

        auto ll_start_time = std::chrono::high_resolution_clock::now();
        for (int idx_hrzn = 1; idx_hrzn <= params_.ocp_dims.n_hrzn; ++idx_hrzn)
        {
            manif::SE2<double> pose_at_tk(solPxAtTk(idx_hrzn), solPyAtTk(idx_hrzn), solYawAtTk(idx_hrzn));
            auto update_obs_subset_start_time = std::chrono::high_resolution_clock::now();
            ptr_sd_map_->argMinDistanceOverGamma(pose_at_tk, params_.ifBilinearInterpArgMinGamma, gamma_gs_result_);
            ptr_sd_map_->getClosestObsAtGamma(pose_at_tk, gamma_gs_result_.minimizer[0], temp_vector2d_);
            auto update_obs_subset_end_time = std::chrono::high_resolution_clock::now();
            elapsed_time_.update_obs_subset += std::chrono::duration_cast<std::chrono::microseconds>(update_obs_subset_end_time - update_obs_subset_start_time).count();
            constr_update_idx = num_constr_each_time_step_[idx_hrzn];
            for (int ii = 0; ii < num_constr_each_time_step_[idx_hrzn]; ++ii)
            {
                if ((obs_gamma_star_sol_[idx_hrzn][ii].obs_coords - temp_vector2d_).norm() <= thr_obs_replace)
                {
                    constr_update_idx = ii;
                    break;
                }
            }
            if (constr_update_idx == num_constr_each_time_step_[idx_hrzn])
            {
                num_constr_each_time_step_[idx_hrzn] += 1;
                if (num_constr_each_time_step_[idx_hrzn] > max_constr_num)
                {
                    constr_update_idx = getObsWithMaximumGVal(idx_hrzn);
                    if (constr_update_idx < 0)
                    {
                        DEBUG_MSG(" number of constraints exceeds maximum " << max_constr_num);
                        return false;
                    }
                    num_constr_each_time_step_[idx_hrzn] = max_constr_num;
                }
            }
            obs_gamma_star_sol_[idx_hrzn][constr_update_idx].obs_coords = temp_vector2d_;
            num_imposed_constr = num_constr_each_time_step_[idx_hrzn];
            updateGammaAndSepPlane(idx_hrzn, num_imposed_constr);
            updateNominalOCPLinearConstraints(idx_hrzn, params_.robot.robot_capsule_radius);
        }
        auto ul_start_time = std::chrono::high_resolution_clock::now();
        solver_status = diff_drive_robot_acados_solve(acados_ocp_capsule_);
        if (solver_status == ACADOS_SUCCESS || solver_status == ACADOS_MAXITER)
        {
            exportAcadosSolToLocalArray(x_sol_new_iter_, u_sol_new_iter_);
        }
        auto ul_end_time = std::chrono::high_resolution_clock::now();
        elapsed_time_.lower_level += std::chrono::duration_cast<std::chrono::microseconds>(ul_start_time - ll_start_time).count();
        elapsed_time_.upper_level += std::chrono::duration_cast<std::chrono::microseconds>(ul_end_time - ul_start_time).count();
        return (solver_status == ACADOS_SUCCESS || solver_status == ACADOS_MAXITER);
    }

    int CapsuleNominalController::getObsWithMaximumGVal(int idx_hrzn)
    {
        ocp_nlp_get_at_stage(nlp_solver_, idx_hrzn, "C", Cg_);
        ocp_nlp_get_at_stage(nlp_solver_, idx_hrzn, "lg", lg_);
        Eigen::Matrix<double, DIFF_DRIVE_ROBOT_NG, DIFF_DRIVE_ROBOT_NX> eigen_Cg = Eigen::Map<Eigen::Matrix<double, DIFF_DRIVE_ROBOT_NG, DIFF_DRIVE_ROBOT_NX, Eigen::ColMajor>>(Cg_);
        Eigen::Vector<double, DIFF_DRIVE_ROBOT_NG> eigen_lg = Eigen::Map<Eigen::Vector<double, DIFF_DRIVE_ROBOT_NG>>(lg_);
        Eigen::Vector<double, DIFF_DRIVE_ROBOT_NX> eigen_x_sol = Eigen::Map<Eigen::Vector<double, DIFF_DRIVE_ROBOT_NX>>(x_sol_ + idx_hrzn * params_.ocp_dims.nx);
        Eigen::Vector<double, DIFF_DRIVE_ROBOT_NG> g_val = eigen_Cg * eigen_x_sol - eigen_lg;
        Eigen::Index maxIndex;
        double max_g = g_val.maxCoeff(&maxIndex);
        if (max_g < 0)
        {
            return -1;
        }
        else
        {
            return maxIndex;
        }
    }

    bool CapsuleNominalController::solveEndpointGammaConstrainedSubproblem()
    {
        int num_imposed_constr = 2;
        for (int idx_hrzn = 1; idx_hrzn <= params_.ocp_dims.n_hrzn; ++idx_hrzn)
        {
            num_constr_each_time_step_[idx_hrzn] = num_imposed_constr;
            findClosestObsGivenGamma(0., 0, idx_hrzn);
            findClosestObsGivenGamma(params_.robot.robot_capsule_half_length, 1, idx_hrzn);
            updateNominalOCPLinearConstraints(idx_hrzn, params_.robot.robot_capsule_radius);
        }
        auto ul_start_time = std::chrono::high_resolution_clock::now();
        int solver_status = diff_drive_robot_acados_solve(acados_ocp_capsule_);
        exportAcadosSolToLocalArray(x_sol_, u_sol_);
        auto ul_end_time = std::chrono::high_resolution_clock::now();
        elapsed_time_.upper_level += std::chrono::duration_cast<std::chrono::microseconds>(ul_end_time - ul_start_time).count();
        return solver_status;
    }

    void CapsuleNominalController::updateGammaAndSepPlane(int idx_hrzn, int num_imposed_constr)
    {
        Eigen::Vector2d temp_coords, temp_coords2;
        Eigen::Vector<int, 2> gamma_star_grid_indices;
        Eigen::Vector<int, 2> gamma_star_grid_clipped;
        manif::SE2<double> pose_at_tk(solPxAtTk(idx_hrzn), solPyAtTk(idx_hrzn), solYawAtTk(idx_hrzn));
        double gradient_x, gradient_y, gradient_norm;
        for (int idx_obs = 0; idx_obs < num_imposed_constr; ++idx_obs)
        {
            auto &iter = obs_gamma_star_sol_[idx_hrzn][idx_obs];
            lower_level_param_.heading_line = solYawAtTk(idx_hrzn);
            lower_level_param_.p_center_ellipsoid(0) = iter.obs_coords.x() - solPxAtTk(idx_hrzn);
            lower_level_param_.p_center_ellipsoid(1) = iter.obs_coords.y() - solPyAtTk(idx_hrzn);
            ptr_ll_solver_->argMinDistance(lower_level_param_, lower_level_result_);
            temp_coords(0) = solPxAtTk(idx_hrzn) + lower_level_result_.gamma_line * std::cos(solYawAtTk(idx_hrzn));
            temp_coords(1) = solPyAtTk(idx_hrzn) + lower_level_result_.gamma_line * std::sin(solYawAtTk(idx_hrzn));
            ptr_sd_map_->world2grid_indices_round(temp_coords, gamma_star_grid_indices);
            gamma_star_grid_clipped(0) = std::clamp(gamma_star_grid_indices.coeff(0), 1, ptr_sd_map_->get_map_width() - 2);
            gamma_star_grid_clipped(1) = std::clamp(gamma_star_grid_indices.coeff(1), 1, ptr_sd_map_->get_map_height() - 2);
            bool flag_inside_map = (gamma_star_grid_clipped(0) == gamma_star_grid_indices(0)) && (gamma_star_grid_clipped(1) == gamma_star_grid_indices(1));
            double vec_r2o_norm = (temp_coords - iter.obs_coords).norm();
            // NOTE: Hyperparameter
            if (vec_r2o_norm <= 1.5 * ptr_sd_map_->get_grid_size() || !flag_inside_map || ptr_sd_map_->ifGridOccupied(gamma_star_grid_indices, params_.sdf.map_occ_threshold))
            {
                // NOTE: When cell is occupied
                // OR When too close, keep the previous gradient
                // OR the point is outside the map
                // approximate the gradient by taking the negative value of vec_r2o will have problems when the robot is inside another obstacle
                gamma_star_grid_indices = gamma_star_grid_clipped;
                gradient_x = ptr_sd_map_->get_sd_at_grid_indices(gamma_star_grid_indices.coeff(0) + 1, gamma_star_grid_indices.coeff(1)) - ptr_sd_map_->get_sd_at_grid_indices(gamma_star_grid_indices.coeff(0) - 1, gamma_star_grid_indices.coeff(1));
                gradient_y = ptr_sd_map_->get_sd_at_grid_indices(gamma_star_grid_indices.coeff(0), gamma_star_grid_indices.coeff(1) + 1) - ptr_sd_map_->get_sd_at_grid_indices(gamma_star_grid_indices.coeff(0), gamma_star_grid_indices.coeff(1) - 1);
                gradient_norm = std::hypot(gradient_x, gradient_y);
                iter.sep_plane.x() = gradient_x / (1e-8 + gradient_norm);
                iter.sep_plane.y() = gradient_y / (1e-8 + gradient_norm);
                ptr_sd_map_->getClosestObsAtGamma(pose_at_tk, lower_level_result_.gamma_line, temp_coords2);
                if ((temp_coords2 - iter.obs_coords).norm() > 5 * ptr_sd_map_->get_grid_size())
                {
                    iter.obs_coords = temp_coords2;
                }
            }
            else
            {
                iter.sep_plane.x() = (temp_coords.x() - iter.obs_coords.x()) / vec_r2o_norm;
                iter.sep_plane.y() = (temp_coords.y() - iter.obs_coords.y()) / vec_r2o_norm;
            }
            iter.gamma_polygon.x() = lower_level_result_.gamma_line;
            iter.gamma_polygon.y() = 0.;
        }
    }
}
