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

#include "sipoc_mr_solver/dilated_polygon_robust_controller.hpp"

#include <cmath>

namespace sipoc_mr_solver
{

    void DilatedPolygonRobustController::checkOCPSameDimsAsExported()
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

    void DilatedPolygonRobustController::createAcadosSolver()
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

    void DilatedPolygonRobustController::configureLowerLevelSolver()
    {
        ptr_sd_map_ = std::make_unique<sipoc_mr_utils::SignedDistanceMap>(params_.sdf, params_.robot);
        ptr_ll_nominal_solver_ = std::make_unique<sipoc_mr_utils::DistanceMinimizerPointToPolygon>(params_.robot);
        ptr_ll_robust_solver_ = std::make_unique<sipoc_mr_utils::DistanceMinimizerEllipsoidToPolygon>(params_.robot);
        ptr_unc_propagation_ = std::make_unique<sipoc_mr_utils::EllipsoidalUncertaintyPropagation>(params_.ocp_dims, params_.disturbance);
        if (nlp_solver_ == NULL)
        {
            throw std::runtime_error("nlp_solver_ is an empty pointer.");
        }
        ptr_unc_propagation_->configureAcadosInterface(nlp_solver_);
    }

    bool DilatedPolygonRobustController::solveOCPSubproblem(int idx_iter)
    {
        Eigen::Vector2d temp_vector2d_2;
        int max_constr_num = params_.ocp_iters.max_constr_num;
        int solver_status = true;
        int constr_update_idx, num_imposed_constr;
        // NOTE: Hard-coded parameter: threshold for replacing / adding the constraints
        double thr_obs_replace = std::max(0.5, static_cast<double>(20 - idx_iter)) * ptr_sd_map_->get_grid_size();

        auto disturb_start_time = std::chrono::high_resolution_clock::now();
        if (grads_valid4prop)
        {
            ptr_unc_propagation_->propagateUncertainty(params_.shooting_nodes);
        }
        else
        {
            ptr_unc_propagation_->setUncMatricesZeros();
        }

        auto ll_start_time = std::chrono::high_resolution_clock::now();
        for (int idx_hrzn = 1; idx_hrzn <= params_.ocp_dims.n_hrzn; ++idx_hrzn)
        {
            manif::SE2<double> pose_at_tk(solPxAtTk(idx_hrzn), solPyAtTk(idx_hrzn), solYawAtTk(idx_hrzn));
            auto update_obs_subset_start_time = std::chrono::high_resolution_clock::now();
            ptr_sd_map_->argMinDistanceOverGamma(pose_at_tk, params_.ifBilinearInterpArgMinGamma, gamma_gs_result_);
            temp_vector2d_2 << gamma_gs_result_.minimizer[0], gamma_gs_result_.minimizer[1];
            ptr_sd_map_->getClosestObsAtGamma(pose_at_tk, temp_vector2d_2, temp_vector2d_);
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
            updateRobustOCPLinearConstraints(idx_hrzn);
        }
        auto ul_start_time = std::chrono::high_resolution_clock::now();
        solver_status = diff_drive_robot_acados_solve(acados_ocp_capsule_);
        if (solver_status == ACADOS_SUCCESS || solver_status == ACADOS_MAXITER)
        {
            exportAcadosSolToLocalArray(x_sol_new_iter_, u_sol_new_iter_);
            grads_valid4prop = true;
        }
        auto ul_end_time = std::chrono::high_resolution_clock::now();
        elapsed_time_.prop_disturb += std::chrono::duration_cast<std::chrono::microseconds>(ll_start_time - disturb_start_time).count();
        elapsed_time_.lower_level += std::chrono::duration_cast<std::chrono::microseconds>(ul_start_time - ll_start_time).count();
        elapsed_time_.upper_level += std::chrono::duration_cast<std::chrono::microseconds>(ul_end_time - ul_start_time).count();
        return (solver_status == ACADOS_SUCCESS || solver_status == ACADOS_MAXITER);
    }

    int DilatedPolygonRobustController::getObsWithMaximumGVal(int idx_hrzn)
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

    bool DilatedPolygonRobustController::solveEndpointGammaConstrainedSubproblem()
    {
        unsigned int num_imposed_constr = std::min(static_cast<unsigned int>(params_.robot.vertices_flattened.size() / 2), static_cast<unsigned int>(params_.ocp_iters.max_constr_num));
        Eigen::Vector2d gamma_val;
        for (int idx_hrzn = 1; idx_hrzn <= params_.ocp_dims.n_hrzn; ++idx_hrzn)
        {
            num_constr_each_time_step_[idx_hrzn] = num_imposed_constr;
            for (unsigned int idx_constr = 0; idx_constr < num_imposed_constr; ++idx_constr)
            {
                gamma_val << params_.robot.vertices_flattened[2 * idx_constr], params_.robot.vertices_flattened[2 * idx_constr + 1];
                findClosestObsGivenGamma(gamma_val, idx_constr, idx_hrzn);
            }
            // NOTE: nominal constraints
            updateNominalOCPLinearConstraints(idx_hrzn, params_.robot.robot_polygon_dilation_radius);
        }
        auto ul_start_time = std::chrono::high_resolution_clock::now();
        int solver_status = diff_drive_robot_acados_solve(acados_ocp_capsule_);
        exportAcadosSolToLocalArray(x_sol_, u_sol_);
        auto ul_end_time = std::chrono::high_resolution_clock::now();
        elapsed_time_.upper_level += std::chrono::duration_cast<std::chrono::microseconds>(ul_end_time - ul_start_time).count();
        return solver_status;
    }

    void DilatedPolygonRobustController::findClosestObsGivenGamma(const Eigen::Vector2d &gamma_val, unsigned int idx_constr, unsigned int idx_hrzn)
    {
        manif::SE2<double> pose_at_tk(solPxAtTk(idx_hrzn), solPyAtTk(idx_hrzn), solYawAtTk(idx_hrzn));

        Eigen::Vector2d temp_coords, gradient_vec;
        obs_gamma_star_sol_[idx_hrzn][idx_constr].gamma_polygon = gamma_val;
        ptr_sd_map_->getClosestObsAtGamma(pose_at_tk, gamma_val, temp_coords);
        gradient_vec = pose_at_tk.translation() + pose_at_tk.rotation() * gamma_val - temp_coords;
        gradient_vec.normalize();
        obs_gamma_star_sol_[idx_hrzn][idx_constr].obs_coords = temp_coords;
        obs_gamma_star_sol_[idx_hrzn][idx_constr].sep_plane = gradient_vec;
    }

    void DilatedPolygonRobustController::updateGammaAndSepPlane(int idx_hrzn, int num_imposed_constr)
    {
        Eigen::Vector2d temp_coords, temp_coords2;
        Eigen::Vector<int, 2> gamma_star_grid_indices;
        Eigen::Vector<int, 2> gamma_star_grid_clipped;
        manif::SE2<double> pose_at_tk(solPxAtTk(idx_hrzn), solPyAtTk(idx_hrzn), solYawAtTk(idx_hrzn));
        double gradient_x, gradient_y, gradient_norm;
        bool obs_inside_polygon;
        for (int idx_obs = 0; idx_obs < num_imposed_constr; ++idx_obs)
        {
            auto &iter = obs_gamma_star_sol_[idx_hrzn][idx_obs];
            lower_level_param_.heading_line = solYawAtTk(idx_hrzn);
            lower_level_param_.p_center_ellipsoid(0) = iter.obs_coords.x() - solPxAtTk(idx_hrzn);
            lower_level_param_.p_center_ellipsoid(1) = iter.obs_coords.y() - solPyAtTk(idx_hrzn);
            lower_level_param_.shape_matrix_ellipsoid = params_.disturbance.robust_scale * ptr_unc_propagation_->unc_matrices[idx_hrzn].block<2, 2>(0, 0);
            obs_inside_polygon = ptr_ll_nominal_solver_->ifInsidePolygon(solYawAtTk(idx_hrzn), lower_level_param_.p_center_ellipsoid);
            if (lower_level_param_.shape_matrix_ellipsoid.coeff(0, 0) <= 1e-7 || lower_level_param_.shape_matrix_ellipsoid.coeff(1, 1) <= 1e-7)
            {
                ptr_ll_nominal_solver_->argMinDistance(lower_level_param_, lower_level_result_);
                lower_level_result_.delta_p_ellipsoid.setZero();
            }
            else
            {
                ptr_ll_robust_solver_->argMinDistance(lower_level_param_, lower_level_result_);
            }

            temp_coords = pose_at_tk.translation() + pose_at_tk.rotation() * lower_level_result_.gamma_polygon;
            ptr_sd_map_->world2grid_indices_round(temp_coords, gamma_star_grid_indices);
            gamma_star_grid_clipped(0) = std::clamp(gamma_star_grid_indices.coeff(0), 1, ptr_sd_map_->get_map_width() - 2);
            gamma_star_grid_clipped(1) = std::clamp(gamma_star_grid_indices.coeff(1), 1, ptr_sd_map_->get_map_height() - 2);
            bool flag_inside_map = (gamma_star_grid_clipped(0) == gamma_star_grid_indices(0)) && (gamma_star_grid_clipped(1) == gamma_star_grid_indices(1));
            double vec_r2o_norm = (temp_coords - iter.obs_coords).norm();
            // NOTE: Hyperparameter
            if (obs_inside_polygon || vec_r2o_norm <= 1.5 * ptr_sd_map_->get_grid_size() || !flag_inside_map || ptr_sd_map_->ifGridOccupied(gamma_star_grid_indices, params_.sdf.map_occ_threshold))
            {
                // NOTE: When the point is inside the polygon
                // OR When cell is occupied
                // OR When too close, keep the previous gradient
                // OR the point is outside the map
                // approximate the gradient by taking the negative value of vec_r2o will have problems when the robot is inside another obstacle
                gamma_star_grid_indices = gamma_star_grid_clipped;
                gradient_x = ptr_sd_map_->get_sd_at_grid_indices(gamma_star_grid_indices.coeff(0) + 1, gamma_star_grid_indices.coeff(1)) - ptr_sd_map_->get_sd_at_grid_indices(gamma_star_grid_indices.coeff(0) - 1, gamma_star_grid_indices.coeff(1));
                gradient_y = ptr_sd_map_->get_sd_at_grid_indices(gamma_star_grid_indices.coeff(0), gamma_star_grid_indices.coeff(1) + 1) - ptr_sd_map_->get_sd_at_grid_indices(gamma_star_grid_indices.coeff(0), gamma_star_grid_indices.coeff(1) - 1);
                gradient_norm = std::hypot(gradient_x, gradient_y);
                iter.sep_plane.x() = gradient_x / (1e-8 + gradient_norm);
                iter.sep_plane.y() = gradient_y / (1e-8 + gradient_norm);
                ptr_sd_map_->getClosestObsAtGamma(pose_at_tk, lower_level_result_.gamma_polygon, temp_coords2);
                if ((temp_coords2 - iter.obs_coords).norm() > 5 * ptr_sd_map_->get_grid_size())
                {
                    iter.obs_coords = temp_coords2;
                }
                iter.gamma_unc_ellipse.setZero();
                iter.gamma_polygon = lower_level_result_.gamma_polygon;
            }
            else
            {
                iter.sep_plane.x() = (temp_coords.x() - iter.obs_coords.x()) / vec_r2o_norm;
                iter.sep_plane.y() = (temp_coords.y() - iter.obs_coords.y()) / vec_r2o_norm;
                iter.gamma_unc_ellipse = lower_level_result_.delta_p_ellipsoid;
                iter.gamma_polygon = lower_level_result_.gamma_polygon;
            }
        }
    }

    void DilatedPolygonRobustController::updateRobustOCPLinearConstraints(int idx_hrzn)
    {
        int max_constr_num = params_.ocp_iters.max_constr_num;
        memset(Cg_, 0, params_.ocp_dims.nx * params_.ocp_iters.max_constr_num * sizeof(double));
        std::fill(lg_, lg_ + max_constr_num, -1e3);
        Eigen::Matrix2d rot_mat, rot_mat_derivative;
        rot_mat << std::cos(x_sol_[idx_hrzn * params_.ocp_dims.nx + 2]), -std::sin(x_sol_[idx_hrzn * params_.ocp_dims.nx + 2]),
            std::sin(x_sol_[idx_hrzn * params_.ocp_dims.nx + 2]), std::cos(x_sol_[idx_hrzn * params_.ocp_dims.nx + 2]);
        rot_mat_derivative << -std::sin(x_sol_[idx_hrzn * params_.ocp_dims.nx + 2]), -std::cos(x_sol_[idx_hrzn * params_.ocp_dims.nx + 2]),
            std::cos(x_sol_[idx_hrzn * params_.ocp_dims.nx + 2]), -std::sin(x_sol_[idx_hrzn * params_.ocp_dims.nx + 2]);
        double backoff_angular_uncertainty = std::sqrt(std::max(0.0, params_.disturbance.robust_scale * ptr_unc_propagation_->unc_matrices[idx_hrzn].coeff(2, 2))) * max_gamma_polygon_norm_;
        for (int idx_constr = 0; idx_constr < num_constr_each_time_step_[idx_hrzn]; ++idx_constr)
        {
            const auto &sol = obs_gamma_star_sol_[idx_hrzn][idx_constr];
            Cg_[idx_constr + max_constr_num * 0] = sol.sep_plane.x();
            Cg_[idx_constr + max_constr_num * 1] = sol.sep_plane.y();
            Cg_[idx_constr + max_constr_num * 2] = sol.sep_plane.dot(rot_mat_derivative * sol.gamma_polygon);
            lg_[idx_constr] = -sol.sep_plane.dot((rot_mat - x_sol_[idx_hrzn * params_.ocp_dims.nx + 2] * rot_mat_derivative) * sol.gamma_polygon + sol.gamma_unc_ellipse - sol.obs_coords) + params_.robot.robot_polygon_dilation_radius + backoff_angular_uncertainty;
        }
        ocp_nlp_constraints_model_set(nlp_config_, nlp_dims_, nlp_in_, nlp_out_, idx_hrzn, "lg", lg_);
        ocp_nlp_constraints_model_set(nlp_config_, nlp_dims_, nlp_in_, nlp_out_, idx_hrzn, "C", Cg_);
    }
}
