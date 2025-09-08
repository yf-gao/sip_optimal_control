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

#ifndef SIPOC_MR_CONTROLLER__BASE_CONTROLLER_HPP_
#define SIPOC_MR_CONTROLLER__BASE_CONTROLLER_HPP_

#include "sipoc_mr_utils/signed_distance_map.hpp"
#include "sipoc_mr_utils/common_structs.hpp"

extern "C"
{
// general acados
#include "acados/ocp_nlp/ocp_nlp_constraints_bgh.h"
#include "acados_c/ocp_nlp_interface.h"
}

#include <manif/SE2.h>
#include <memory>
#include <chrono>
#include <iostream>
#include <vector>

#ifdef DEBUG_PRINTING
#define DEBUG_MSG(str)                 \
    do                                 \
    {                                  \
        std::cout << str << std::endl; \
    } while (false)
#else
#define DEBUG_MSG(str) \
    do                 \
    {                  \
    } while (false)
#endif

namespace sipoc_mr_solver
{
    class BaseController
    {

    public:
        bool solver_initialized = false;

        BaseController()
        {
            params_.shooting_nodes.clear();
            for (int idx = 0; idx <= params_.ocp_dims.n_hrzn; ++idx)
            {
                params_.shooting_nodes.push_back(idx * params_.delta_t_if_shooting_nodes_is_empty);
            }
            allocateMemory();
        }
        BaseController(int n_hrzn, double delta_t)
        {
            params_.ocp_dims.n_hrzn = n_hrzn;
            params_.shooting_nodes.clear();
            for (int idx = 0; idx <= params_.ocp_dims.n_hrzn; ++idx)
            {
                params_.shooting_nodes.push_back(idx * delta_t);
            }
            allocateMemory();
        }
        BaseController(sipoc_mr_utils::OCPParameters &param)
        {
            params_ = param;
            if (params_.shooting_nodes.size() == 0)
            {
                for (int idx = 0; idx <= params_.ocp_dims.n_hrzn; ++idx)
                {
                    params_.shooting_nodes.push_back(idx * params_.delta_t_if_shooting_nodes_is_empty);
                }
            }
            allocateMemory();
        }
        virtual ~BaseController()
        {
            deallocateMemory();
        }
        inline void setupOcpSolver()
        {
            createAcadosSolver();
            checkOCPSameTimeStampsAsExported();
            configureOCPWeights();
            configureOCPVelAccBounds();
            configureLowerLevelSolver();
        }
        inline void loadOccAndUpdateSDF(std::string sdf_file_str, sipoc_mr_utils::SDFParam &sdf_param)
        {
            ptr_sd_map_->loadOccupancyMap(sdf_file_str, sdf_param);
            ptr_sd_map_->computeSignedDistanceMap();
        }
        inline void UpdateOccAndSDF(const std::vector<signed char> &map, unsigned char threshold)
        {
            ptr_sd_map_->updateOccupancyMap(map);
            ptr_sd_map_->computeSignedDistanceMap(threshold);
        }

        inline int iterConverg()
        {
            return num_iter_converg_;
        }

        inline void exportU0Star(Eigen::Vector<double, 2> &u0_star)
        {
            u0_star(0) = u_sol_[0];
            u0_star(1) = u_sol_[1];
            return;
        }

        inline void exportX1VelStar(Eigen::Vector<double, 2> &x1_vel_star)
        {
            x1_vel_star(0) = x_sol_[params_.ocp_dims.nx + 3];
            x1_vel_star(1) = x_sol_[params_.ocp_dims.nx + 4];
            return;
        }

        inline double lower_level_elapsed_time()
        {
            return elapsed_time_.lower_level;
        }

        inline double upper_level_elapsed_time()
        {
            return elapsed_time_.upper_level;
        }

        inline double prop_disturb_elapsed_time()
        {
            return elapsed_time_.prop_disturb;
        }

        inline void resetElapsedTime()
        {
            elapsed_time_.lower_level = 0.;
            elapsed_time_.upper_level = 0.;
            elapsed_time_.prop_disturb = 0.;
            elapsed_time_.update_obs_subset = 0.;
        }

        void updateReferenceTrajectory(const double *yref, const double *yref_e);
        void updateInitialState(const manif::SE2<double> &current_pose, const manif::SE2Tangent<double> &current_ref_vel);
        void updateInitialState(const Eigen::Vector<double, 9> &full_state);
        void initializeOCPByReference();
        virtual bool solveEndpointGammaConstrainedSubproblem() = 0;
        sipoc_mr_utils::SIPSolverStatus solveOCP(bool if_reset_num_constr);
        void shiftInputStateTrajectory(unsigned int num_steps);
        void gatherActiveConstraintsAndShiftOneStep();

        // FOR ROS2
        void obsSolAtTk(int idx_hrzn, std::vector<Eigen::Vector2d> &vec_obs);

        inline void poseSolAtTk(Eigen::Vector<double, 3> &pose_k, int idx)
        {
            pose_k(0) = x_sol_[idx * params_.ocp_dims.nx];
            pose_k(1) = x_sol_[idx * params_.ocp_dims.nx + 1];
            pose_k(2) = x_sol_[idx * params_.ocp_dims.nx + 2];
            return;
        }

    protected:
        sipoc_mr_utils::OCPParameters params_;
        struct ComputationTime
        {
            double lower_level;
            double upper_level;
            double prop_disturb;
            double update_obs_subset;
        } elapsed_time_;
        struct ObsGammaStarSolution
        {
            Eigen::Vector2d obs_coords;
            Eigen::Vector2d gamma_polygon;
            Eigen::Vector2d gamma_unc_ellipse;
            Eigen::Vector2d sep_plane;
        };

        std::unique_ptr<sipoc_mr_utils::SignedDistanceMap> ptr_sd_map_;

        ocp_nlp_config *nlp_config_ = NULL;
        ocp_nlp_dims *nlp_dims_ = NULL;
        ocp_nlp_in *nlp_in_ = NULL;
        ocp_nlp_out *nlp_out_ = NULL;
        ocp_nlp_solver *nlp_solver_ = NULL;
        void *nlp_opts_ = NULL;

        double *lubx0_;
        double *yref_;
        double *yref_e_;
        double *x_sol_;
        double *x_sol_new_iter_;
        double *u_sol_;
        double *u_sol_new_iter_;
        double *sl_sol_;
        double *lg_;
        double *Cg_;
        double *lam_;
        int *num_constr_each_time_step_;
        std::vector<std::vector<ObsGammaStarSolution>> obs_gamma_star_sol_;
        double flag_constr_tightening_ = 1.0;
        int num_iter_converg_ = 0;

        void allocateMemory();
        void deallocateMemory();
        void setZerosAllArrays();
        void checkOCPSameTimeStampsAsExported();
        void configureOCPWeights();
        void configureOCPVelAccBounds();
        void findClosestObsGivenGamma(double gamma_val, unsigned int idx_constr, unsigned int idx_hrzn);
        void updateNominalOCPLinearConstraints(int idx_hrzn, double padding_radius);
        bool ifConverged();

        virtual void checkOCPSameDimsAsExported() = 0;
        virtual void createAcadosSolver() = 0;
        virtual bool solveOCPSubproblem(int idx_iter) = 0;
        virtual void configureLowerLevelSolver() = 0;

        inline double solPxAtTk(int idx_hrzn)
        {
            return x_sol_[idx_hrzn * params_.ocp_dims.nx];
        }

        inline double solPyAtTk(int idx_hrzn)
        {
            return x_sol_[idx_hrzn * params_.ocp_dims.nx + 1];
        }

        inline double solLinearVelAtTk(int idx_hrzn)
        {
            return x_sol_[idx_hrzn * params_.ocp_dims.nx + 3];
        }

        inline double solLinearAccAtTk(int idx_hrzn)
        {
            return u_sol_[idx_hrzn * params_.ocp_dims.nu];
        }

        inline double solYawAtTk(int idx_hrzn)
        {
            return x_sol_[idx_hrzn * params_.ocp_dims.nx + 2];
        }

        inline void exportAcadosSolToLocalArray(double *dest_x_sol, double *dest_u_sol)
        {
            ocp_nlp_get_all(nlp_solver_, nlp_in_, nlp_out_, "x", dest_x_sol);
            ocp_nlp_get_all(nlp_solver_, nlp_in_, nlp_out_, "u", dest_u_sol);
            for (int idx_hrzn = 1; idx_hrzn <= params_.ocp_dims.n_hrzn; ++idx_hrzn)
            {
                // NOTE: the collision-avoidance constraints are not imposed for the initial state. Therfore get the values by time index.
                ocp_nlp_out_get(nlp_config_, nlp_dims_, nlp_out_, idx_hrzn, "sl", sl_sol_ + idx_hrzn * params_.ocp_iters.max_constr_num);
            }
        }
    };
}

#endif // SIPOC_MR_CONTROLLER__BASE_CONTROLLER_HPP_
