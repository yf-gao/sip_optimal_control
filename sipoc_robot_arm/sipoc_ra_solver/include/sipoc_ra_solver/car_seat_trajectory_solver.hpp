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

#ifndef SIPOC_RA_SOLVER__CAR_SEAT_TRAJECTORY_SOLVER_HPP
#define SIPOC_RA_SOLVER__CAR_SEAT_TRAJECTORY_SOLVER_HPP

#include "sipoc_ra_utils/utils.hpp"
#include "sipoc_ra_solver/pinocchio_kinematics.hpp"
#include "sipoc_ra_solver/coal_collision_detection.hpp"
#include "sipoc_ra_solver/collision_constraint_manager.hpp"

#if defined(SIPOC_RA_SOLVER__SIA20D_PLANNING)
#include "acados_generated_code/c_generated_code_sipoc_ra_planning/acados_solver_robot_joints.h"
#elif defined(SIPOC_RA_SOLVER__SIA20D_MPC)
#include "acados_generated_code/c_generated_code_sipoc_ra_mpc/acados_solver_robot_joints.h"
#endif

#include <manif/manif.h>
#include <eigen3/Eigen/Dense>

#include <memory>
#include <vector>
#include <map>

namespace sipoc_ra_solver
{

    class CarSeatTrajectorySolver
    {
    public:
        CarSeatTrajectorySolver(const sipoc_ra_utils::ConfigTrajectorySIPSolver &config_sip_solver, const std::string &urdf_path, const std::string &env_path, double seat_scale);

        ~CarSeatTrajectorySolver();

        sipoc_ra_utils::SolverStatus solve();
        sipoc_ra_utils::SolverStatus solveOneIteration(bool if_get_value_from_sol = false);

        void setOCPCurrentJointState(const Eigen::VectorXd &current_joint_state);
        void setOCPTerminalJointState(const Eigen::VectorXd &current_joint_state);
        void setOCPRefTrajectory(const std::vector<Eigen::VectorXd> &ref_trajectory, bool flag_reinitialize);
        void initializeSolver(const std::vector<Eigen::VectorXd> &vec_joint_state_init);

        void printInfoIter() const;
        void summarizeInfoIter(sipoc_ra_utils::InfoIterSIPSolver &info_iter) const;

        double evaluateConstraintSatisfaction(bool eval_whole_trajectory);
        bool checkSlackVariableNonZero(double thr);
        void shiftInputStateTrajectory();
        void gatherActiveConstraintsAndShiftOneStep();

        std::vector<manif::SE3<double>> getSolLinkPosesOverTime(const std::string &link_name) const;

        inline std::shared_ptr<PinocchioKinematics> getPinocchioKinematicsSharedPtr() const
        {
            return pinocchio_kin_;
        }

        inline const std::vector<Eigen::VectorXd> &getSolRobotJointsOverTime() const
        {
            return vec_joint_state_sol_;
        }

        inline void getU0Star(Eigen::VectorXd &u0_star) const
        {
            std::copy(array_u_sol_, array_u_sol_ + config_sip_solver_.ocp.nu, u0_star.data());
        }

        inline const sipoc_ra_utils::ConfigTrajectorySIPSolver &getSolverConfig() const
        {
            return config_sip_solver_;
        }

        unsigned int getNumHorizons() const
        {
            return config_sip_solver_.ocp.n_hrzn;
        }

        unsigned int getNumJoints() const
        {
            return pinocchio_kin_->getModelNv();
        }

        inline void getPCSubsetPositions(const std::string &collision_element, unsigned int idx, std::vector<Eigen::Vector3d> &pc_positions)
        {
            collision_constraint_manager_->getPCSubsetPositions(collision_element, idx, pc_positions);
        }

        inline void getCollSeparatingPlanes(unsigned int idx, std::vector<Eigen::Vector<double, 6>> &vec_sep_planes)
        {
            vec_sep_planes = map_idx_hrzn_sep_planes_.at(idx);
        }

        inline double evaluateMinimumSignedDistance(const Eigen::VectorXd &joint_angles) const
        {
            return collision_constraint_manager_->evaluateMinimumSignedDistance(pinocchio_kin_, joint_angles);
        }

        inline void clearPCSubsets()
        {
            collision_constraint_manager_->clearPCSubsets();
        }

    private:
        std::shared_ptr<PinocchioKinematics> pinocchio_kin_;
        std::shared_ptr<CollisionConstraintManager> collision_constraint_manager_;
        std::map<std::string, std::string> map_collision_element_link_name_;

        robot_joints_solver_capsule *acados_ocp_capsule_ = NULL;
        ocp_nlp_config *nlp_config_ = NULL;
        ocp_nlp_dims *nlp_dims_ = NULL;
        ocp_nlp_in *nlp_in_ = NULL;
        ocp_nlp_out *nlp_out_ = NULL;
        ocp_nlp_solver *nlp_solver_ = NULL;
        void *nlp_opts_ = NULL;
        double *array_lg_ = NULL;
        double *array_Cg_ = NULL;
        double *array_x_sol_ = NULL;
        double *array_u_sol_ = NULL;
        double *array_sl_sol_ = NULL;
        double *array_lam_ = NULL;
        double *array_y_ref_ = NULL;

        sipoc_ra_utils::ConfigTrajectorySIPSolver config_sip_solver_;

        std::map<unsigned int, std::vector<Eigen::Vector<double, 6>>> map_idx_hrzn_sep_planes_;
        std::vector<Eigen::VectorXd> vec_joint_state_ref_, vec_joint_state_sol_, vec_joint_state_last_iter_;
        std::vector<double> vec_l_infinity_step_;
        std::vector<Eigen::VectorXd> vec_constr_coeffs_tmp_;
        std::vector<sipoc_ra_utils::InfoIterSIPSolver> vec_info_iter_;
        sipoc_ra_utils::InfoIterSIPSolver info_iter_current_;
        unsigned int num_joints_;

        void initializeSolver();
        void checkOCPSameDimsAsExported() const;
        void createAcadosSolver();
        void configureOCP();
    };
}

#endif // SIPOC_RA_SOLVER__CAR_SEAT_TRAJECTORY_SOLVER_HPP
