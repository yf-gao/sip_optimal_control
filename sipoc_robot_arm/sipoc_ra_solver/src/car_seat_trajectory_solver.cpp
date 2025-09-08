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

#include "sipoc_ra_solver/car_seat_trajectory_solver.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <filesystem>
#include <chrono>

namespace sipoc_ra_solver
{

    CarSeatTrajectorySolver::CarSeatTrajectorySolver(const sipoc_ra_utils::ConfigTrajectorySIPSolver &config_sip_solver, const std::string &urdf_path, const std::string &env_path, double seat_scale) : config_sip_solver_(config_sip_solver)
    {
        pinocchio_kin_ = std::make_shared<PinocchioKinematics>(urdf_path);
        num_joints_ = pinocchio_kin_->getModelNv();
        if (config_sip_solver_.ocp.joint_vel_as_state && 2 * num_joints_ != config_sip_solver_.ocp.nx)
        {
            std::cout << "num_joints_: " << num_joints_ << std::endl;
            std::cout << "config_sip_solver_.ocp.nx: " << config_sip_solver_.ocp.nx << std::endl;
            throw std::runtime_error("The number of joints in the robot model does not equal to config_sip_solver_.ocp.nx / 2.");
        }
        if (!config_sip_solver_.ocp.joint_vel_as_state && num_joints_ != config_sip_solver_.ocp.nx)
        {
            throw std::runtime_error("The number of joints in the robot model does not equal to config_sip_solver_.ocp.nx.");
        }

        // acados
        this->checkOCPSameDimsAsExported();
        this->createAcadosSolver();
        this->configureOCP();

        std::map<std::string, sipoc_ra_utils::ConfigCollisionElementShape> map_collision_elements_shapes;
        map_collision_element_link_name_.clear();

        sipoc_ra_utils::ConfigCollisionElementShape config_convex_hull;
        config_convex_hull.link_name = "seat";
        unsigned int num_elements = 10;
        std::string car_seat_path = ament_index_cpp::get_package_share_directory("sipoc_ra_support") + "/meshes/car_seat/collision";
        for (unsigned int i = 1; i <= num_elements; ++i)
        {
            config_convex_hull.convex.mesh_path = car_seat_path + "/seat_" + std::to_string(i) + ".stl";
            config_convex_hull.convex.scale = seat_scale;
            map_collision_elements_shapes.insert(std::make_pair("seat_" + std::to_string(i), config_convex_hull));
            map_collision_element_link_name_.insert(std::make_pair("seat_" + std::to_string(i), config_convex_hull.link_name));
        }
        collision_constraint_manager_ = std::make_shared<CollisionConstraintManager>(map_collision_elements_shapes, env_path, config_sip_solver_.ocp.num_max_constr, config_sip_solver_.ocp.n_hrzn, num_joints_, config_sip_solver_.use_cropped_octomap);

        manif::SE3<double> se3_car, se3_rail;
        pinocchio_kin_->computeForwardKinematics(Eigen::VectorXd::Zero(num_joints_));
        pinocchio_kin_->getFrameSE3("car", se3_car);
        collision_constraint_manager_->setEnvPose(se3_car);

        vec_joint_state_sol_.clear();
        map_idx_hrzn_sep_planes_.clear();
        for (unsigned int i = 0; i <= config_sip_solver_.ocp.n_hrzn; ++i)
        {
            vec_joint_state_sol_.emplace_back(Eigen::VectorXd::Zero(config_sip_solver_.ocp.nx));
            map_idx_hrzn_sep_planes_[i] = std::vector<Eigen::Vector<double, 6>>();
        }
        vec_l_infinity_step_.resize(config_sip_solver_.ocp.n_hrzn + 1);
    }

    CarSeatTrajectorySolver::~CarSeatTrajectorySolver()
    {
        // acados
        free(array_lg_);
        free(array_Cg_);
        free(array_x_sol_);
        free(array_u_sol_);
        free(array_sl_sol_);
        free(array_lam_);
        free(array_y_ref_);
        robot_joints_acados_free(acados_ocp_capsule_);
        robot_joints_acados_free_capsule(acados_ocp_capsule_);
    }

    void CarSeatTrajectorySolver::checkOCPSameDimsAsExported() const
    {
        if (config_sip_solver_.ocp.n_hrzn != ROBOT_JOINTS_N)
        {
            std::cout << "config_sip_solver_.ocp.n_hrzn: " << config_sip_solver_.ocp.n_hrzn << std::endl;
            std::cout << "ROBOT_JOINTS_N: " << ROBOT_JOINTS_N << std::endl;
            throw std::runtime_error("not same number of shooting nodes");
        }
        if (config_sip_solver_.ocp.num_max_constr != ROBOT_JOINTS_NG)
        {
            std::cout << "config_sip_solver_.ocp.num_max_constr: " << config_sip_solver_.ocp.num_max_constr << std::endl;
            std::cout << "ROBOT_JOINTS_NG: " << ROBOT_JOINTS_NG << std::endl;
            throw std::runtime_error("not same number of inequality constraints");
        }
        if (config_sip_solver_.ocp.nx != ROBOT_JOINTS_NX)
        {
            std::cout << "config_sip_solver_.ocp.nx: " << config_sip_solver_.ocp.nx << std::endl;
            std::cout << "ROBOT_JOINTS_NX: " << ROBOT_JOINTS_NX << std::endl;
            throw std::runtime_error("not same number of states");
        }
        if (config_sip_solver_.ocp.nu != ROBOT_JOINTS_NU)
        {
            std::cout << "config_sip_solver_.ocp.nu: " << config_sip_solver_.ocp.nu << std::endl;
            std::cout << "ROBOT_JOINTS_NU: " << ROBOT_JOINTS_NU << std::endl;
            throw std::runtime_error("not same number of control inputs");
        }
    }

    void CarSeatTrajectorySolver::createAcadosSolver()
    {
        acados_ocp_capsule_ = robot_joints_acados_create_capsule();
        double *new_time_steps = NULL;
        int status = robot_joints_acados_create_with_discretization(
            acados_ocp_capsule_, ROBOT_JOINTS_N, new_time_steps);
        if (status)
        {
            exit(1);
        }
        nlp_config_ = robot_joints_acados_get_nlp_config(acados_ocp_capsule_);
        nlp_dims_ = robot_joints_acados_get_nlp_dims(acados_ocp_capsule_);
        nlp_in_ = robot_joints_acados_get_nlp_in(acados_ocp_capsule_);
        nlp_out_ = robot_joints_acados_get_nlp_out(acados_ocp_capsule_);
        nlp_solver_ = robot_joints_acados_get_nlp_solver(acados_ocp_capsule_);
        nlp_opts_ = robot_joints_acados_get_nlp_opts(acados_ocp_capsule_);

        const unsigned int n_hrzn = config_sip_solver_.ocp.n_hrzn;
        const unsigned int nu = config_sip_solver_.ocp.nu;
        const unsigned int nx = config_sip_solver_.ocp.nx;
        array_lg_ = (double *)malloc(config_sip_solver_.ocp.num_max_constr * sizeof(double));
        array_Cg_ = (double *)malloc(config_sip_solver_.ocp.num_max_constr * nx * sizeof(double));
        array_x_sol_ = (double *)malloc((n_hrzn + 1) * nx * sizeof(double));
        array_u_sol_ = (double *)malloc(n_hrzn * nu * sizeof(double));
        array_sl_sol_ = (double *)malloc((n_hrzn + 1) * config_sip_solver_.ocp.num_max_constr * sizeof(double));
        array_lam_ = (double *)malloc(n_hrzn * (ROBOT_JOINTS_NBX + ROBOT_JOINTS_NBU + ROBOT_JOINTS_NG + ROBOT_JOINTS_NS) * 2 * sizeof(double));
        array_y_ref_ = (double *)malloc((n_hrzn * (nx + nu) + nx) * sizeof(double));
        memset(array_lg_, 0., config_sip_solver_.ocp.num_max_constr * sizeof(double));
        memset(array_Cg_, 0., config_sip_solver_.ocp.num_max_constr * nx * sizeof(double));
        memset(array_x_sol_, 0., (n_hrzn + 1) * nx * sizeof(double));
        memset(array_u_sol_, 0., n_hrzn * nu * sizeof(double));
        memset(array_sl_sol_, 0., (n_hrzn + 1) * config_sip_solver_.ocp.num_max_constr * sizeof(double));
        memset(array_lam_, 0., (ROBOT_JOINTS_NBX + ROBOT_JOINTS_NBU + ROBOT_JOINTS_NG + ROBOT_JOINTS_NS) * n_hrzn * 2 * sizeof(double));
        memset(array_y_ref_, 0., (n_hrzn * (nx + nu) + nx) * sizeof(double));
    }

    void CarSeatTrajectorySolver::configureOCP()
    {
        int nw = config_sip_solver_.ocp.nx + config_sip_solver_.ocp.nu;
        double *W = (double *)malloc(nw * nw * sizeof(double));
        memset(W, 0, nw * nw * sizeof(double));
        for (uint32_t i = 0; i < num_joints_; ++i)
        {
            W[i + nw * i] = config_sip_solver_.ocp.weight_joint_angle;
            W[i + num_joints_ + nw * (i + num_joints_)] = config_sip_solver_.ocp.weight_joint_vel;
            if (config_sip_solver_.ocp.joint_vel_as_state)
            {
                W[i + 2 * num_joints_ + nw * (i + 2 * num_joints_)] = config_sip_solver_.ocp.weight_joint_acc;
            }
        }
        for (uint32_t idx_hrzn = 0; idx_hrzn < config_sip_solver_.ocp.n_hrzn; ++idx_hrzn)
        {
            ocp_nlp_cost_model_set(nlp_config_, nlp_dims_, nlp_in_, idx_hrzn, "W", W);
        }
        free(W);

        nw = config_sip_solver_.ocp.nx;
        double *W_e = (double *)malloc(nw * nw * sizeof(double));
        memset(W_e, 0, nw * nw * sizeof(double));
        for (uint32_t i = 0; i < num_joints_; ++i)
        {
            W_e[i + nw * i] = config_sip_solver_.ocp.weight_joint_angle_terminal;
            if (config_sip_solver_.ocp.joint_vel_as_state)
            {
                W_e[i + num_joints_ + nw * (i + num_joints_)] = config_sip_solver_.ocp.weight_joint_vel_terminal;
            }
        }
        ocp_nlp_cost_model_set(nlp_config_, nlp_dims_, nlp_in_, config_sip_solver_.ocp.n_hrzn, "W", W_e);
        free(W_e);

        double *lubu = (double *)malloc(2 * num_joints_ * sizeof(double));
        double *lbu = lubu;
        double *ubu = lubu + num_joints_;
        if (config_sip_solver_.ocp.joint_vel_as_state)
        {
            for (unsigned int i = 0; i < num_joints_; ++i)
            {
                lbu[i] = -config_sip_solver_.bounds.max_joint_acc;
                ubu[i] = config_sip_solver_.bounds.max_joint_acc;
            }
        }
        else
        {
            for (unsigned int i = 0; i < num_joints_; ++i)
            {
                lbu[i] = -config_sip_solver_.bounds.max_joint_vel;
                ubu[i] = config_sip_solver_.bounds.max_joint_vel;
            }
        }
        for (unsigned int i = 0; i < config_sip_solver_.ocp.n_hrzn; i++)
        {
            ocp_nlp_constraints_model_set(nlp_config_, nlp_dims_, nlp_in_, nlp_out_, i, "lbu", lbu);
            ocp_nlp_constraints_model_set(nlp_config_, nlp_dims_, nlp_in_, nlp_out_, i, "ubu", ubu);
        }
        free(lubu);

        if (config_sip_solver_.ocp.joint_vel_as_state)
        {
            double *lubx = (double *)malloc(2 * num_joints_ * sizeof(double));
            double *lbx = lubx;
            double *ubx = lubx + num_joints_;
            for (unsigned int i = 0; i < num_joints_; ++i)
            {
                lbx[i] = -config_sip_solver_.bounds.max_joint_vel;
                ubx[i] = config_sip_solver_.bounds.max_joint_vel;
            }
            for (unsigned int i = 1; i <= config_sip_solver_.ocp.n_hrzn; i++)
            {
                ocp_nlp_constraints_model_set(nlp_config_, nlp_dims_, nlp_in_, nlp_out_, i, "lbx", lbx);
                ocp_nlp_constraints_model_set(nlp_config_, nlp_dims_, nlp_in_, nlp_out_, i, "ubx", ubx);
            }
            free(lubx);
        }
    }

    void CarSeatTrajectorySolver::setOCPCurrentJointState(const Eigen::VectorXd &current_joint_state)
    {
        std::copy(current_joint_state.data(), current_joint_state.data() + config_sip_solver_.ocp.nx, array_y_ref_);
        ocp_nlp_cost_model_set(nlp_config_, nlp_dims_, nlp_in_, 0, "yref", array_y_ref_);
        ocp_nlp_constraints_model_set(nlp_config_, nlp_dims_, nlp_in_, nlp_out_, 0, "lbx", array_y_ref_);
        ocp_nlp_constraints_model_set(nlp_config_, nlp_dims_, nlp_in_, nlp_out_, 0, "ubx", array_y_ref_);
    }

    void CarSeatTrajectorySolver::setOCPTerminalJointState(const Eigen::VectorXd &terminal_joint_state)
    {
        if (!config_sip_solver_.ocp.terminal_equality_constraints)
        {
            throw std::runtime_error("The terminal state is not subject to equality constraints.");
        }
        unsigned int n_hrzn = config_sip_solver_.ocp.n_hrzn;
        double *ptr = array_y_ref_ + n_hrzn * (config_sip_solver_.ocp.nx + config_sip_solver_.ocp.nu);
        std::copy(terminal_joint_state.data(), terminal_joint_state.data() + config_sip_solver_.ocp.nx, ptr);
        ocp_nlp_cost_model_set(nlp_config_, nlp_dims_, nlp_in_, n_hrzn, "yref", ptr);
        ocp_nlp_constraints_model_set(nlp_config_, nlp_dims_, nlp_in_, nlp_out_, n_hrzn, "lbx", ptr);
        ocp_nlp_constraints_model_set(nlp_config_, nlp_dims_, nlp_in_, nlp_out_, n_hrzn, "ubx", ptr);
    }

    void CarSeatTrajectorySolver::setOCPRefTrajectory(const std::vector<Eigen::VectorXd> &ref_trajectory, bool flag_reinitialize)
    {
        if (ref_trajectory.size() != config_sip_solver_.ocp.n_hrzn + 1)
        {
            throw std::invalid_argument("As arguments of function CarSeatTrajectorySolver::setOCPRefTrajectory, std::vector<Eigen::VectorXd> must have " +
                                        std::to_string(config_sip_solver_.ocp.n_hrzn + 1) + " elements.");
        }

        const unsigned int n_hrzn = config_sip_solver_.ocp.n_hrzn;
        const unsigned int nu = config_sip_solver_.ocp.nu;
        const unsigned int nx = config_sip_solver_.ocp.nx;
        // NOTE: the reference input are zeros.
        memset(array_y_ref_, 0., (n_hrzn * (nx + nu) + nx) * sizeof(double));
        for (unsigned int idx = 1; idx <= n_hrzn; ++idx)
        {
            std::copy(ref_trajectory[idx].data(), ref_trajectory[idx].data() + config_sip_solver_.ocp.nx, array_y_ref_ + idx * (config_sip_solver_.ocp.nx + config_sip_solver_.ocp.nu));
            ocp_nlp_cost_model_set(nlp_config_, nlp_dims_, nlp_in_, idx, "yref", array_y_ref_ + idx * (config_sip_solver_.ocp.nx + config_sip_solver_.ocp.nu));
        }

        vec_joint_state_ref_ = ref_trajectory;
        if (flag_reinitialize)
        {
            this->initializeSolver(vec_joint_state_ref_);
        }
    }

    void CarSeatTrajectorySolver::initializeSolver()
    {
        throw std::runtime_error("Please place an std::vector<Eigen::VectorXd> for initialization");
    }

    void CarSeatTrajectorySolver::initializeSolver(const std::vector<Eigen::VectorXd> &vec_joint_state_init)
    {
        // Initialize the solution with the target joint values
        vec_joint_state_last_iter_ = vec_joint_state_init;

        // acados
        memset(array_u_sol_, 0., config_sip_solver_.ocp.n_hrzn * config_sip_solver_.ocp.nu * sizeof(double));
        for (unsigned int idx = 0; idx < config_sip_solver_.ocp.n_hrzn; ++idx)
        {
            ocp_nlp_out_set(nlp_config_, nlp_dims_, nlp_out_, nlp_in_, idx, "u", array_u_sol_ + idx * config_sip_solver_.ocp.nu);
        }
        for (unsigned int idx = 0; idx <= config_sip_solver_.ocp.n_hrzn; ++idx)
        {
            std::copy(vec_joint_state_ref_[idx].data(), vec_joint_state_ref_[idx].data() + config_sip_solver_.ocp.nx, array_x_sol_ + idx * config_sip_solver_.ocp.nx);
            ocp_nlp_out_set(nlp_config_, nlp_dims_, nlp_out_, nlp_in_, idx, "x", array_x_sol_ + idx * config_sip_solver_.ocp.nx);
        }
    }

    std::vector<manif::SE3<double>> CarSeatTrajectorySolver::getSolLinkPosesOverTime(const std::string &link_name) const
    {
        std::vector<manif::SE3<double>> vec_se3_link;
        for (unsigned int idx = 0; idx <= config_sip_solver_.ocp.n_hrzn; ++idx)
        {
            manif::SE3<double> se3_link;
            pinocchio_kin_->computeForwardKinematics(vec_joint_state_sol_[idx].head(num_joints_));
            pinocchio_kin_->getFrameSE3(link_name, se3_link);
            vec_se3_link.push_back(se3_link);
        }
        return vec_se3_link;
    }

    sipoc_ra_utils::SolverStatus CarSeatTrajectorySolver::solveOneIteration(bool /*if_get_value_from_sol*/)
    {
        info_iter_current_.time_constr_linearization = 0.0;
        info_iter_current_.time_detailed_collision_detection.time_coal = 0.0;
        info_iter_current_.time_detailed_collision_detection.time_min_element = 0.0;
        info_iter_current_.time_detailed_collision_detection.time_update = 0.0;

        auto start_iter = std::chrono::high_resolution_clock::now();

        // The initial robot pose is not subject to collision constraints
        // The constraints of the initial state are still imposed in acados, the feasibility in handled in the Python script.
        for (unsigned int idx = 1; idx <= config_sip_solver_.ocp.n_hrzn; ++idx)
        {
            pinocchio_kin_->computeForwardKinematics(vec_joint_state_last_iter_[idx].head(num_joints_));

            auto start_constr_lin = std::chrono::high_resolution_clock::now();
            bool success_lin_constr = collision_constraint_manager_->linearizeConstraints(vec_joint_state_last_iter_[idx].head(num_joints_), pinocchio_kin_, idx, vec_constr_coeffs_tmp_, map_idx_hrzn_sep_planes_.at(idx));
            auto end_constr_lin = std::chrono::high_resolution_clock::now();
            info_iter_current_.time_constr_linearization += std::chrono::duration<double>(end_constr_lin - start_constr_lin).count();

            if (!success_lin_constr)
            {
                std::cerr << "Error in solving linearizing constraints." << std::endl;
                return sipoc_ra_utils::SolverStatus::LL_FAILED;
            }

            // acados
            std::fill(array_lg_, array_lg_ + config_sip_solver_.ocp.num_max_constr, -1e4);
            std::fill(array_Cg_, array_Cg_ + config_sip_solver_.ocp.num_max_constr * config_sip_solver_.ocp.nx, 1e-3);
            for (unsigned int idx_constr = 0; idx_constr < vec_constr_coeffs_tmp_.size(); ++idx_constr)
            {
                // vec_constr_coeffs_tmp_[idx_constr].head(nv).dot(x) + vec_constr_coeffs_tmp_[idx_constr].tail(1)[0] >= 0.
                // in acados, Cx+Du >= lg. D is set to zero in Python.
                // Here we take the minus sign for lg.
                array_lg_[idx_constr] = -1.0 * vec_constr_coeffs_tmp_[idx_constr].tail(1)[0];
                // NOTE: Use the value in the header file. potentially optimized by compiler
                // NOTE: The constraints are only functions of the position, not the velocity.
                for (unsigned int ix = 0; ix < num_joints_; ++ix)
                {
                    array_Cg_[idx_constr + ROBOT_JOINTS_NG * ix] = vec_constr_coeffs_tmp_[idx_constr](ix);
                }
            }
            ocp_nlp_constraints_model_set(nlp_config_, nlp_dims_, nlp_in_, nlp_out_, idx, "lg", array_lg_);
            ocp_nlp_constraints_model_set(nlp_config_, nlp_dims_, nlp_in_, nlp_out_, idx, "C", array_Cg_);
        }

        auto start_ocp = std::chrono::high_resolution_clock::now();
        int solver_status = robot_joints_acados_solve(acados_ocp_capsule_);
        auto end_ocp = std::chrono::high_resolution_clock::now();
        if (solver_status == ACADOS_SUCCESS || solver_status == ACADOS_MAXITER)
        {
            ocp_nlp_get_all(nlp_solver_, nlp_in_, nlp_out_, "x", array_x_sol_);
            ocp_nlp_get_all(nlp_solver_, nlp_in_, nlp_out_, "u", array_u_sol_);
            for (unsigned int idx = 0; idx <= config_sip_solver_.ocp.n_hrzn; ++idx)
            {
                std::copy(array_x_sol_ + idx * config_sip_solver_.ocp.nx, array_x_sol_ + (idx + 1) * config_sip_solver_.ocp.nx, vec_joint_state_sol_[idx].data());
            }
            for (unsigned int idx = 1; idx <= config_sip_solver_.ocp.n_hrzn; ++idx)
            {
                ocp_nlp_get_at_stage(nlp_solver_, idx, "C", array_Cg_);
                ocp_nlp_get_at_stage(nlp_solver_, idx, "lg", array_lg_);
                Eigen::Matrix<double, ROBOT_JOINTS_NG, ROBOT_JOINTS_NX> eigen_Cg = Eigen::Map<Eigen::Matrix<double, ROBOT_JOINTS_NG, ROBOT_JOINTS_NX, Eigen::ColMajor>>(array_Cg_);
                Eigen::Vector<double, ROBOT_JOINTS_NG> eigen_lg = Eigen::Map<Eigen::Vector<double, ROBOT_JOINTS_NG>>(array_lg_);
                Eigen::Vector<double, ROBOT_JOINTS_NX> eigen_x_sol = Eigen::Map<Eigen::Vector<double, ROBOT_JOINTS_NX>>(array_x_sol_ + idx * config_sip_solver_.ocp.nx);
                Eigen::Vector<double, ROBOT_JOINTS_NG> g_val = eigen_Cg * eigen_x_sol - eigen_lg;
                collision_constraint_manager_->updateDistanceByConstrGVals(g_val, idx);
            }
        }

        auto start_coll = std::chrono::high_resolution_clock::now();
        for (unsigned int idx = 1; idx <= config_sip_solver_.ocp.n_hrzn; ++idx)
        {
            collision_constraint_manager_->detectCollisionAndUpdatePCSubset(pinocchio_kin_, vec_joint_state_sol_[idx].head(num_joints_), idx, info_iter_current_.time_detailed_collision_detection);
        }
        auto end_coll = std::chrono::high_resolution_clock::now();

        info_iter_current_.time_ocp_solver = std::chrono::duration<double>(end_ocp - start_ocp).count();
        info_iter_current_.time_collision_detection = std::chrono::duration<double>(end_coll - start_coll).count();
        info_iter_current_.time_all_comps = std::chrono::duration<double>(end_coll - start_iter).count();

        if (solver_status == ACADOS_SUCCESS || solver_status == ACADOS_MAXITER)
        {
            return sipoc_ra_utils::SolverStatus::SUCCESS;
        }
        return sipoc_ra_utils::SolverStatus::UL_FAILED;
    }

    sipoc_ra_utils::SolverStatus CarSeatTrajectorySolver::solve()
    {
        this->vec_info_iter_.clear();

        bool flag_cvrg = false;
        for (unsigned int idx_iter = 0; idx_iter < config_sip_solver_.num_iterations; ++idx_iter)
        {
            sipoc_ra_utils::SolverStatus status = this->solveOneIteration(idx_iter >= 1);
            if (status != sipoc_ra_utils::SolverStatus::SUCCESS)
            {
                std::cerr << "idx_iter = " << idx_iter << ". Error in solveOneIteration: " << static_cast<int>(status) << std::endl;
                return status;
            }

            std::transform(vec_joint_state_sol_.begin(), vec_joint_state_sol_.end(), vec_joint_state_last_iter_.begin(), vec_l_infinity_step_.begin(),
                           [](Eigen::VectorXd a, Eigen::VectorXd b)
                           { return (a - b).lpNorm<Eigen::Infinity>(); });
            info_iter_current_.l_infinity_step = *std::max_element(vec_l_infinity_step_.begin(), vec_l_infinity_step_.end());
            this->vec_info_iter_.push_back(info_iter_current_);

            if (info_iter_current_.l_infinity_step <= config_sip_solver_.tol)
            {
                std::cout << "Converged after " << idx_iter + 1 << " iterations." << std::endl;
                flag_cvrg = true;
                break;
            }
            vec_joint_state_last_iter_ = vec_joint_state_sol_;
        }

        if (!flag_cvrg)
        {
            std::cout << "Maximum iterations reached without convergence." << std::endl;
            return sipoc_ra_utils::SolverStatus::UL_MAX_ITERATIONS_REACHED;
        }
        return sipoc_ra_utils::SolverStatus::SUCCESS;
    }

    void CarSeatTrajectorySolver::gatherActiveConstraintsAndShiftOneStep()
    {
        std::vector<int> vec_idx_active_constr;
        for (unsigned int idx_hrzn = 2; idx_hrzn < config_sip_solver_.ocp.n_hrzn; ++idx_hrzn)
        {
            vec_idx_active_constr.clear();
            ocp_nlp_out_get(nlp_config_, nlp_dims_, nlp_out_, idx_hrzn, "lam", array_lam_);
            // Lam: state-input constraint, collision-avoidance constraint, slack variable constraints
            for (unsigned int idx_constr = 0; idx_constr < config_sip_solver_.ocp.num_max_constr; ++idx_constr)
            {
                if (std::abs(array_lam_[idx_constr + ROBOT_JOINTS_NBX + ROBOT_JOINTS_NBU]) >= 1e-3)
                {
                    vec_idx_active_constr.push_back(idx_constr);
                }
            }
            collision_constraint_manager_->shiftActiveConstraints(vec_idx_active_constr, idx_hrzn, idx_hrzn - 1, false);
        }
        // Last stage
        vec_idx_active_constr.clear();
        ocp_nlp_out_get(nlp_config_, nlp_dims_, nlp_out_, config_sip_solver_.ocp.n_hrzn, "lam", array_lam_);
        for (unsigned int idx_constr = 0; idx_constr < config_sip_solver_.ocp.num_max_constr; ++idx_constr)
        {
            if (std::abs(array_lam_[idx_constr + ROBOT_JOINTS_NBX]) >= 1e-3)
            {
                vec_idx_active_constr.push_back(idx_constr);
            }
        }
        collision_constraint_manager_->shiftActiveConstraints(vec_idx_active_constr, config_sip_solver_.ocp.n_hrzn, config_sip_solver_.ocp.n_hrzn - 1, false);
        // Copy the shifted second-to-last stage constraints to the last stage
        int number_active_constr = vec_idx_active_constr.size();
        vec_idx_active_constr.resize(number_active_constr);
        for (int idx_constr = 0; idx_constr < number_active_constr; ++idx_constr)
        {
            vec_idx_active_constr[idx_constr] = idx_constr;
        }
        collision_constraint_manager_->shiftActiveConstraints(vec_idx_active_constr, config_sip_solver_.ocp.n_hrzn - 1, config_sip_solver_.ocp.n_hrzn, false);
    }

    void CarSeatTrajectorySolver::shiftInputStateTrajectory()
    {
        std::memmove(array_x_sol_, array_x_sol_ + config_sip_solver_.ocp.nx, config_sip_solver_.ocp.n_hrzn * config_sip_solver_.ocp.nx * sizeof(double));
        std::memmove(array_u_sol_, array_u_sol_ + config_sip_solver_.ocp.nu, (config_sip_solver_.ocp.n_hrzn - 1) * config_sip_solver_.ocp.nu * sizeof(double));
        for (unsigned int idx = 0; idx < config_sip_solver_.ocp.n_hrzn + 1; ++idx)
        {
            std::copy(array_x_sol_ + idx * config_sip_solver_.ocp.nx, array_x_sol_ + (idx + 1) * config_sip_solver_.ocp.nx, vec_joint_state_last_iter_[idx].data());
        }
        // NOTE: solution in acados solver is not shifted
    }

    void CarSeatTrajectorySolver::printInfoIter() const
    {
        unsigned int num_iter = vec_info_iter_.size();
        double time_total = 0.0;
        std::cout << "# Iteration = " << num_iter << std::endl;
        std::cout << "L1 Step: " << std::endl;
        for (unsigned int idx = 0; idx < num_iter; ++idx)
        {
            std::cout << vec_info_iter_[idx].l_infinity_step << ", ";
        }
        std::cout << std::endl;
        std::cout << "Time per iteration: " << std::endl;
        for (unsigned int idx = 0; idx < num_iter; ++idx)
        {
            std::cout << vec_info_iter_[idx].time_all_comps << ", ";
            time_total += vec_info_iter_[idx].time_all_comps;
        }
        std::cout << std::endl;
        std::cout << "Time for collision detection: " << std::endl;
        for (unsigned int idx = 0; idx < num_iter; ++idx)
        {
            std::cout << vec_info_iter_[idx].time_collision_detection << ", ";
        }
        std::cout << std::endl;
        std::cout << "Time for constraint linearization: " << std::endl;
        for (unsigned int idx = 0; idx < num_iter; ++idx)
        {
            std::cout << vec_info_iter_[idx].time_constr_linearization << ", ";
        }
        std::cout << std::endl;
        std::cout << "Time for OCP solver: " << std::endl;
        for (unsigned int idx = 0; idx < num_iter; ++idx)
        {
            std::cout << vec_info_iter_[idx].time_ocp_solver << ", ";
        }
        std::cout << std::endl;
        std::cout << "Total time for solving trajectory: " << time_total << " seconds." << std::endl;
    }

    double CarSeatTrajectorySolver::evaluateConstraintSatisfaction(bool eval_whole_trajectory)
    {
        ocp_nlp_get_all(nlp_solver_, nlp_in_, nlp_out_, "sl", array_sl_sol_);
        double min_signed_distance = collision_constraint_manager_->evaluateMinimumSignedDistance(pinocchio_kin_, vec_joint_state_sol_[0].head(num_joints_));
        if (!eval_whole_trajectory)
        {
            return min_signed_distance;
        }

        for (unsigned int idx_hrzn = 1; idx_hrzn <= config_sip_solver_.ocp.n_hrzn; ++idx_hrzn)
        {
            double signed_distance_i = collision_constraint_manager_->evaluateMinimumSignedDistance(pinocchio_kin_, vec_joint_state_sol_[idx_hrzn].head(num_joints_));
            if (signed_distance_i < -1e-4)
            {
                double max_sl = *std::max_element(array_sl_sol_ + idx_hrzn * config_sip_solver_.ocp.num_max_constr, array_sl_sol_ + (idx_hrzn + 1) * config_sip_solver_.ocp.num_max_constr);
                if (max_sl <= 1e-4)
                {
                    std::cerr << "The slack variables are close to zero, but the signed distance is negative: " << signed_distance_i << std::endl;
                }
            }
            min_signed_distance = std::min(min_signed_distance, signed_distance_i);
        }
        return min_signed_distance;
    }

    bool CarSeatTrajectorySolver::checkSlackVariableNonZero(double thr)
    {
        ocp_nlp_get_all(nlp_solver_, nlp_in_, nlp_out_, "sl", array_sl_sol_);
        double max_sl = *std::max_element(array_sl_sol_, array_sl_sol_ + (config_sip_solver_.ocp.n_hrzn + 1) * config_sip_solver_.ocp.num_max_constr);
        return max_sl <= thr;
    }

    void CarSeatTrajectorySolver::summarizeInfoIter(sipoc_ra_utils::InfoIterSIPSolver &info_iter) const
    {
        info_iter.l_infinity_step = std::nan("");
        info_iter.time_all_comps = std::accumulate(vec_info_iter_.begin(), vec_info_iter_.end(), 0.0,
                                                   [](double sum, const sipoc_ra_utils::InfoIterSIPSolver &iter_info)
                                                   { return sum + iter_info.time_all_comps; });
        info_iter.time_collision_detection = std::accumulate(vec_info_iter_.begin(), vec_info_iter_.end(), 0.0,
                                                             [](double sum, const sipoc_ra_utils::InfoIterSIPSolver &iter_info)
                                                             { return sum + iter_info.time_collision_detection; });
        info_iter.time_constr_linearization = std::accumulate(vec_info_iter_.begin(), vec_info_iter_.end(), 0.0,
                                                              [](double sum, const sipoc_ra_utils::InfoIterSIPSolver &iter_info)
                                                              { return sum + iter_info.time_constr_linearization; });
        info_iter.time_ocp_solver = std::accumulate(vec_info_iter_.begin(), vec_info_iter_.end(), 0.0,
                                                    [](double sum, const sipoc_ra_utils::InfoIterSIPSolver &iter_info)
                                                    { return sum + iter_info.time_ocp_solver; });
        info_iter.time_detailed_collision_detection.time_coal = std::accumulate(
            vec_info_iter_.begin(), vec_info_iter_.end(), 0.0,
            [](double sum, const sipoc_ra_utils::InfoIterSIPSolver &iter_info)
            { return sum + iter_info.time_detailed_collision_detection.time_coal; });
        info_iter.time_detailed_collision_detection.time_min_element = std::accumulate(
            vec_info_iter_.begin(), vec_info_iter_.end(), 0.0,
            [](double sum, const sipoc_ra_utils::InfoIterSIPSolver &iter_info)
            { return sum + iter_info.time_detailed_collision_detection.time_min_element; });
        info_iter.time_detailed_collision_detection.time_update = std::accumulate(
            vec_info_iter_.begin(), vec_info_iter_.end(), 0.0,
            [](double sum, const sipoc_ra_utils::InfoIterSIPSolver &iter_info)
            { return sum + iter_info.time_detailed_collision_detection.time_update; });
        info_iter.num_iter = vec_info_iter_.size();
        return;
    }
}
