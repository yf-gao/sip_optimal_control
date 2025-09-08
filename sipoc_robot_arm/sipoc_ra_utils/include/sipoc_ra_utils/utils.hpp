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

#ifndef SIPOC_RA_SUPPORT__UTILS_HPP_
#define SIPOC_RA_SUPPORT__UTILS_HPP_

#include <eigen3/Eigen/Dense>
#include <manif/manif.h>
#include <pinocchio/spatial/se3.hpp>

#include <variant>
#include <vector>
#include <map>

namespace sipoc_ra_utils
{
    Eigen::Matrix<double, 3, 3> setRPY(const Eigen::Vector3d rot);
    Eigen::Quaterniond RPT2Quaternion(const Eigen::Vector3d &rpy);
    Eigen::Matrix3d skew(const Eigen::Vector3d &v);
    void SE3Manif2Pinocchio(const manif::SE3<double> &se3_manif, pinocchio::SE3 &se3_pinocchio);
    void SE3Pinocchio2Manif(const pinocchio::SE3 &se3_pinocchio, manif::SE3<double> &se3_manif);
    void interpolateJointValues(const Eigen::VectorXd &start, const Eigen::VectorXd &end, int num_hrzn, std::vector<Eigen::VectorXd> &interpolated_values);

    using VecVariant = std::variant<std::vector<double>, std::vector<int>>;
    void write_results_to_json(std::map<std::string, VecVariant> &results, std::string folder);
}

#endif // SIPOC_RA_SUPPORT__UTILS_HPP_
