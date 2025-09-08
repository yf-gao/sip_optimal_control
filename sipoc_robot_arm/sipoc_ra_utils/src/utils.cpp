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

#include "sipoc_ra_utils/utils.hpp"
#include <nlohmann/json.hpp>

#include <iostream>
#include <filesystem>
#include <sstream>
#include <fstream>
#include <string>

namespace sipoc_ra_utils
{
    Eigen::Matrix<double, 3, 3> setRPY(const Eigen::Vector3d rot)
    {
        Eigen::Matrix<double, 3, 3> ret;
        ret = Eigen::AngleAxisd(rot(0), Eigen::Vector3d::UnitX()) * Eigen::AngleAxisd(rot(1), Eigen::Vector3d::UnitY()) * Eigen::AngleAxisd(rot(2), Eigen::Vector3d::UnitZ());
        return ret;
    }

    Eigen::Quaterniond RPT2Quaternion(const Eigen::Vector3d &rpy)
    {
        Eigen::Quaterniond q = Eigen::AngleAxisd(rpy(0), Eigen::Vector3d::UnitX()) *
                               Eigen::AngleAxisd(rpy(1), Eigen::Vector3d::UnitY()) *
                               Eigen::AngleAxisd(rpy(2), Eigen::Vector3d::UnitZ());
        return q;
    }

    Eigen::Matrix3d skew(const Eigen::Vector3d &v)
    {
        Eigen::Matrix3d m;
        m << 0, -v.z(), v.y(),
            v.z(), 0, -v.x(),
            -v.y(), v.x(), 0;
        return m;
    }

    void SE3Manif2Pinocchio(const manif::SE3<double> &se3_manif, pinocchio::SE3 &se3_pinocchio)
    {
        se3_pinocchio.translation() = se3_manif.translation();
        se3_pinocchio.rotation() = se3_manif.rotation();
    }

    void SE3Pinocchio2Manif(const pinocchio::SE3 &se3_pinocchio, manif::SE3<double> &se3_manif)
    {
        se3_manif.translation(se3_pinocchio.translation());
        se3_manif.quat(Eigen::Quaterniond(se3_pinocchio.rotation()));
    }

    void interpolateJointValues(const Eigen::VectorXd &start, const Eigen::VectorXd &end, int num_hrzn, std::vector<Eigen::VectorXd> &interpolated_values)
    {
        for (int i = 0; i <= num_hrzn; ++i)
        {
            double t = static_cast<double>(i) / num_hrzn;
            Eigen::VectorXd joint_values = start + t * (end - start);
            interpolated_values.push_back(joint_values);
        }
    }

    void write_results_to_json(std::map<std::string, VecVariant> &results, std::string folder)
    {
        const char *result_dir = std::getenv("SIPOC_RESULT_DIR");
        if (result_dir == nullptr)
        {
            std::cout << "SIPOC_RESULT_DIR environment variable is not set. Skip writing results to JSON." << std::endl;
            return;
        }

        std::time_t t = std::time(nullptr);
        std::tm local_tm;
#if defined(_WIN32) || defined(_WIN64)
        localtime_s(&local_tm, &t); // Windows secure version
#else
        localtime_r(&t, &local_tm); // POSIX thread-safe version
#endif
        std::ostringstream oss;
        oss << std::put_time(&local_tm, "%Y%m%d_%H%M");
        std::string file_name = std::string(result_dir) + folder + oss.str() + std::string(".json");
        std::cout << "Writing results to " << file_name << std::endl;

        nlohmann::json j;
        for (const auto &[key, value] : results)
        {
            std::cout << "Key: " << key << std::endl;
            if (key.find("num") != std::string::npos || key.find("status") != std::string::npos)
            {
                j[key] = std::get<std::vector<int>>(value);
            }
            else
            {
                j[key] = std::get<std::vector<double>>(value);
            }
        }
        std::ofstream file(file_name);
        if (file.is_open())
        {
            file << j.dump(4);
            file.close();
            std::cout << "Results written to " << file_name << std::endl;
        }
    }
}
