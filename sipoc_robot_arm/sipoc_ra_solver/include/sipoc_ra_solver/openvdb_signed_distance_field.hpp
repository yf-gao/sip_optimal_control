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

#ifndef SIPOC_RA_SOLVER__OpenVDB_SIGNED_DISTANCE_FIELD_HPP
#define SIPOC_RA_SOLVER__OpenVDB_SIGNED_DISTANCE_FIELD_HPP

#include <openvdb/openvdb.h>
#include <openvdb/tools/Interpolation.h>

#include <eigen3/Eigen/Dense>
#include <string>

namespace sipoc_ra_solver
{
    class OpenVDBSignedDistanceField
    {

    public:
        OpenVDBSignedDistanceField(std::string env_path, double voxel_size);
        ~OpenVDBSignedDistanceField();
        void ComputeNormal(const Eigen::Vector3d &point, Eigen::Vector3d &normal) const;

        inline void setEnvTranslation(const Eigen::Vector3d &translation)
        {
            env_translation_ = openvdb::Vec3d(translation.x(), translation.y(), translation.z());
        }

    private:
        double voxel_size_;
        openvdb::FloatGrid::Ptr sdf_ptr_;
        std::unique_ptr<openvdb::tools::GridSampler<openvdb::FloatTree, openvdb::tools::QuadraticSampler>> sampler_;
        openvdb::Vec3d delta_x_, delta_y_, delta_z_;
        openvdb::Vec3d env_translation_;
    };

}

#endif // SIPOC_RA_SOLVER__OpenVDB_SIGNED_DISTANCE_FIELD_HPP
