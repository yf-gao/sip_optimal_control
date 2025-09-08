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

#include "sipoc_ra_solver/openvdb_signed_distance_field.hpp"

#include <CGAL/Polygon_mesh_processing/IO/polygon_mesh_io.h>
#include <CGAL/Simple_cartesian.h>
#include <CGAL/Surface_mesh.h>
#include <CGAL/Side_of_triangle_mesh.h>

#include <openvdb/tools/MeshToVolume.h>

#include <vector>

using namespace sipoc_ra_solver;

OpenVDBSignedDistanceField::OpenVDBSignedDistanceField(std::string env_path, double voxel_size) : voxel_size_(voxel_size)
{
    using Kernel = CGAL::Simple_cartesian<double>;
    CGAL::Surface_mesh<Kernel::Point_3> mesh;
    std::string mesh_path = env_path + ".stl";
    if (!CGAL::Polygon_mesh_processing::IO::read_polygon_mesh(mesh_path, mesh))
    {
        throw std::runtime_error("Failed to read mesh from " + mesh_path);
    }

    std::vector<openvdb::Vec3s> points;    // Vertex positions
    std::vector<openvdb::Vec3I> triangles; // Triangle indices
    for (auto v : mesh.vertices())
    {
        Kernel::Point_3 p = mesh.point(v);
        points.push_back(openvdb::Vec3s(static_cast<float>(p.x()), static_cast<float>(p.y()), static_cast<float>(p.z())));
    }
    openvdb::v11_0::math::Vec3<unsigned int> tri;
    unsigned int num_faces = mesh.number_of_faces();
    unsigned int counter = 0;
    for (auto f : mesh.faces())
    {
        auto vertices = CGAL::vertices_around_face(mesh.halfedge(f), mesh);
        auto it = vertices.begin();
        tri.x() = static_cast<unsigned int>(*it);
        it++;
        tri.y() = static_cast<unsigned int>(*it);
        it++;
        tri.z() = static_cast<unsigned int>(*it);
        it++;
        if (it == vertices.end())
        {
            triangles.push_back(tri); // Only store triangles
            counter += 1;
        }
    }
    std::cout << "Read " << num_faces << " faces and add " << counter << " triangles from mesh." << std::endl;

    // Create OpenVDB signed distance field from the mesh
    openvdb::math::Transform::Ptr xform = openvdb::math::Transform::createLinearTransform(voxel_size_);
    float exteriorBandWidth = 5.0f; // NOTE: Hyperparameter
    float interiorBandWidth = 5.0f;
    sdf_ptr_ = openvdb::tools::meshToSignedDistanceField<openvdb::FloatGrid>(
        *xform, points, triangles, {}, exteriorBandWidth, interiorBandWidth);

    sampler_ = std::make_unique<openvdb::tools::GridSampler<openvdb::FloatTree, openvdb::tools::QuadraticSampler>>(*sdf_ptr_);
    delta_x_ = openvdb::Vec3d(voxel_size_, 0.0, 0.0);
    delta_y_ = openvdb::Vec3d(0.0, voxel_size_, 0.0);
    delta_z_ = openvdb::Vec3d(0.0, 0.0, voxel_size_);
    env_translation_ = openvdb::Vec3d(0.0, 0.0, 0.0);
}

OpenVDBSignedDistanceField::~OpenVDBSignedDistanceField() = default;

void OpenVDBSignedDistanceField::ComputeNormal(const Eigen::Vector3d &point, Eigen::Vector3d &normal) const
{
    openvdb::Vec3d openvdb_coord(point.x(), point.y(), point.z());
    openvdb_coord -= env_translation_;
    normal.x() = (sampler_->wsSample(openvdb_coord + delta_x_) - sampler_->wsSample(openvdb_coord - delta_x_)) / (2 * voxel_size_);
    normal.y() = (sampler_->wsSample(openvdb_coord + delta_y_) - sampler_->wsSample(openvdb_coord - delta_y_)) / (2 * voxel_size_);
    normal.z() = (sampler_->wsSample(openvdb_coord + delta_z_) - sampler_->wsSample(openvdb_coord - delta_z_)) / (2 * voxel_size_);

    if (std::abs(sampler_->wsSample(openvdb_coord)) >= 5.0 * voxel_size_)
    {
        std::cerr << "Warning: The point is too far from the surface. " << std::endl;
        return;
    }
    normal.normalize();
    return;
}
