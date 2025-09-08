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

#include "sipoc_mr_utils/signed_distance_map.hpp"

#define SDT_DEAD_RECKONING_IMPLEMENTATION
#include "sipoc_mr_utils/sd_dead_reckoning.h"

#include <boost/algorithm/string/split.hpp>
#include <boost/algorithm/string.hpp>
#include <boost/algorithm/string/iter_find.hpp>
#include <string>
#include <fstream>
#include <filesystem>
#include <algorithm>

namespace sipoc_mr_utils
{
    void SignedDistanceMap::allocateSDMapMemory()
    {
        px_lsp_ = (double *)malloc(sdf_param_.map_width * sizeof(double));
        py_lsp_ = (double *)malloc(sdf_param_.map_height * sizeof(double));
        if (if_polygon_)
        {
            ptr_gamma_grids_ = (double *)malloc(2 * robot_param_.num_gamma_grids * sizeof(double));
        }
        else
        {
            ptr_gamma_grids_ = (double *)malloc(robot_param_.num_gamma_grids * sizeof(double));
        }
        ptr_occupancy_map_ = (unsigned char *)malloc(sdf_param_.map_width * sdf_param_.map_height * sizeof(unsigned char));
        ptr_sdf_map = (double *)malloc(sdf_param_.map_width * sdf_param_.map_height * sizeof(double));
        ptr_px_field = (int *)malloc(sdf_param_.map_width * sdf_param_.map_height * sizeof(int));
        ptr_py_field = (int *)malloc(sdf_param_.map_width * sdf_param_.map_height * sizeof(int));
        sdf_at_gamma_grids_.resize(robot_param_.num_gamma_grids);
    }

    void SignedDistanceMap::deallocateSDMapMemory()
    {
        free(px_lsp_);
        free(py_lsp_);
        free(ptr_gamma_grids_);
        free(ptr_occupancy_map_);
        free(ptr_sdf_map);
        free(ptr_px_field);
        free(ptr_py_field);
    }

    void SignedDistanceMap::setArrayValues()
    {
        if (if_polygon_)
        {
            double weight = 0, edges_length_sum = 0.;
            int idx_next_vertex, idx_grid, num_grids_sum = 0;
            std::vector<int> num_grids_per_vertex = {};
            std::vector<double> edge_length = {};
            for (int idx_vertex = 0; idx_vertex < num_polygon_vertices_; ++idx_vertex)
            {
                idx_next_vertex = (idx_vertex + 1) % num_polygon_vertices_;
                edge_length.push_back(std::hypot(robot_param_.vertices_flattened[2 * idx_vertex], robot_param_.vertices_flattened[2 * idx_vertex + 1]));
                edges_length_sum += edge_length.back();
            }
            for (int idx_vertex = 0; idx_vertex < num_polygon_vertices_; ++idx_vertex)
            {
                num_grids_per_vertex.push_back(std::max(2, static_cast<int>(std::round(edge_length[idx_vertex] / edges_length_sum * robot_param_.num_gamma_grids))));
                num_grids_sum += num_grids_per_vertex.back();
            }
            std::vector<int>::iterator iter_max = std::max_element(num_grids_per_vertex.begin(), num_grids_per_vertex.end());
            *iter_max += robot_param_.num_gamma_grids - num_grids_sum;

            idx_grid = 0;
            for (int idx_vertex = 0; idx_vertex < num_polygon_vertices_; ++idx_vertex)
            {
                idx_next_vertex = (idx_vertex + 1) % num_polygon_vertices_;
                for (int i = 0; i < num_grids_per_vertex[idx_vertex]; ++i)
                {
                    weight = (1.0 * i) / num_grids_per_vertex[idx_vertex];
                    ptr_gamma_grids_[2 * idx_grid] = (1 - weight) * robot_param_.vertices_flattened[2 * idx_vertex] + weight * robot_param_.vertices_flattened[2 * idx_next_vertex];
                    ptr_gamma_grids_[2 * idx_grid + 1] = (1 - weight) * robot_param_.vertices_flattened[2 * idx_vertex + 1] + weight * robot_param_.vertices_flattened[2 * idx_next_vertex + 1];
                    idx_grid += 1;
                }
            }
        }
        else
        {
            double delta_step = 2 * robot_param_.robot_capsule_half_length / (robot_param_.num_gamma_grids - 1);
            for (int i = 0; i < robot_param_.num_gamma_grids; ++i)
            {
                ptr_gamma_grids_[i] = -robot_param_.robot_capsule_half_length + i * delta_step;
            }
        }
        for (int i = 0; i < sdf_param_.map_width; ++i)
        {
            px_lsp_[i] = sdf_param_.lb_px_grid + i * sdf_param_.grid_size;
        }
        for (int i = 0; i < sdf_param_.map_height; ++i)
        {
            py_lsp_[i] = sdf_param_.lb_py_grid + i * sdf_param_.grid_size;
        }
    }

    void SignedDistanceMap::loadOccupancyMap(std::string sdf_file_str, struct SDFParam &sdf_param)
    {
        std::ifstream file;
        if (!std::filesystem::exists(sdf_file_str))
        {
            throw std::runtime_error(sdf_file_str + std::string(": File does not exist"));
        }

        bool flag_same_map_shape = setMapConfiguration(sdf_param);
        if (!flag_same_map_shape)
        {
            deallocateSDMapMemory();
            allocateSDMapMemory();
            setArrayValues();
        }

        file.open(sdf_file_str, std::ifstream::in);
        std::string m_line;
        std::getline(file, m_line);
        std::vector<std::string> m_line_split;

        for (int idx_height = 0; idx_height < sdf_param_.map_height; idx_height++)
        {
            std::getline(file, m_line);
            m_line_split.clear();
            boost::split(m_line_split, m_line, boost::is_any_of(","));
            for (int idx_width = 0; idx_width < sdf_param_.map_width; idx_width++)
            {
                ptr_occupancy_map_[idx_width + idx_height * sdf_param_.map_width] = 99 * std::stoi(m_line_split[idx_width]);
            }
        }
    }

    void SignedDistanceMap::computeSignedDistanceMap()
    {
        sdt_dead_reckoning(sdf_param_.map_width, sdf_param_.map_height, 1, ptr_occupancy_map_, ptr_sdf_map, ptr_px_field, ptr_py_field);
    }

    void SignedDistanceMap::computeSignedDistanceMap(unsigned char threshold)
    {
        sdt_dead_reckoning(sdf_param_.map_width, sdf_param_.map_height, threshold, ptr_occupancy_map_, ptr_sdf_map, ptr_px_field, ptr_py_field);
    }

    bool SignedDistanceMap::world2grid_indices_floor(const Eigen::Vector2d &coords, Eigen::Vector<int, 2> &grid_indices, Eigen::Vector2d &weights)
    {
        // NOTE: No errors when the grid indices is outside of the map
        double grid_size = sdf_param_.grid_size;
        grid_indices(0) = static_cast<int>(std::floor((coords.coeff(0) - sdf_param_.lb_px_grid) / grid_size));
        grid_indices(1) = static_cast<int>(std::floor((coords.coeff(1) - sdf_param_.lb_py_grid) / grid_size));
        if (grid_indices.coeff(0) < 0 || grid_indices.coeff(1) < 0 || grid_indices.coeff(0) >= sdf_param_.map_width - 1 || grid_indices.coeff(1) >= sdf_param_.map_height - 1)
        {
            return false;
        }
        weights(0) = (px_lsp_[grid_indices.coeff(0) + 1] - coords.coeff(0)) / grid_size;
        weights(1) = (py_lsp_[grid_indices.coeff(1) + 1] - coords.coeff(1)) / grid_size;
        return true;
    }

    void SignedDistanceMap::world2grid_indices_round(const Eigen::Vector2d &coords, Eigen::Vector<int, 2> &grid_indices)
    {
        grid_indices(0) = static_cast<int>(std::round((coords.coeff(0) - sdf_param_.lb_px_grid) / sdf_param_.grid_size));
        grid_indices(1) = static_cast<int>(std::round((coords.coeff(1) - sdf_param_.lb_py_grid) / sdf_param_.grid_size));
    }

    void SignedDistanceMap::getClosestObsAtCoords(const Eigen::Vector2d &coords, Eigen::Vector2d &obs_coords)
    {
        world2grid_indices_round(coords, temp_vector2i_);
        temp_vector2i_(0) = std::clamp(temp_vector2i_.coeff(0), 0, sdf_param_.map_width - 1);
        temp_vector2i_(1) = std::clamp(temp_vector2i_.coeff(1), 0, sdf_param_.map_height - 1);
        // query the grid indices for the closest obstacle
        temp_vector2i_2_(0) = ptr_px_field[temp_vector2i_.coeff(1) * sdf_param_.map_width + temp_vector2i_.coeff(0)];
        temp_vector2i_2_(1) = ptr_py_field[temp_vector2i_.coeff(1) * sdf_param_.map_width + temp_vector2i_.coeff(0)];
        grid2world_coordinate(temp_vector2i_2_, obs_coords);
        return;
    }

    void SignedDistanceMap::getClosestObsAtGamma(const manif::SE2<double> &current_pose, double gamma, Eigen::Vector2d &obs_coords)
    {
        temp_vector2d_(0) = current_pose.x() + gamma * current_pose.real();
        temp_vector2d_(1) = current_pose.y() + gamma * current_pose.imag();
        getClosestObsAtCoords(temp_vector2d_, obs_coords);
        return;
    }

    void SignedDistanceMap::getClosestObsAtGamma(const manif::SE2<double> &current_pose, const Eigen::Vector2d &gamma, Eigen::Vector2d &obs_coords)
    {
        temp_vector2d_ = current_pose.translation() + current_pose.rotation() * gamma;
        getClosestObsAtCoords(temp_vector2d_, obs_coords);
        return;
    }

    void SignedDistanceMap::evaluateDistanceAtAllGammas(const manif::SE2<double> &current_pose)
    {
        for (int i = 0; i < robot_param_.num_gamma_grids; ++i)
        {
            if (if_polygon_)
            {
                temp_vector2d_2_(0) = ptr_gamma_grids_[2 * i];
                temp_vector2d_2_(1) = ptr_gamma_grids_[2 * i + 1];
                temp_vector2d_ = current_pose.translation() + current_pose.rotation() * temp_vector2d_2_;
            }
            else
            {
                temp_vector2d_(0) = current_pose.x() + ptr_gamma_grids_[i] * current_pose.real();
                temp_vector2d_(1) = current_pose.y() + ptr_gamma_grids_[i] * current_pose.imag();
            }
            world2grid_indices_round(temp_vector2d_, temp_vector2i_);
            temp_vector2i_(0) = std::clamp(temp_vector2i_.coeff(0), 0, sdf_param_.map_width - 1);
            temp_vector2i_(1) = std::clamp(temp_vector2i_.coeff(1), 0, sdf_param_.map_height - 1);
            sdf_at_gamma_grids_[i] = ptr_sdf_map[temp_vector2i_.coeff(1) * sdf_param_.map_width + temp_vector2i_.coeff(0)];
        }
    }

    void SignedDistanceMap::argMinDistanceOverGamma(const manif::SE2<double> &current_pose, bool ifBilinearInterp, MinimizerResult<double> &result)
    {
        evaluateDistanceAtAllGammas(current_pose);
        std::vector<double>::iterator iter_minimizer = std::min_element(sdf_at_gamma_grids_.begin(), sdf_at_gamma_grids_.end());
        if (ifBilinearInterp)
        {
            evaluateDistanceSelectivelyBilinearInterp(current_pose, *iter_minimizer + 1.0, result);
        }
        else
        {
            if (if_polygon_)
            {
                result.minimizer[0] = ptr_gamma_grids_[2 * std::distance(sdf_at_gamma_grids_.begin(), iter_minimizer)];
                result.minimizer[1] = ptr_gamma_grids_[2 * std::distance(sdf_at_gamma_grids_.begin(), iter_minimizer) + 1];
            }
            else
            {
                result.minimizer[0] = ptr_gamma_grids_[std::distance(sdf_at_gamma_grids_.begin(), iter_minimizer)];
            }
            result.min_value = (*iter_minimizer) * sdf_param_.grid_size;
        }
        return;
    }

    void SignedDistanceMap::evaluateDistanceSelectivelyBilinearInterp(const manif::SE2<double> &current_pose, double threshold, MinimizerResult<double> &result)
    {
        double min_value = threshold;
        double weighted_distance;
        for (int i = 0; i < robot_param_.num_gamma_grids; ++i)
        {
            if (sdf_at_gamma_grids_[i] <= threshold)
            {
                if (if_polygon_)
                {
                    temp_vector2d_2_(0) = ptr_gamma_grids_[2 * i];
                    temp_vector2d_2_(1) = ptr_gamma_grids_[2 * i + 1];
                    temp_vector2d_ = current_pose.translation() + current_pose.rotation() * temp_vector2d_2_;
                }
                else
                {
                    temp_vector2d_(0) = current_pose.x() + ptr_gamma_grids_[i] * current_pose.real();
                    temp_vector2d_(1) = current_pose.y() + ptr_gamma_grids_[i] * current_pose.imag();
                }
                bool flag_within_map = world2grid_indices_floor(temp_vector2d_, temp_vector2i_, temp_vector2d_2_);
                if (not flag_within_map)
                {
                    continue;
                }
                weighted_distance = temp_vector2d_2_.coeff(0) * temp_vector2d_2_.coeff(1) * ptr_sdf_map[temp_vector2i_.coeff(1) * sdf_param_.map_width + temp_vector2i_.coeff(0)];
                weighted_distance += (1.0 - temp_vector2d_2_.coeff(0)) * temp_vector2d_2_.coeff(1) * ptr_sdf_map[temp_vector2i_.coeff(1) * sdf_param_.map_width + temp_vector2i_.coeff(0) + 1];
                weighted_distance += temp_vector2d_2_.coeff(0) * (1.0 - temp_vector2d_2_.coeff(1)) * ptr_sdf_map[(temp_vector2i_.coeff(1) + 1) * sdf_param_.map_width + temp_vector2i_.coeff(0)];
                weighted_distance += (1.0 - temp_vector2d_2_.coeff(0)) * (1.0 - temp_vector2d_2_.coeff(1)) * ptr_sdf_map[(temp_vector2i_.coeff(1) + 1) * sdf_param_.map_width + temp_vector2i_.coeff(0) + 1];
                if (weighted_distance <= min_value)
                {
                    min_value = weighted_distance;
                    if (if_polygon_)
                    {
                        result.minimizer[0] = ptr_gamma_grids_[2 * i];
                        result.minimizer[1] = ptr_gamma_grids_[2 * i + 1];
                    }
                    else
                    {
                        result.minimizer[0] = ptr_gamma_grids_[i];
                    }
                }
            }
        }
        result.min_value = min_value * sdf_param_.grid_size;
    }

    bool SignedDistanceMap::setMapConfiguration(struct SDFParam &sdf_param)
    {
        sdf_param_.lb_px_grid = sdf_param.lb_px_grid;
        sdf_param_.lb_py_grid = sdf_param.lb_py_grid;
        bool flag_same_shape = (sdf_param_.map_width == sdf_param.map_width) && (sdf_param_.map_height == sdf_param.map_height);
        sdf_param_.map_width = sdf_param.map_width;
        sdf_param_.map_height = sdf_param.map_height;
        sdf_param_.grid_size = sdf_param.grid_size;
        return flag_same_shape;
    }
}
