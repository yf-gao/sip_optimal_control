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

#ifndef SIPOC_MR_UTILS__SIGNED_DISTANCE_MAP_HPP_
#define SIPOC_MR_UTILS__SIGNED_DISTANCE_MAP_HPP_

#include "sipoc_mr_utils/common_structs.hpp"

#include <Eigen/Dense>
#include <manif/SE2.h>

#include <vector>
#include <string>
/*
NOTE: Regarding the occupancy map: 0 stands for free and 99 stands for occupied. (see loadOccupancyMap Function)
The default threshold of being regarding as being occupied is 1 (see ifGridOccupied and computeSignedDistanceMap).
*/

namespace sipoc_mr_utils
{
    class SignedDistanceMap
    {
    public:
        SignedDistanceMap()
        {
            setDefaultParamValue();
            num_polygon_vertices_ = robot_param_.vertices_flattened.size() / 2;
            if (if_polygon_ && num_polygon_vertices_ < 3)
            {
                throw std::runtime_error("if_polygon_=True, Number of polygon vertices must be at least 3.");
            }
            allocateSDMapMemory();
            setArrayValues();
        }

        SignedDistanceMap(const SDFParam &sdf_param, const RobotParam<double> &robot_param)
        {
            sdf_param_ = sdf_param;
            robot_param_ = robot_param;
            num_polygon_vertices_ = robot_param_.vertices_flattened.size() / 2;
            if (if_polygon_ && num_polygon_vertices_ < 3)
            {
                throw std::runtime_error("if_polygon_=True, Number of polygon vertices must be at least 3.");
            }
            allocateSDMapMemory();
            setArrayValues();
        }

        SignedDistanceMap(const std::vector<double> &polygon_vertices)
        {
            robot_param_.vertices_flattened = polygon_vertices;
            setDefaultParamValue();
            num_polygon_vertices_ = robot_param_.vertices_flattened.size() / 2;
            if (if_polygon_ && num_polygon_vertices_ < 3)
            {
                throw std::runtime_error("if_polygon_=True, Number of polygon vertices must be at least 3.");
            }
            allocateSDMapMemory();
            setArrayValues();
        }

        ~SignedDistanceMap()
        {
            deallocateSDMapMemory();
        }

        double *ptr_sdf_map = nullptr;
        int *ptr_px_field = nullptr;
        int *ptr_py_field = nullptr;

        bool world2grid_indices_floor(const Eigen::Vector2d &coords, Eigen::Vector<int, 2> &grid_indices, Eigen::Vector2d &weights);
        void world2grid_indices_round(const Eigen::Vector2d &coords, Eigen::Vector<int, 2> &grid_indices);
        inline void grid2world_coordinate(const Eigen::Vector<int, 2> &grid_indices, Eigen::Vector2d &coords)
        {
            // NOTE: grid indices [px, py]
            coords << px_lsp_[grid_indices.coeff(0)], py_lsp_[grid_indices.coeff(1)];
        }
        inline void updateOccupancyMap(const std::vector<signed char> &map)
        {
            int index = 0;
            for (std::vector<signed char>::const_iterator it = map.begin(); it != map.end(); ++it, ++index)
            {
                ptr_occupancy_map_[index] = static_cast<unsigned char>(*it);
            }
        }
        void computeSignedDistanceMap();
        void computeSignedDistanceMap(unsigned char threshold);
        void loadOccupancyMap(std::string sdf_file_str, struct SDFParam &sdf_param);
        void evaluateDistanceAtAllGammas(const manif::SE2<double> &current_pose);
        void argMinDistanceOverGamma(const manif::SE2<double> &current_pose, bool ifBilinearInterp, MinimizerResult<double> &result);
        void getClosestObsAtCoords(const Eigen::Vector2d &coords, Eigen::Vector2d &obs_coords);
        void getClosestObsAtGamma(const manif::SE2<double> &current_pose, double gamma, Eigen::Vector2d &obs_coords);
        void getClosestObsAtGamma(const manif::SE2<double> &current_pose, const Eigen::Vector2d &gamma, Eigen::Vector2d &obs_coords);
        void evaluateDistanceSelectivelyBilinearInterp(const manif::SE2<double> &current_pose, double threshold, MinimizerResult<double> &result);

        inline double get_grid_size()
        {
            return sdf_param_.grid_size;
        }

        inline int get_map_width()
        {
            return sdf_param_.map_width;
        }

        inline int get_map_height()
        {
            return sdf_param_.map_height;
        }

        inline double get_sd_at_grid_indices(int px_grid_idx, int py_grid_idx)
        {
            return ptr_sdf_map[px_grid_idx + py_grid_idx * sdf_param_.map_width];
        }

        inline bool ifGridOccupied(const Eigen::Vector<int, 2> &grid, unsigned char threshold)
        {
            return ptr_occupancy_map_[grid.coeff(0) + grid.coeff(1) * sdf_param_.map_width] >= threshold;
        }

    private:
        unsigned char *ptr_occupancy_map_ = nullptr;
        double *ptr_gamma_grids_ = nullptr;
        double *px_lsp_ = nullptr;
        double *py_lsp_ = nullptr;
        std::vector<double> sdf_at_gamma_grids_;
        struct RobotParam<double> robot_param_ =
        {
        };
        Eigen::Vector2d temp_vector2d_, temp_vector2d_2_;
        Eigen::Vector<int, 2> temp_vector2i_, temp_vector2i_2_;
        int num_polygon_vertices_ = 0;
        static constexpr bool if_polygon_ = POLYGON_FLAG_VALUE;
        void setArrayValues();
        bool setMapConfiguration(struct SDFParam &sdf_param);
        void allocateSDMapMemory();
        void deallocateSDMapMemory();

        inline void setDefaultParamValue()
        {
            sdf_param_.map_width = 800;
            sdf_param_.map_height = 200;
            sdf_param_.grid_size = 0.02;
            sdf_param_.lb_px_grid = -7.99;
            sdf_param_.lb_py_grid = -1.99;
            robot_param_.num_gamma_grids = 50;
            robot_param_.robot_capsule_half_length = 0.5;
        }

    protected:
        struct SDFParam sdf_param_ = {};
    };

} // namespace sipoc_mr_utils
#endif // SIPOC_MR_UTILS__SIGNED_DISTANCE_MAP_HPP_
