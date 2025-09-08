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

#ifndef SIPOC_MR_UTILS__DISTANCE_MINIMIZER_POINT_TO_POLYGON_PYBIND_HPP_
#define SIPOC_MR_UTILS__DISTANCE_MINIMIZER_POINT_TO_POLYGON_PYBIND_HPP_

#include "sipoc_mr_utils/distance_minimizer_point_to_polygon.hpp"

#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>

namespace py = pybind11;

namespace sipoc_mr_utils
{
    class DistanceMinimizerPointToPolygonPybind : public DistanceMinimizerPointToPolygon
    {
    public:
        py::array_t<double> argMinDistancePybind(py::array_t<double> param);
        bool ifInsidePolygonPybind(py::array_t<double> param);
        py::array_t<double> halfSpaceAPybind();
        py::array_t<double> halfSpacebPybind();

        DistanceMinimizerPointToPolygonPybind(std::vector<double> vertices_flattened):DistanceMinimizerPointToPolygon(vertices_flattened)
        {
        }
        ~DistanceMinimizerPointToPolygonPybind()=default;
    };
} // namespace sipoc_mr_utils
#endif // SIPOC_MR_UTILS__DISTANCE_MINIMIZER_POINT_TO_POLYGON_PYBIND_HPP_