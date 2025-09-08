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

#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>

#include "sipoc_mr_utils/common_structs.hpp"
namespace py = pybind11;

PYBIND11_MODULE(py_sd_map, m)
{
    py::class_<sipoc_mr_utils::SDFParam>(m, "SDFParam")
        .def(py::init<>())
        .def_readwrite("map_height", &sipoc_mr_utils::SDFParam::map_height)
        .def_readwrite("map_width", &sipoc_mr_utils::SDFParam::map_width)
        .def_readwrite("grid_size", &sipoc_mr_utils::SDFParam::grid_size)
        .def_readwrite("lb_px_grid", &sipoc_mr_utils::SDFParam::lb_px_grid)
        .def_readwrite("lb_py_grid", &sipoc_mr_utils::SDFParam::lb_py_grid)
        .def_readwrite("map_occ_threshold", &sipoc_mr_utils::SDFParam::map_occ_threshold);
}
