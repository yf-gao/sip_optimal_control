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

#include "sipoc_mr_utils/pybind/distance_minimizer_ellipsoid_to_polygon_pybind.hpp"

#include <pybind11/stl.h>

namespace py = pybind11;

namespace sipoc_mr_utils
{
    py::array_t<double> DistanceMinimizerEllipsoidToPolygonPybind::argMinDistancePybind(py::array_t<double> param)
    {
        struct DistanceMinimizerSolution<double> sol_object;
        struct DistanceMinimizerParameter<double> param_object;

        auto param_ptr = param.mutable_data();
        if (param.shape(0) != 7)
        {
            throw std::invalid_argument("The length of input vectors is NOT equal to 3.");
        }
        param_object.shape_matrix_ellipsoid << param_ptr[0], param_ptr[1], param_ptr[2], param_ptr[3];
        param_object.p_center_ellipsoid << param_ptr[4], param_ptr[5];
        param_object.heading_line = param_ptr[6];

        argMinDistance(param_object, sol_object);
        double pybind_sol[5];
        pybind_sol[0] = sol_object.gamma_polygon.coeff(0);
        pybind_sol[1] = sol_object.gamma_polygon.coeff(1);
        pybind_sol[2] = sol_object.delta_p_ellipsoid.coeff(0);
        pybind_sol[3] = sol_object.delta_p_ellipsoid.coeff(1);
        pybind_sol[4] = sol_object.squared_distance;
        return py::array_t<double>(
            py::buffer_info(
                pybind_sol,
                sizeof(double), // itemsize
                py::format_descriptor<double>::format(),
                1,                                  // ndim
                std::vector<size_t>{5},             // shape
                std::vector<size_t>{sizeof(double)} // strides
                ));
    }
}

PYBIND11_MODULE(py_distance_ellip_polygon, m)
{
    py::class_<sipoc_mr_utils::DistanceMinimizerEllipsoidToPolygonPybind>(m, "DistanceMinimizerEllipsoidToPolygon")
        .def(py::init<const std::vector<double> &>())
        .def("argMinDistance", &sipoc_mr_utils::DistanceMinimizerEllipsoidToPolygonPybind::argMinDistancePybind)
        .def("__del__", [](sipoc_mr_utils::DistanceMinimizerEllipsoidToPolygonPybind *instance)
             {
            // Custom destructor is invoked explicitly
            delete instance; }, py::keep_alive<0, 1>());
}
