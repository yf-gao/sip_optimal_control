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

#include "sipoc_mr_utils/pybind/distance_minimizer_point_to_polygon_pybind.hpp"

#include <pybind11/stl.h>

namespace py = pybind11;

namespace sipoc_mr_utils
{
    py::array_t<double> DistanceMinimizerPointToPolygonPybind::argMinDistancePybind(py::array_t<double> param)
    {
        struct DistanceMinimizerSolution<double> sol_object;
        struct DistanceMinimizerParameter<double> param_object;

        auto param_ptr = param.mutable_data();
        if (param.shape(0) != 3)
        {
            throw std::invalid_argument("The length of input vectors is NOT equal to 3.");
        }
        param_object.p_center_ellipsoid << param_ptr[0], param_ptr[1];
        param_object.heading_line = param_ptr[2];

        argMinDistance(param_object, sol_object);
        double pybind_debug_sol[2];
        pybind_debug_sol[0] = sol_object.gamma_polygon.coeff(0);
        pybind_debug_sol[1] = sol_object.gamma_polygon.coeff(1);
        return py::array_t<double>(
            py::buffer_info(
                pybind_debug_sol,
                sizeof(double), // itemsize
                py::format_descriptor<double>::format(),
                1,                                  // ndim
                std::vector<size_t>{2},             // shape
                std::vector<size_t>{sizeof(double)} // strides
                ));
    }

    bool DistanceMinimizerPointToPolygonPybind::ifInsidePolygonPybind(py::array_t<double> param)
    {
        auto param_ptr = param.mutable_data();
        if (param.shape(0) != 3)
        {
            throw std::invalid_argument("The length of input vectors is NOT equal to 3.");
        }
        Eigen::Vector2d pt(param_ptr[1], param_ptr[2]);
        return ifInsidePolygon(param_ptr[0], pt);
    }

    py::array_t<double> DistanceMinimizerPointToPolygonPybind::halfSpaceAPybind()
    {
        // NOTE: Column-major order
        return py::array_t<double>(
            py::buffer_info(
                half_planes_A_.data(),
                sizeof(double), // itemsize
                py::format_descriptor<double>::format(),
                1,                                                                  // ndim
                std::vector<size_t>{half_planes_A_.rows() * half_planes_A_.cols()}, // shape
                std::vector<size_t>{sizeof(double)}                                 // strides
                ));
    }

    py::array_t<double> DistanceMinimizerPointToPolygonPybind::halfSpacebPybind()
    {
        return py::array_t<double>(
            py::buffer_info(
                half_planes_b_.data(),
                sizeof(double), // itemsize
                py::format_descriptor<double>::format(),
                1,                                          // ndim
                std::vector<size_t>{half_planes_b_.size()}, // shape
                std::vector<size_t>{sizeof(double)}         // strides
                ));
    }
}

PYBIND11_MODULE(py_distance_point_polygon, m)
{
    py::class_<sipoc_mr_utils::DistanceMinimizerPointToPolygonPybind>(m, "DistanceMinimizerPointToPolygon")
        .def(py::init<const std::vector<double> &>())
        .def("argMinDistance", &sipoc_mr_utils::DistanceMinimizerPointToPolygonPybind::argMinDistancePybind)
        .def("ifInsidePolygon", &sipoc_mr_utils::DistanceMinimizerPointToPolygonPybind::ifInsidePolygonPybind)
        .def("halfSpaceA", &sipoc_mr_utils::DistanceMinimizerPointToPolygonPybind::halfSpaceAPybind)
        .def("halfSpaceb", &sipoc_mr_utils::DistanceMinimizerPointToPolygonPybind::halfSpacebPybind)
        .def("__del__", [](sipoc_mr_utils::DistanceMinimizerPointToPolygonPybind *instance)
             {
            // Custom destructor is invoked explicitly
            delete instance; }, py::keep_alive<0, 1>());
}
