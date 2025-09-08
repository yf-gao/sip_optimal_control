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
// Authors: Niels van Duijkeren, Yunfan Gao
//

#include "sipoc_mr_utils/velocity_profile_optimizer.hpp"

#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <pybind11/stl_bind.h>

#include <vector>
#include <iostream>

namespace py = pybind11;

PYBIND11_MAKE_OPAQUE(std::vector<double>)

PYBIND11_MODULE(py_vel_profile_optimizer, m)
{
    py::bind_vector<std::vector<double>>(m, "VectorDouble");
    py::class_<sipoc_mr_utils::VelocityProfileOptimizer>(m, "VelocityProfileOptimizer")
        .def(py::init<>())
        .def("configureOptimizer", &sipoc_mr_utils::VelocityProfileOptimizer::configureOptimizer)
        .def("configureOptimizer",
             [](sipoc_mr_utils::VelocityProfileOptimizer &self, py::array_t<double> shooting_nodes)
             {
                 // Convert numpy array to std::vector<double>
                 std::vector<double> shooting_nodes_vec(shooting_nodes.size());
                 std::memcpy(shooting_nodes_vec.data(), shooting_nodes.data(), shooting_nodes.size() * sizeof(double));
                 self.configureOptimizer(shooting_nodes_vec);
             })
        .def("setRefPath", &sipoc_mr_utils::VelocityProfileOptimizer::setRefPathFlattened)
        .def("setRefPath",
             [](sipoc_mr_utils::VelocityProfileOptimizer &self, py::array_t<double> waypoints)
             {
                 // Convert numpy array to std::vector<double>
                 std::vector<double> waypoints_vec(waypoints.size());
                 std::memcpy(waypoints_vec.data(), waypoints.data(), waypoints.size() * sizeof(double));
                 self.setRefPathFlattened(waypoints_vec);
             })
        .def("computeReferenceTraj", &sipoc_mr_utils::VelocityProfileOptimizer::computeReferenceTraj)
        .def("computeReferenceTraj", [](sipoc_mr_utils::VelocityProfileOptimizer &self, py::array_t<double> robot_state)
             {
            // Convert numpy array to std::vector<double>
            std::vector<double> robot_state_vec(robot_state.size());
            std::memcpy(robot_state_vec.data(), robot_state.data(), robot_state.size() * sizeof(double));
            // Compute the reference trajectory
            self.computeReferenceTraj(robot_state_vec);
            // Convert std::vector<double> to py::array_t<double>
            const std::vector<double>& x_ref = self.referenceTrajConstRef();
            return std::vector<double>(x_ref); });
}
