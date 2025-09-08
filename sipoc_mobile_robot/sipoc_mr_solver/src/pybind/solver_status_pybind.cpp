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

PYBIND11_MODULE(py_solver_status, m)
{
    /*
    success = 0
    acadosFail = 1
    reachMaxIters = 2
    */
    py::enum_<sipoc_mr_utils::SIPSolverStatus>(m, "SIPSolverStatus", py::arithmetic())
        .value("success", sipoc_mr_utils::SIPSolverStatus::success)
        .value("acadosFail", sipoc_mr_utils::SIPSolverStatus::acadosFail)
        .value("reachMaxIters", sipoc_mr_utils::SIPSolverStatus::reachMaxIters);
}
