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

#include "sipoc_ra_utils/spline_wrapper_8dim.hpp"

#include <gsl/gsl_blas.h>

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <iostream>


namespace sipoc_ra_utils
{

    void SplineWrapper8Dim::configure(uint32_t degree, const ControlVector &controls)
    {
        const uint32_t ncontrols = controls.size();
        if (ncontrols <= degree)
        {
            throw std::runtime_error("Number of controls must be at least degree + 1");
        }
        const uint32_t nbreakpts = computeNumBreakPoints(ncontrols, degree);

        allocateInterface(interface_, degree, nbreakpts, ncontrols);
        gsl_bspline_knots_uniform(0.0, 1.0, interface_.workspace.get());
        setControlPoints(controls);
    }

    void SplineWrapper8Dim::configure(const KnotVector &knots, const ControlVector &controls)
    {
        const uint32_t nbreakpts = knots.size();
        const uint32_t ncontrols = controls.size();
        const uint32_t degree = computeDegree(nbreakpts, ncontrols);

        allocateInterface(interface_, degree, nbreakpts, ncontrols);
        setKnotPoints(knots);
        setControlPoints(controls);
    }

    auto SplineWrapper8Dim::getDegree() const -> uint32_t
    {
        if (!isInitialized())
        {
            throw std::runtime_error("Spline not initialized.");
        }

        return gsl_bspline_order(interface_.workspace.get()) - 1;
    }

    auto SplineWrapper8Dim::getNumControlPoints() const -> uint32_t
    {
        if (!isInitialized())
        {
            throw std::runtime_error("Spline not initialized.");
        }

        return gsl_bspline_ncoeffs(interface_.workspace.get());
    }

    auto SplineWrapper8Dim::getNumKnotPoints() const -> uint32_t
    {
        if (!isInitialized())
        {
            throw std::runtime_error("Spline not initialized.");
        }

        return gsl_bspline_nbreak(interface_.workspace.get());
    }

    void SplineWrapper8Dim::setKnotPoints(const KnotVector &knots)
    {
        if (!isInitialized())
        {
            throw std::runtime_error("Spline not initialized.");
        }

        const uint32_t ndata = knots.size();
        const uint32_t nbreakpts = gsl_bspline_nbreak(interface_.workspace.get());
        if (ndata != nbreakpts)
        {
            throw std::runtime_error("Mismatch in number of knot points (expected: " + std::to_string(nbreakpts) +
                                     ", got: " + std::to_string(ndata) + ").");
        }

        // Test if the knot points are strictly increasing
        if (!std::is_sorted(knots.begin(), knots.end()))
        {
            throw std::runtime_error("Knot points must be strictly increasing.");
        }

        gsl_vector_const_view knots_view = gsl_vector_const_view_array(knots.data(), nbreakpts);
        gsl_bspline_knots(&knots_view.vector, interface_.workspace.get());
    }

    void SplineWrapper8Dim::setControlPoints(const ControlVector &controls)
    {
        if (!isInitialized())
        {
            throw std::runtime_error("Spline not initialized.");
        }

        const uint32_t ndata = controls.size();
        const uint32_t ncontrols = gsl_bspline_ncoeffs(interface_.workspace.get());
        if (ndata != ncontrols)
        {
            throw std::runtime_error("Mismatch in number of control points (expected: " + std::to_string(ncontrols) +
                                     ", got: " + std::to_string(ncontrols) + ").");
        }

        for (uint32_t i = 0; i < ncontrols; ++i)
        {
            for (uint32_t j = 0; j < ny; ++j)
            {
                gsl_matrix_set(interface_.controls.get(), j, i, controls[i](j));
            }
        }
    }

    auto SplineWrapper8Dim::getKnotPoints() const -> KnotVector
    {
        if (!isInitialized())
        {
            throw std::runtime_error("Spline not initialized.");
        }
        const uint32_t nbreakpts = gsl_bspline_nbreak(interface_.workspace.get());

        KnotVector ret = KnotVector(nbreakpts, 0.0);
        for (uint32_t i = 0; i < nbreakpts; ++i)
        {
            ret[i] = gsl_bspline_breakpoint(i, interface_.workspace.get());
        }

        return ret;
    }

    auto SplineWrapper8Dim::getControlPoints() const -> ControlVector
    {
        if (!isInitialized())
        {
            throw std::runtime_error("Spline not initialized.");
        }

        const uint32_t ncontrols = gsl_bspline_ncoeffs(interface_.workspace.get());

        ControlVector ret(ncontrols, Control::Zero());
        for (uint32_t i = 0; i < ncontrols; ++i)
        {
            for (uint32_t j = 0; j < ny; ++j)
            {
                ret[i](j) = gsl_matrix_get(interface_.controls.get(), j, i);
            }
        }

        return ret;
    }

    auto SplineWrapper8Dim::eval(double x) const -> Output
    {
        if (!isInitialized())
        {
            throw std::runtime_error("Spline not initialized.");
        }

        size_t istart, iend;
        gsl_bspline_eval_nonzero(x, interface_.B.get(), &istart, &iend, interface_.workspace.get());
        gsl_matrix_const_view controls_view =
            gsl_matrix_const_submatrix(interface_.controls.get(), 0, istart, ny, iend - istart + 1);

        Output result = Output::Zero();
        gsl_vector_view result_view = gsl_vector_view_array(result.data(), ny);

        gsl_blas_dgemv(CblasNoTrans, 1, &controls_view.matrix, interface_.B.get(), 0, &result_view.vector);

        return result;
    }

    auto SplineWrapper8Dim::p(double x) const -> OutputTangent
    {
        Output y;
        return p(x, y);
    }

    auto SplineWrapper8Dim::p(double x, Output &y) const -> OutputTangent
    {
        if (!isInitialized())
        {
            throw std::runtime_error("Spline not initialized.");
        }

        if (getDegree() == 0)
        {
            return OutputTangent::Zero();
        }

        size_t istart, iend;
        gsl_bspline_deriv_eval_nonzero(x, 1, interface_.dB.get(), &istart, &iend, interface_.workspace.get());
        gsl_vector_const_view B_view = gsl_matrix_const_column(interface_.dB.get(), 0);
        gsl_vector_const_view dB_view = gsl_matrix_const_column(interface_.dB.get(), 1);
        gsl_matrix_const_view controls_view =
            gsl_matrix_const_submatrix(interface_.controls.get(), 0, istart, ny, iend - istart + 1);

        OutputTangent result = OutputTangent::Zero();
        gsl_vector_view result_view = gsl_vector_view_array(result.data(), ny);

        gsl_blas_dgemv(CblasNoTrans, 1, &controls_view.matrix, &dB_view.vector, 0, &result_view.vector);

        gsl_vector_view y_view = gsl_vector_view_array(y.data(), ny);
        gsl_blas_dgemv(CblasNoTrans, 1, &controls_view.matrix, &B_view.vector, 0, &y_view.vector);

        return result;
    }

    auto SplineWrapper8Dim::pp(double x) const -> OutputTangent
    {
        Output y;
        OutputTangent p;
        return pp(x, y, p);
    }

    auto SplineWrapper8Dim::pp(double x, Output &y, OutputTangent &p) const -> OutputTangent
    {
        if (!isInitialized())
        {
            throw std::runtime_error("Spline not initialized.");
        }

        if (getDegree() < 2)
        {
            return OutputTangent::Zero();
        }

        size_t istart, iend;
        gsl_bspline_deriv_eval_nonzero(x, 2, interface_.dB.get(), &istart, &iend, interface_.workspace.get());
        gsl_vector_const_view B_view = gsl_matrix_const_column(interface_.dB.get(), 0);
        gsl_vector_const_view dB_view = gsl_matrix_const_column(interface_.dB.get(), 1);
        gsl_vector_const_view ddB_view = gsl_matrix_const_column(interface_.dB.get(), 2);
        gsl_matrix_const_view controls_view =
            gsl_matrix_const_submatrix(interface_.controls.get(), 0, istart, ny, iend - istart + 1);

        OutputTangent result = OutputTangent::Zero();
        gsl_vector_view result_view = gsl_vector_view_array(result.data(), ny);

        gsl_blas_dgemv(CblasNoTrans, 1, &controls_view.matrix, &ddB_view.vector, 0, &result_view.vector);

        gsl_vector_view y_view = gsl_vector_view_array(y.data(), ny);
        gsl_blas_dgemv(CblasNoTrans, 1, &controls_view.matrix, &B_view.vector, 0, &y_view.vector);

        gsl_vector_view p_view = gsl_vector_view_array(p.data(), ny);
        gsl_blas_dgemv(CblasNoTrans, 1, &controls_view.matrix, &dB_view.vector, 0, &p_view.vector);

        return result;
    }

    auto SplineWrapper8Dim::curvature(double x) const -> double
    {
        if (!isInitialized())
        {
            throw std::runtime_error("Spline not initialized.");
        }

        Output yx;
        OutputTangent px;
        OutputTangent ppx = pp(x, yx, px);

        // sqrt(||g'||^2*||g''||^2-<g',g''>^2) / ||g'||^3
        double n2px = px.squaredNorm();
        double npx = sqrt(n2px);
        double n2ppx = ppx.squaredNorm();
        double pxdppx = px.dot(ppx);
        return std::sqrt(std::abs(n2px * n2ppx - pxdppx * pxdppx)) / (n2px * npx);
    }

    auto SplineWrapper8Dim::regressor(double x) const -> Regressor
    {
        if (!isInitialized())
        {
            throw std::runtime_error("Spline not initialized.");
        }

        const uint32_t degree = gsl_bspline_order(interface_.workspace.get()) - 1;

        RegressorCoeffs coeffs(degree + 1, 0.0);
        gsl_vector_view basis_view = gsl_vector_view_array(coeffs.data(), degree + 1);

        size_t istart, iend;
        gsl_bspline_eval_nonzero(x, &basis_view.vector, &istart, &iend, interface_.workspace.get());

        return Regressor(coeffs, istart);
    }

    auto SplineWrapper8Dim::supportLowerLimit() const -> double
    {
        if (!isInitialized())
        {
            throw std::runtime_error("Spline not initialized.");
        }

        return gsl_bspline_breakpoint(0, interface_.workspace.get());
    }

    auto SplineWrapper8Dim::supportUpperLimit() const -> double
    {
        if (!isInitialized())
        {
            throw std::runtime_error("Spline not initialized.");
        }

        return gsl_bspline_breakpoint(gsl_bspline_nbreak(interface_.workspace.get()) - 1, interface_.workspace.get());
    }

    inline auto SplineWrapper8Dim::isInitialized() const -> bool
    {
        return interface_.workspace ? true : false;
    }

    inline uint32_t SplineWrapper8Dim::computeDegree(uint32_t nbreakpts, uint32_t ncontrols)
    {
        return ncontrols - nbreakpts + 1;
    }

    inline uint32_t SplineWrapper8Dim::computeNumBreakPoints(uint32_t ncontrols, uint32_t degree)
    {
        return ncontrols - degree + 1;
    }

    void SplineWrapper8Dim::allocateInterface(Interface &interface, uint32_t degree, uint32_t nbreakpts, uint32_t ncontrols)
    {
        interface.workspace = std::unique_ptr<gsl_bspline_workspace, void (*)(gsl_bspline_workspace *)>(
            gsl_bspline_alloc(degree + 1, nbreakpts), gsl_bspline_free);
        interface.B = std::unique_ptr<gsl_vector, void (*)(gsl_vector *)>(gsl_vector_alloc(degree + 1), gsl_vector_free);
        interface.dB =
            std::unique_ptr<gsl_matrix, void (*)(gsl_matrix *)>(gsl_matrix_alloc(degree + 1, 3), gsl_matrix_free);
        interface.controls =
            std::unique_ptr<gsl_matrix, void (*)(gsl_matrix *)>(gsl_matrix_alloc(ny, ncontrols), gsl_matrix_free);
    }

} // namespace sipoc_mr_utils
