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

#include "sipoc_ra_utils/spline_fitter_8dim.hpp"

#include <stdexcept>

namespace sipoc_ra_utils
{

    SplineFitter::SplineFitter(SplineWrapper8Dim spline, int32_t num_data) : spline_(std::move(spline))
    {
        initialize(num_data);
    }

    SplineFitter::SplineFitter(SplineWrapper8Dim spline, const DataGrid &data_grid) : spline_(std::move(spline))
    {
        initialize(data_grid);
    }

    void SplineFitter::init(uint32_t N, uint32_t degree, uint32_t nknots)
    {
        // Knot points vary from 0 .. 1 equidistantly
        typename SplineWrapper8Dim::KnotVector knots(nknots, 0.0);
        for (uint32_t i = 0; i < knots.size(); ++i)
        {
            knots[i] = i / static_cast<double>(knots.size() - 1);
        }

        SplineWrapper8Dim spline(degree, SplineWrapper8Dim::ControlVector(nknots + degree - 1, SplineWrapper8Dim::Control::Zero()));
        spline.setKnotPoints(knots);
        this->setBSpline(std::move(spline), N);
    }

    void SplineFitter::setBSpline(SplineWrapper8Dim bspline, int32_t num_data)
    {
        spline_ = std::move(bspline);
        initialize(num_data);
    }

    void SplineFitter::setBSpline(SplineWrapper8Dim bspline, const DataGrid &data_grid)
    {
        spline_ = std::move(bspline);
        initialize(data_grid);
    }

    void SplineFitter::setDataGrid(const DataGrid &data_grid)
    {
        initialize(data_grid);
    }

    auto SplineFitter::fit(const DataContainer &data) const -> typename SplineWrapper8Dim::Controls
    {
        if (!spline_)
        {
            throw std::runtime_error("Spline not initialized.");
        }

        if (data.size() != solver_.rows())
        {
            throw std::runtime_error("Data size does not match initialized data grid.");
        }

        auto rhs = parseInput(data);

        auto c = solver_.solve(rhs);

        return parseOutput(c);
    }

    auto SplineFitter::bspline() const -> const SplineWrapper8Dim &
    {
        if (!spline_)
        {
            throw std::runtime_error("Spline not initialized.");
        }

        return *spline_;
    }

    auto SplineFitter::knots() const -> typename SplineWrapper8Dim::Knots
    {
        return this->bspline().getKnotPoints();
    }

    void SplineFitter::initialize(const DataGrid &data_grid)
    {
        // Solve least-squares problem
        //
        //  minimize ||A*c - r||^2
        //
        // with optimization variables c, and parameters r.

        if (!spline_)
        {
            throw std::runtime_error("Spline not initialized.");
        }

        /// Create regressor matrix
        Regressor_t regressor = Regressor_t::Zero(data_grid.size(), spline_->getNumControlPoints());

        /// Fill regressor matrix
        for (uint32_t i = 0; i < data_grid.size(); ++i)
        {
            typename SplineWrapper8Dim::Regressor regr = spline_->regressor(data_grid[i]);
            for (uint32_t ii = 0; ii < regr.num_coeffs(); ++ii)
            {
                regressor(i, regr.offset() + ii) = regr.coeffs()[ii];
            }
        }

        /// Factorize regressor
        solver_.compute(regressor);
    }

    void SplineFitter::initialize(int32_t num_data)
    {
        if (!spline_)
        {
            throw std::runtime_error("Spline not initialized.");
        }

        if (num_data < 2)
        {
            throw std::runtime_error("Number of data points must be at least 2.");
        }

        const double ll = spline_->supportLowerLimit();
        const double ul = spline_->supportUpperLimit();
        if (ll > ul)
        {
            throw std::runtime_error("Lower limit must be less than upper limit.");
        }
        const double step = (ul - ll) / (num_data - 1);

        // Data points across equidistantly
        std::vector<double> data_grid(num_data);
        for (uint32_t i = 0; i < data_grid.size(); ++i)
        {
            data_grid[i] = ll + i * step;
        }

        initialize(data_grid);
    }

    auto SplineFitter::parseInput(const DataContainer &data) const
        -> Eigen::Matrix<double, Eigen::Dynamic, SplineWrapper8Dim::ny>
    {
        // Number of data points has been asserted already and must be greater than 1
        Eigen::Matrix<double, Eigen::Dynamic, SplineWrapper8Dim::ny> input(data.size(), data.front().rows());

        for (uint32_t i = 0; i < data.size(); ++i)
        {
            input.template block<1, SplineWrapper8Dim::ny>(i, 0) = data[i].transpose();
        }

        return input;
    }

    auto SplineFitter::parseOutput(const Eigen::Matrix<double, Eigen::Dynamic, SplineWrapper8Dim::ny> &c) const ->
        typename SplineWrapper8Dim::Controls
    {
        typename SplineWrapper8Dim::Controls output;
        const int32_t ncontrols = c.rows();

        output.resize(ncontrols);

        for (uint32_t i = 0; i < ncontrols; ++i)
        {
            output[i] = c.template block<1, SplineWrapper8Dim::ny>(i, 0).transpose();
        }

        return output;
    }

} // namespace sipoc_mr_utils
