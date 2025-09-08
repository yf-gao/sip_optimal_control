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
// Author: Niels van Duijkeren
//

#ifndef SIPOC_MR_UTILS__SPLINE_FITTER_HPP_
#define SIPOC_MR_UTILS__SPLINE_FITTER_HPP_

#include <sipoc_mr_utils/spline_wrapper.hpp>

#include <Eigen/Core>
#include <Eigen/QR>

#include <vector>

namespace sipoc_mr_utils
{

    class SplineFitter
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        using DataGrid = std::vector<double>;
        using DataPoint = SplineWrapper::Output;
        using DataContainer = std::vector<DataPoint, Eigen::aligned_allocator<DataPoint>>;

        SplineFitter() = default;
        SplineFitter(SplineWrapper spline, int32_t num_data);
        SplineFitter(SplineWrapper spline, const DataGrid &data_grid);

        void init(uint32_t N, uint32_t degree, uint32_t nknots);

        void setBSpline(SplineWrapper bspline, int32_t num_data);
        void setBSpline(SplineWrapper bspline, const DataGrid &data_grid);
        void setDataGrid(const DataGrid &data_grid);
        auto fit(const DataContainer &data) const -> typename SplineWrapper::Controls;

        auto bspline() const -> const SplineWrapper &;
        auto knots() const -> typename SplineWrapper::Knots;

    private:
        std::optional<SplineWrapper> spline_{};

        void initialize(const DataGrid &data_grid);
        void initialize(int32_t num_data);

        auto parseInput(const DataContainer &data) const -> Eigen::Matrix<double, Eigen::Dynamic, SplineWrapper::ny>;
        auto parseOutput(const Eigen::Matrix<double, Eigen::Dynamic, SplineWrapper::ny> &c) const ->
            typename SplineWrapper::Controls;

        /// The Eigen Full-Pivot Housholder QR is relatively slow, but stable.
        /// Speed is of no concern because the matrix decomposition needs to be performed only once.
        using Regressor_t = Eigen::MatrixXd;
        Eigen::FullPivHouseholderQR<Regressor_t> solver_;
    };

} // namespace sipoc_mr_utils

#endif // SIPOC_MR_UTILS__SPLINE_FITTER_HPP_
