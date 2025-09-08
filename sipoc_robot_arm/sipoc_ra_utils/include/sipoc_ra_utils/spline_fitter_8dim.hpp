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

#ifndef SIPOC_RA_UTILS__SPLINE_FITTER_8DIM_HPP_
#define SIPOC_RA_UTILS__SPLINE_FITTER_8DIM_HPP_

#include <sipoc_ra_utils/spline_wrapper_8dim.hpp>

#include <Eigen/Core>
#include <Eigen/QR>

#include <vector>

namespace sipoc_ra_utils
{
    class SplineFitter
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        using DataGrid = std::vector<double>;
        using DataPoint = SplineWrapper8Dim::Output;
        using DataContainer = std::vector<DataPoint, Eigen::aligned_allocator<DataPoint>>;

        SplineFitter() = default;

        SplineFitter(SplineWrapper8Dim spline, int32_t num_data);

        SplineFitter(SplineWrapper8Dim spline, const DataGrid &data_grid);

        void init(uint32_t N, uint32_t degree, uint32_t nknots);

        void setBSpline(SplineWrapper8Dim bspline, int32_t num_data);
        void setBSpline(SplineWrapper8Dim bspline, const DataGrid &data_grid);
        void setDataGrid(const DataGrid &data_grid);
        auto fit(const DataContainer &data) const -> typename SplineWrapper8Dim::Controls;

        auto bspline() const -> const SplineWrapper8Dim &;
        auto knots() const -> typename SplineWrapper8Dim::Knots;

    private:
        std::optional<SplineWrapper8Dim> spline_{};

        void initialize(const DataGrid &data_grid);
        void initialize(int32_t num_data);

        auto parseInput(const DataContainer &data) const -> Eigen::Matrix<double, Eigen::Dynamic, SplineWrapper8Dim::ny>;
        auto parseOutput(const Eigen::Matrix<double, Eigen::Dynamic, SplineWrapper8Dim::ny> &c) const ->
            typename SplineWrapper8Dim::Controls;

        /// The Eigen Full-Pivot Housholder QR is relatively slow, but stable.
        /// Speed is of no concern because the matrix decomposition needs to be performed only once.
        using Regressor_t = Eigen::MatrixXd;
        Eigen::FullPivHouseholderQR<Regressor_t> solver_;
    };

} // namespace sipoc_ra_utils

#endif // SIPOC_RA_UTILS__SPLINE_FITTER_8DIM_HPP_
