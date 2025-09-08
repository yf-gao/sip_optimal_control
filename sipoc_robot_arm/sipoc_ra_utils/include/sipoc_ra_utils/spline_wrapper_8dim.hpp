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

#ifndef SIPOC_RA_UTILS__SPLINE_WRAPPER_8DIM_HPP_
#define SIPOC_RA_UTILS__SPLINE_WRAPPER_8DIM_HPP_

#include <gsl/gsl_bspline.h>

#include <Eigen/Core>
#include <memory>
#include <optional>


namespace sipoc_ra_utils
{
    class SplineWrapper8Dim
    {
    public:
        using Output = Eigen::Matrix<double, 8, 1>;
        using OutputTangent = Eigen::Matrix<double, 8, 1>;
        using Control = Eigen::Matrix<double, 8, 1>;

        using KnotVector = std::vector<double>;
        using ControlVector = std::vector<Control, Eigen::aligned_allocator<Control>>;
        using Knots = KnotVector;
        using Controls = ControlVector;

        using RegressorCoeffs = std::vector<double>;
        class Regressor
        {
        public:
            Regressor(RegressorCoeffs coeffs, uint32_t offset) : coeffs_(std::move(coeffs)), offset_(offset)
            {
            }
            auto coeffs() const -> const RegressorCoeffs &
            {
                return coeffs_;
            }
            double coeffs(const uint32_t i) const
            {
                return coeffs().at(i);
            }
            uint32_t offset() const
            {
                return offset_;
            }
            uint32_t num_coeffs() const
            {
                return coeffs().size();
            }
            double at(const uint32_t i) const
            {
                return coeffs().at(i);
            }
            double operator[](const uint32_t i) const
            {
                return coeffs()[i];
            }

        private:
            RegressorCoeffs coeffs_;
            uint32_t offset_{};
        };

        constexpr static int32_t ny = 8;

        SplineWrapper8Dim() = default;
        template <typename... Args, typename = std::enable_if_t<(sizeof...(Args) > 1)>>
        SplineWrapper8Dim(Args &&...args)
        {
            configure(std::forward<Args>(args)...);
        }

        virtual ~SplineWrapper8Dim() = default;

        SplineWrapper8Dim(const SplineWrapper8Dim &) = delete;
        SplineWrapper8Dim &operator=(const SplineWrapper8Dim &) = delete;
        SplineWrapper8Dim(SplineWrapper8Dim &&) = default;
        SplineWrapper8Dim &operator=(SplineWrapper8Dim &&) = default;

        void configure(uint32_t degree, const ControlVector &controls);
        void configure(const KnotVector &knots, const ControlVector &controls);

        auto getDegree() const -> uint32_t;
        auto getNumControlPoints() const -> uint32_t;
        auto getNumKnotPoints() const -> uint32_t;

        void setKnotPoints(const KnotVector &knots);
        void setControlPoints(const ControlVector &controls);

        auto getKnotPoints() const -> KnotVector;
        auto getControlPoints() const -> ControlVector;

        auto eval(double x) const -> Output;
        auto p(double x) const -> OutputTangent;
        auto p(double x, Output &y) const -> OutputTangent;
        auto pp(double x) const -> OutputTangent;
        auto pp(double x, Output &y, OutputTangent &p) const -> OutputTangent;
        auto curvature(double x) const -> double;
        auto regressor(double x) const -> Regressor;
        auto supportLowerLimit() const -> double;
        auto supportUpperLimit() const -> double;
        auto isInitialized() const -> bool;

        auto operator()(double x) const -> Output
        {
            return eval(x);
        }

    private:
        struct Interface
        {
            Interface()
                : workspace(nullptr, gsl_bspline_free), B(nullptr, gsl_vector_free), dB(nullptr, gsl_matrix_free),
                  controls(nullptr, gsl_matrix_free)
            {
            }
            virtual ~Interface() = default;
            Interface(const Interface &) = delete;
            Interface &operator=(const Interface &) = delete;
            Interface(Interface &&) = default;
            Interface &operator=(Interface &&) = default;

            std::unique_ptr<gsl_bspline_workspace, void (*)(gsl_bspline_workspace *)> workspace;
            std::unique_ptr<gsl_vector, void (*)(gsl_vector *)> B;
            std::unique_ptr<gsl_matrix, void (*)(gsl_matrix *)> dB;
            std::unique_ptr<gsl_matrix, void (*)(gsl_matrix *)> controls;
        };
        Interface interface_;

        static uint32_t computeDegree(uint32_t nbreakpts, uint32_t ncontrols);
        static uint32_t computeNumBreakPoints(uint32_t ncontrols, uint32_t degree);
        static void allocateInterface(Interface &interface, uint32_t degree, uint32_t nbreakpts, uint32_t ncontrols);
    };

} // namespace sipoc_ra_utils

#endif // SIPOC_RA_UTILS__SPLINE_WRAPPER_8DIM_HPP_
