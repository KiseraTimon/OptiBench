#pragma once

/*
 * Special Notes: Natural Cubic Spline
 *
 * Interpolates points P0..Pn at knots t0 < t1 < ... < tn with one cubic per interval so that
 * the whole curve is C2 (position, first and second derivative continuous). With hi = t(i+1) - ti
 * and Mi = S''(ti) the unknown second derivatives, C2 continuity gives for i = 1..n-1
 *
 *      h(i-1) M(i-1) + 2 (h(i-1) + hi) Mi + hi M(i+1) = 6 [ (P(i+1) - Pi)/hi - (Pi - P(i-1))/h(i-1) ]
 *
 * and the "natural" end conditions M0 = Mn = 0 (zero curvature at the ends). The system is
 * tridiagonal and strictly diagonally dominant, so the Thomas algorithm solves it in O(n)
 * without pivoting. Each interval is then
 *
 *      S(t) = A Pi + B P(i+1) + ((A^3 - A) Mi + (B^3 - B) M(i+1)) hi^2 / 6,
 *      A = (t(i+1) - t) / hi,  B = 1 - A.
 *
 * The spline is parametric (x and y are splined separately against t), so it works for any
 * point sequence, not only graphs y = f(x).
 */

#include <span>
#include <vector>

#include "optibench/curves/Composite.h"
#include "optibench/curves/CurveMath.h"

namespace ob::curves {

    class NaturalCubicSpline {
    public:
        /**
         * @param points the points to interpolate; consecutive duplicates are removed
         * @param parametrization how knots are spaced (chordal follows the point spacing)
         * @throws std::invalid_argument if fewer than 2 distinct points remain
         */
        explicit NaturalCubicSpline(std::vector<Vec2> points,
                                    Parametrization parametrization = Parametrization::Chordal);

        // interpolated points (after duplicate removal) and their knots
        std::span<const Vec2> points() const { return points_; }
        std::span<const double> knots() const { return knots_; }

        // second derivatives S''(ti) at the knots
        std::span<const Vec2> secondDerivatives() const { return m_; }

        // parameter range [t0, tn]
        double paramMin() const { return knots_.front(); }
        double paramMax() const { return knots_.back(); }

        // point at knot parameter t (clamped to the range)
        Vec2 evaluate(double t) const;

        // derivative of order 1..3 at t (0 for higher orders)
        Vec2 derivative(double t, int order = 1) const;

        // the exact same curve as cubic Bezier segments (one per interval)
        CompositeBezier toBezier() const;

    private:
        std::vector<Vec2> points_;
        std::vector<double> knots_;
        std::vector<Vec2> m_;

        // index of the interval containing t (clamped)
        std::size_t interval(double t) const;
    };
}
