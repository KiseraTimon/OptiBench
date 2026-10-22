#pragma once

/*
 * Special Notes: B-Spline Curves
 *
 * A degree-p B-spline with control points P0..Pn and a non-decreasing knot vector
 * u0 <= u1 <= ... <= um (m = n + p + 1) is
 *
 *      C(u) = sum_i N(i,p)(u) Pi,      u in [u(p), u(n+1)]
 *
 * with the Cox-de Boor basis
 *
 *      N(i,0)(u) = 1 if u(i) <= u < u(i+1) else 0
 *      N(i,p)(u) = (u - u(i)) / (u(i+p) - u(i)) N(i,p-1)(u) + (u(i+p+1) - u) / (u(i+p+1) - u(i+1)) N(i+1,p-1)(u)
 *
 * Properties: local control (Pi only affects [u(i), u(i+p+1))), partition of unity, and C(p-k)
 * continuity at a knot of multiplicity k.
 *
 *   evaluate()      de Boor's algorithm: p rounds of affine combinations, O(p^2), stable
 *   insertKnot()    Boehm's algorithm: adds a knot without changing the curve
 *   toBezier()      inserts every interior knot up to multiplicity p, which splits the spline
 *                   into its Bezier pieces
 *
 * Knot vector kinds provided by the factories:
 *   clamped (open uniform): p + 1 equal knots at each end, so the curve starts at P0 and ends
 *                           at Pn, tangent to the control polygon there; domain [0, 1]
 *   uniform:                knots 0, 1, 2, ...; the curve does not reach P0 / Pn; domain [p, n + 1]
 */

#include <span>
#include <vector>

#include "optibench/curves/Composite.h"

namespace ob::curves {

    class BSpline {
    public:
        /**
         * @param degree int: p >= 0
         * @param controlPoints at least p + 1 points
         * @param knots non-decreasing, finite, exactly controlPoints.size() + p + 1 values, with
         *              a non-empty domain u(p) < u(n+1) and no interior multiplicity above p + 1
         * @throws std::invalid_argument when any requirement is violated
         */
        BSpline(int degree, std::vector<Vec2> controlPoints, std::vector<double> knots);

        // clamped (open uniform) knot vector on [0, 1]; interpolates the first and last points
        static BSpline clamped(int degree, std::vector<Vec2> controlPoints);

        // uniform knot vector 0, 1, ..., n + p + 1
        static BSpline uniform(int degree, std::vector<Vec2> controlPoints);

        int degree() const { return degree_; }
        std::span<const Vec2> controlPoints() const { return points_; }
        std::span<const double> knots() const { return knots_; }

        // moves one control point (local control: only nearby spans change)
        void setControlPoint(std::size_t i, Vec2 p) { points_.at(i) = p; }

        // valid parameter range [u(p), u(n+1)]
        double paramMin() const;
        double paramMax() const;

        // true if both ends have p + 1 equal knots
        bool isClamped() const;

        // multiplicity of the knot value u (0 if it is not a knot)
        int multiplicity(double u) const;

        // point at u (clamped to the domain), de Boor's algorithm
        Vec2 evaluate(double u) const;

        // the non-zero basis values N(k-p..k, p)(u) and the span index k
        std::vector<double> basis(double u, std::size_t& span) const;

        // derivative B-spline (degree p - 1); for p = 0 a zero curve
        BSpline derivativeSpline() const;

        // order-th derivative at u
        Vec2 derivative(double u, int order = 1) const;

        // unit tangent at u (zero vector where the derivative vanishes)
        Vec2 tangent(double u) const { return derivative(u).normalized(); }

        /**
         * inserts the knot u up to `times` times (Boehm), never exceeding multiplicity p
         * @return the number of insertions actually made (0 if u is outside the domain)
         */
        int insertKnot(double u, int times = 1);

        // the same curve as Bezier segments of degree p, one per non-empty knot span
        CompositeBezier toBezier() const;

    private:
        int degree_;
        std::vector<Vec2> points_;
        std::vector<double> knots_;

        // index k with u(k) <= u < u(k+1) inside the domain (last non-empty span at the end)
        std::size_t findSpan(double u) const;
    };
}
