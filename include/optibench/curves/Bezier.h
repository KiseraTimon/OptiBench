#pragma once

/*
 * Special Notes: Bezier Curves
 *
 * A Bezier curve of degree n has n + 1 control points P0..Pn and is
 *
 *      B(t) = sum_{i=0..n} B(i,n)(t) Pi,      t in [0, 1],
 *
 * where B(i,n)(t) = C(n,i) t^i (1-t)^(n-i) are the Bernstein polynomials. Useful facts that
 * the implementation (and tests) rely on:
 *
 *   - endpoint interpolation:  B(0) = P0, B(1) = Pn
 *   - partition of unity:      sum_i B(i,n)(t) = 1, all B(i,n)(t) >= 0 on [0, 1]
 *   - convex hull property:    the curve lies inside the convex hull of its control points
 *   - derivative (hodograph):  B'(t) = n sum_{i=0..n-1} B(i,n-1)(t) (P(i+1) - Pi)
 *   - de Casteljau:            repeated linear interpolation; evaluates and splits the curve
 *                              with only convex combinations, so it is numerically stable
 *
 * Coordinates are plain doubles; nothing here assumes y-up or y-down. normal() is the tangent
 * rotated by +90 degrees with Vec2::perp(), i.e. counter-clockwise in a y-up frame (it appears
 * clockwise on the y-down framebuffer). curvature() uses the same sign convention.
 */

#include <span>
#include <utility>
#include <vector>

#include "optibench/Vec2.h"
#include "optibench/curves/CurveMath.h"

namespace ob::curves {

    class Bezier {
    public:
        /**
         * @param controlPoints P0..Pn; at least one point (a single point is a degree-0 curve)
         * @throws std::invalid_argument if controlPoints is empty
         */
        explicit Bezier(std::vector<Vec2> controlPoints);

        // straight line from a to b (degree 1)
        static Bezier line(Vec2 a, Vec2 b) { return Bezier({a, b}); }

        // degree n = number of control points - 1
        int degree() const { return static_cast<int>(points_.size()) - 1; }

        // read-only view of the control points
        std::span<const Vec2> controlPoints() const { return points_; }

        // a single control point (index must be 0..degree())
        Vec2 controlPoint(std::size_t i) const { return points_.at(i); }

        // replaces a control point (index must be 0..degree())
        void setControlPoint(std::size_t i, Vec2 p) { points_.at(i) = p; }

        // first and last control points (the curve's endpoints)
        Vec2 front() const { return points_.front(); }
        Vec2 back() const { return points_.back(); }

        /*
         * Evaluation
         */
        // point at t; closed-form fast paths for degree <= 3, de Casteljau otherwise
        Vec2 evaluate(double t) const;

        // point at t using de Casteljau's algorithm for any degree, O(n^2)
        Vec2 evaluateDeCasteljau(double t) const;

        // point at t summing the Bernstein form explicitly, O(n^2); kept for comparison/teaching
        Vec2 evaluateBernstein(double t) const;

        /*
         * Differential geometry
         */
        // the derivative curve B'(t) of degree n - 1 (a single zero point for degree 0)
        Bezier hodograph() const;

        // the order-th derivative at t (order >= 1); zero when order > degree
        Vec2 derivative(double t, int order = 1) const;

        /**
         * unit tangent at t
         *
         * Where B'(t) vanishes (e.g. coincident control points P0 = P1), the direction is taken
         * from the first non-zero higher derivative, which is the one-sided limit of the tangent.
         * A fully degenerate curve (all points equal) returns the zero vector.
         */
        Vec2 tangent(double t) const;

        // unit normal: tangent(t).perp(); zero vector if the tangent is undefined
        Vec2 normal(double t) const;

        /**
         * signed curvature k = cross(B', B'') / |B'|^3
         * Positive when the curve turns towards normal(). Returns 0 where |B'| = 0, where
         * curvature is undefined (cusps, degenerate curves).
         */
        double curvature(double t) const;

        /*
         * Subdivision and degree elevation
         */
        // splits at t into [0, t] and [t, 1], each reparameterized to [0, 1]
        std::pair<Bezier, Bezier> split(double t) const;

        // the piece between t0 and t1 (t0 <= t1), reparameterized to [0, 1]
        Bezier subCurve(double t0, double t1) const;

        // the same curve with degree n + 1 (one more control point)
        Bezier elevated() const;

        /*
         * Flattening
         */
        /**
         * adaptive flattening: recursively halves the curve until every interior control point
         * lies within tolerance of the chord P0-Pn. By the convex hull property every curve point
         * is then within tolerance of the polyline. Flat parts produce few points, tight bends many.
         * @param tolerance double: maximum distance from curve to polyline (clamped to a tiny minimum)
         */
        Polyline flatten(double tolerance) const;

        // fixed-step sampling at segments + 1 evenly spaced parameters (segments clamped to >= 1)
        Polyline flattenUniform(int segments) const;

        /*
         * Bounds and length
         */
        // box of the control polygon (loose but cheap; contains the curve)
        Aabb controlBounds() const { return boundsOf(points_); }

        // tight box: endpoints plus points where x'(t) = 0 or y'(t) = 0
        Aabb bounds() const;

        // arc length between t0 and t1 by adaptive Gauss-Legendre quadrature of |B'(t)|
        double arcLength(double t0 = 0.0, double t1 = 1.0) const;

        // cumulative arc length table, for repeated length/parameter queries
        ArcLengthTable arcLengthTable(int intervals = 32) const;

        // parameter t whose arc length from 0 is s (s clamped to [0, arcLength()])
        double paramAtArcLength(double s) const;

        // segments + 1 points evenly spaced by arc length (arc-length reparameterization)
        Polyline sampleByArcLength(int segments) const;

        // nearest point on the curve to p: returns the parameter t
        double closestParam(Vec2 p) const;

    private:
        std::vector<Vec2> points_;

        // size of the control polygon, used to scale "near zero" thresholds
        double scale() const;
    };
}
