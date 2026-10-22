#pragma once

/*
 * Special Notes: Rational Curves (Rational Bezier and NURBS)
 *
 * Giving each control point a weight wi > 0 makes the curve rational:
 *
 *      C(t) = sum Bi(t) wi Pi / sum Bi(t) wi
 *
 * Equivalently, C is the projection of a polynomial curve in homogeneous coordinates
 * (w x, w y, w) back onto w = 1. All the polynomial algorithms (de Casteljau, de Boor, knot
 * insertion, subdivision) therefore apply unchanged to the homogeneous points, followed by one
 * division. Rational quadratics represent conics exactly: with weights (1, cos(a/2), 1) a quadratic
 * traces a circular arc of angle a, which polynomial curves can only approximate.
 *
 * With positive weights the convex hull property still holds, so flattening and ray
 * intersection work as for polynomial curves. For ray intersection the line test becomes
 *
 *      f(t) = sum Bi(t) wi cross(d, Pi - o)   (the denominator is positive and drops out)
 *
 * Nurbs stores its homogeneous form as two B-splines, A (control points wi Pi) and W
 * (control points (wi, 0)), so evaluation, derivatives (C' = (A' - w' C) / w), knot insertion and
 * Bezier extraction reuse BSpline.
 */

#include <span>
#include <utility>
#include <vector>

#include "optibench/curves/BSpline.h"
#include "optibench/curves/OpticalCurve.h"

namespace ob::curves {

    class RationalBezier {
    public:
        /**
         * @param points control points P0..Pn (at least one)
         * @param weights one positive, finite weight per point
         * @throws std::invalid_argument on size mismatch or non-positive weights
         */
        RationalBezier(std::vector<Vec2> points, std::vector<double> weights);

        // a polynomial Bezier as a rational one (all weights 1)
        static RationalBezier fromBezier(const Bezier& b);

        int degree() const { return static_cast<int>(points_.size()) - 1; }
        std::span<const Vec2> controlPoints() const { return points_; }
        std::span<const double> weights() const { return weights_; }
        Vec2 front() const { return points_.front(); }
        Vec2 back() const { return points_.back(); }

        // point at t (homogeneous de Casteljau)
        Vec2 evaluate(double t) const;

        // first derivative at t
        Vec2 derivative(double t) const;

        // unit tangent / normal (tangent.perp()); zero where the derivative vanishes
        Vec2 tangent(double t) const { return derivative(t).normalized(); }
        Vec2 normal(double t) const { return tangent(t).perp(); }

        // signed curvature, by central differences of the exact first derivative
        double curvature(double t) const;

        // splits at t into two rational curves
        std::pair<RationalBezier, RationalBezier> split(double t) const;

        // adaptive flattening within tolerance (same flatness test as Bezier::flatten)
        Polyline flatten(double tolerance) const;

        // box of the control points (contains the curve)
        Aabb controlBounds() const { return boundsOf(points_); }

    private:
        std::vector<Vec2> points_;
        std::vector<double> weights_;
    };

    // all ray hits with a rational Bezier curve, sorted by distance (see OpticalCurve.h)
    std::vector<CurveHit> intersectRay(const Ray& ray, const RationalBezier& curve, const RayCurveOptions& options = {},
                                       double normalSign = 1.0);

    class Nurbs {
    public:
        /**
         * @param degree int: p >= 1
         * @param points control points (at least p + 1)
         * @param weights one positive weight per point
         * @param knots as for BSpline: points + p + 1 non-decreasing values
         * @throws std::invalid_argument on invalid input
         */
        Nurbs(int degree, std::vector<Vec2> points, std::vector<double> weights, std::vector<double> knots);

        // exact circular arc from startAngle to endAngle (radians, counter-clockwise in y-up terms)
        static Nurbs circularArc(Vec2 center, double radius, double startAngle, double endAngle);

        // exact full circle (four rational quadratic arcs)
        static Nurbs circle(Vec2 center, double radius);

        // exact axis-aligned ellipse
        static Nurbs ellipse(Vec2 center, double rx, double ry);

        int degree() const { return homogeneous_.degree(); }
        std::span<const double> knots() const { return homogeneous_.knots(); }
        double paramMin() const { return homogeneous_.paramMin(); }
        double paramMax() const { return homogeneous_.paramMax(); }

        // control points and weights (recovered from the homogeneous form)
        std::vector<Vec2> controlPoints() const;
        std::vector<double> weights() const;

        // point at u
        Vec2 evaluate(double u) const;

        // first derivative at u
        Vec2 derivative(double u) const;

        // unit tangent at u
        Vec2 tangent(double u) const { return derivative(u).normalized(); }

        // inserts a knot without changing the curve; returns insertions made
        int insertKnot(double u, int times = 1);

        // one rational Bezier per non-empty knot span
        std::vector<RationalBezier> toRationalBezier() const;

        // adaptive flattening within tolerance
        Polyline flatten(double tolerance) const;

        // all ray hits, sorted by distance (shared span end points reported once)
        std::vector<CurveHit> intersect(const Ray& ray, const RayCurveOptions& options = {}, double normalSign = 1.0) const;

    private:
        explicit Nurbs(std::pair<BSpline, BSpline> parts);

        // validates the input and builds the homogeneous pair (A, W)
        static std::pair<BSpline, BSpline> makeHomogeneous(int degree, std::vector<Vec2> points,
                                                           std::vector<double> weights, std::vector<double> knots);

        BSpline homogeneous_; // control points w * P
        BSpline weightCurve_; // control points (w, 0)
    };
}
