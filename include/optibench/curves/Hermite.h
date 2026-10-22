#pragma once

/*
 * Special Notes: Cubic Hermite Curves
 *
 * A cubic Hermite segment is fixed by its end points p0, p1 and end tangents m0, m1:
 *
 *      H(t) = h00(t) p0 + h10(t) m0 + h01(t) p1 + h11(t) m1,   t in [0, 1]
 *
 *      h00 = 2t^3 - 3t^2 + 1     h10 = t^3 - 2t^2 + t
 *      h01 = -2t^3 + 3t^2        h11 = t^3 - t^2
 *
 * It is the same curve as the cubic Bezier (p0, p0 + m0/3, p1 - m1/3, p1). A Hermite spline
 * shares each point and tangent between neighbouring segments, so it is C1 by construction.
 */

#include <array>
#include <span>
#include <vector>

#include "optibench/curves/Composite.h"

namespace ob::curves {

    // the four Hermite basis values {h00, h10, h01, h11} at t
    std::array<double, 4> hermiteBasis(double t);

    // one cubic Hermite segment
    struct HermiteSegment {
        Vec2 p0;    // start point
        Vec2 m0;    // tangent (derivative) at the start
        Vec2 p1;    // end point
        Vec2 m1;    // tangent (derivative) at the end

        // point at t in [0, 1]
        Vec2 evaluate(double t) const;

        // first derivative at t
        Vec2 derivative(double t) const;

        // the identical cubic Bezier
        Bezier toBezier() const { return Bezier({p0, p0 + m0 / 3.0, p1 - m1 / 3.0, p1}); }
    };

    // a C1 chain of Hermite segments through points with prescribed tangents
    class HermiteSpline {
    public:
        /**
         * @param points the points to pass through (at least 2)
         * @param tangents the derivative at each point (same count as points)
         * @throws std::invalid_argument on a size mismatch or fewer than 2 points
         */
        HermiteSpline(std::vector<Vec2> points, std::vector<Vec2> tangents);

        std::span<const Vec2> points() const { return points_; }
        std::span<const Vec2> tangents() const { return tangents_; }
        std::size_t segmentCount() const { return points_.size() - 1; }

        // segment i (0..segmentCount()-1)
        HermiteSegment segment(std::size_t i) const;

        // point at global parameter u in [0, segmentCount()]
        Vec2 evaluate(double u) const;

        // the same curve as cubic Bezier segments
        CompositeBezier toBezier() const;

    private:
        std::vector<Vec2> points_;
        std::vector<Vec2> tangents_;
    };
}
