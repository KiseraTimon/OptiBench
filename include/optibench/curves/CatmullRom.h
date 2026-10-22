#pragma once

/*
 * Special Notes: Catmull-Rom Splines
 *
 * A Catmull-Rom spline interpolates its points; the tangent at each point is computed from
 * its neighbours. For the segment P1 -> P2 with knots t0..t3 (spaced by |P(i+1) - Pi|^alpha):
 *
 *      v1 = (P1 - P0)/(t1 - t0) - (P2 - P0)/(t2 - t0) + (P2 - P1)/(t2 - t1)
 *      v2 = (P2 - P1)/(t2 - t1) - (P3 - P1)/(t3 - t1) + (P3 - P2)/(t3 - t2)
 *
 * (the derivatives of the Barry-Goldman pyramid). Scaled to the segment's own [0, 1] parameter
 * and by (1 - tension), they become Hermite tangents:
 *
 *      m1 = (1 - tension) (t2 - t1) v1,      m2 = (1 - tension) (t2 - t1) v2
 *
 * and the segment is the cubic Bezier (P1, P1 + m1/3, P2 - m2/3, P2).
 *
 *   alpha = 0    uniform: classic Catmull-Rom, m1 = (P2 - P0)/2; C1 in the global parameter
 *   alpha = 0.5  centripetal: no cusps or self-intersections within a segment
 *   alpha = 1    chordal
 *
 *   tension = 0  standard Catmull-Rom; tension = 1 zero tangents (straight segments);
 *   negative values loosen the curve.
 *
 * Non-uniform variants are C1 in the knot parameter; across segments of the composite
 * (each reparameterized to [0, 1]) they are G1.
 *
 * Open curves pass through every point: missing outer neighbours are reflected phantoms
 * (P(-1) = 2 P0 - P1). Closed curves wrap around and add the closing segment.
 */

#include <span>
#include <vector>

#include "optibench/curves/Composite.h"
#include "optibench/curves/CurveMath.h"

namespace ob::curves {

    class CatmullRom {
    public:
        /**
         * @param points the points to interpolate; consecutive duplicates (and, for closed
         *               curves, a last point equal to the first) are removed
         * @param alpha double: knot exponent, usually 0, 0.5 or 1 (must be finite, >= 0)
         * @param tension double: 0 for the standard spline (must be finite)
         * @param closed bool: wrap around into a closed loop
         * @throws std::invalid_argument for fewer than 2 distinct points (3 when closed) or bad parameters
         */
        explicit CatmullRom(std::vector<Vec2> points, double alpha = 0.5, double tension = 0.0, bool closed = false);

        // same, choosing alpha by name
        CatmullRom(std::vector<Vec2> points, Parametrization parametrization, double tension = 0.0,
                   bool closed = false)
            : CatmullRom(std::move(points), alphaOf(parametrization), tension, closed) {}

        std::span<const Vec2> points() const { return points_; }
        double alpha() const { return alpha_; }
        double tension() const { return tension_; }
        bool closed() const { return closed_; }

        // number of segments (points - 1 for open curves, points for closed ones)
        std::size_t segmentCount() const { return bezier_.segmentCount(); }

        // point / tangent at global parameter u in [0, segmentCount()]
        Vec2 evaluate(double u) const { return bezier_.evaluate(u); }
        Vec2 derivative(double u) const { return bezier_.derivative(u); }

        // the curve as cubic Bezier segments (computed once at construction)
        const CompositeBezier& toBezier() const { return bezier_; }

    private:
        std::vector<Vec2> points_;
        double alpha_;
        double tension_;
        bool closed_;
        CompositeBezier bezier_;
    };
}
