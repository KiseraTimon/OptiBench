#pragma once

/*
 * Special Notes: Composite (Piecewise) Bezier Curves
 *
 * A composite curve is a chain of Bezier segments S0, S1, ..., S(N-1). It is evaluated with a
 * global parameter u in [0, N]: segment i covers u in [i, i + 1] with local t = u - i, so
 * d/du = d/dt and derivatives are directly comparable across joins.
 *
 * Continuity at the join between a segment P (degree n) and the next segment Q (degree m):
 *
 *   C0   positions match:        Pn = Q0
 *   G1   C0 + same unit tangent:  (Pn - P(n-1)) and (Q1 - Q0) point the same way
 *   C1   C0 + equal derivatives:  n (Pn - P(n-1)) = m (Q1 - Q0)
 *
 * The enforce helpers keep the earlier segment fixed and move the later segment's first
 * control point (C0) and second control point (G1/C1). Note that for a linear later segment the
 * second control point is its endpoint, so enforcing G1/C1 there moves that endpoint.
 * On a closed chain the closing join moves segment 0's second control point; for a quadratic
 * segment 0 that also turns its end tangent, so closed G1/C1 loops should use cubic segments.
 *
 * Every spline type in ob::curves converts to a CompositeBezier, so flattening, bounds,
 * ray intersection and drawing are implemented once, here and in OpticalCurve.h.
 */

#include <span>
#include <utility>
#include <vector>

#include "optibench/curves/Bezier.h"

namespace ob::curves {

    // geometric continuity classes at a join, ordered from weakest to strongest
    enum class Continuity { None = 0, C0 = 1, G1 = 2, C1 = 3 };

    class CompositeBezier {
    public:
        // an empty curve (no segments)
        CompositeBezier() = default;

        // a chain of segments, taken as given (no continuity is enforced)
        explicit CompositeBezier(std::vector<Bezier> segments) : segments_(std::move(segments)) {}

        // appends a segment as given
        void append(Bezier segment) { segments_.push_back(std::move(segment)); }

        // appends a segment and enforces the requested continuity at the new join
        void append(Bezier segment, Continuity continuity);

        /*
         * Access
         */
        bool empty() const { return segments_.empty(); }
        std::size_t segmentCount() const { return segments_.size(); }
        std::span<const Bezier> segments() const { return segments_; }
        const Bezier& segment(std::size_t i) const { return segments_.at(i); }
        void setSegment(std::size_t i, Bezier segment) { segments_.at(i) = std::move(segment); }

        // largest global parameter (the number of segments)
        double paramMax() const { return static_cast<double>(segments_.size()); }

        // maps a global parameter (clamped to [0, N]) to (segment index, local t)
        std::pair<std::size_t, double> locate(double u) const;

        /*
         * Evaluation (empty curves return zero vectors / 0)
         */
        Vec2 evaluate(double u) const;
        Vec2 derivative(double u, int order = 1) const;
        Vec2 tangent(double u) const;
        Vec2 normal(double u) const;
        double curvature(double u) const;

        // first and last points
        Vec2 front() const { return empty() ? Vec2{} : segments_.front().front(); }
        Vec2 back() const { return empty() ? Vec2{} : segments_.back().back(); }

        // true if the last point coincides with the first (within tol)
        bool isClosed(double tol = 1e-9) const;

        /*
         * Continuity
         */
        /**
         * number of joins: N - 1 for open curves, N for closed curves (the extra join N - 1
         * connects the last segment back to the first)
         */
        std::size_t joinCount() const;

        /**
         * strongest continuity satisfied at a join (between segment join and join + 1)
         * @param tol double: relative tolerance for positions/derivatives, absolute for tangent cosines
         */
        Continuity continuityAt(std::size_t join, double tol = 1e-9) const;

        // weakest continuity over all joins (C1 for a single segment)
        Continuity continuity(double tol = 1e-9) const;

        // moves control points of segment join + 1 so the join satisfies c (None does nothing)
        void enforce(std::size_t join, Continuity c);

        // enforces c at every join, left to right
        void enforceAll(Continuity c);

        /*
         * Geometry
         */
        // adaptive flattening of every segment (shared join points are not repeated)
        Polyline flatten(double tolerance) const;

        // fixed-step flattening with stepsPerSegment steps per segment
        Polyline flattenUniform(int stepsPerSegment) const;

        // tight and control-polygon bounding boxes
        Aabb bounds() const;
        Aabb controlBounds() const;

        // total arc length
        double arcLength() const;

        // global parameter of the nearest point to p (0 for an empty curve)
        double closestParam(Vec2 p) const;

        // all control points in order (shared join points repeated per segment)
        std::vector<Vec2> controlPolygon() const;

    private:
        std::vector<Bezier> segments_;
    };
}
