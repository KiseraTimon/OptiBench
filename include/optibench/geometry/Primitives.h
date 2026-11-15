#pragma once

/*
 * Special Notes: Geometry primitives
 *
 * The small, exact-as-possible building blocks every other geometry routine is written on:
 *      - one tolerance policy for the whole module
 *      - the orientation predicate (which way three points turn)
 *      - closest points and point/segment relations
 *      - robust segment/segment classification (none, single point, collinear overlap)
 *      - axis-aligned bounding boxes (the curves module's Aabb, reused, plus overlap tests)
 *
 * Coordinates are scene units (pixels). Every formula is handedness-agnostic: "counter-clockwise"
 * means counter-clockwise in the maths axes (x right, y up). On screen, where y grows DOWN, that
 * same order looks clockwise. Nothing in this module depends on which way y points.
 */

#include <span>

#include "optibench/Line.h"             // Segment, distanceToSegment
#include "optibench/Vec2.h"
#include "optibench/curves/CurveMath.h" // curves::Aabb, curves::boundsOf

namespace ob::geom {

    /*
     * Tolerance policy
     *
     *      kEps        absolute length tolerance, in scene units. Two points closer than this are
     *                  the same point, and a point this close to an edge is on it.
     *      kRelEps     relative tolerance for cross products. orientation() calls three points
     *                  collinear when |(b - a) x (c - a)| <= kRelEps * |b - a| * |c - a|,
     *                  i.e. when the sine of the angle at a is below kRelEps. Being relative, it
     *                  behaves the same for a 1-pixel triangle and a 1000-pixel one.
     */
    inline constexpr double kEps = 1e-9;
    inline constexpr double kRelEps = 1e-12;

    // which way the path a -> b -> c turns
    enum class Orientation { Clockwise = -1, Collinear = 0, CounterClockwise = 1 };

    // twice the signed area of triangle (a, b, c): (b - a) x (c - a). Positive for a CCW turn.
    inline double cross(Vec2 a, Vec2 b, Vec2 c) { return (b - a).cross(c - a); }

    /**
     * the orientation predicate, with the relative tolerance described above
     * @return Collinear also when any two of the points coincide
     */
    Orientation orientation(Vec2 a, Vec2 b, Vec2 c);

    // squared length (avoids a square root in comparisons)
    inline double lengthSq(Vec2 v) { return v.dot(v); }

    // true if a and b are the same point within tol
    inline bool samePoint(Vec2 a, Vec2 b, double tol = kEps) { return lengthSq(a - b) <= tol * tol; }

    /**
     * parameter of the point on the infinite line a + t (b - a) closest to p
     * @return 0 when a == b (the line has no direction)
     */
    double projectParam(Vec2 p, Vec2 a, Vec2 b);

    // closest point to p on the segment s (an endpoint when the projection falls outside)
    Vec2 closestPointOnSegment(Vec2 p, const Segment& s);

    // true if p lies on segment s within distance tol
    bool onSegment(Vec2 p, const Segment& s, double tol = kEps);

    /**
     * How two segments meet.
     *      None     they do not touch
     *      Point    they meet at exactly one point (crossing, T-junction or shared endpoint): `a`
     *      Overlap  they are collinear and share the sub-segment a..b (a != b)
     */
    struct SegmentIntersection {
        enum class Kind { None, Point, Overlap };
        Kind kind = Kind::None;
        Vec2 a;
        Vec2 b;
        explicit operator bool() const { return kind != Kind::None; }
    };

    /**
     * classifies how two segments meet, using orientation tests rather than a division, so
     * touching endpoints and collinear overlaps are reported exactly instead of being lost to rounding.
     * Zero-length segments are treated as points.
     */
    SegmentIntersection intersect(const Segment& s1, const Segment& s2, double tol = kEps);

    // true only for a proper crossing: a single point interior to both segments
    bool properlyCross(const Segment& s1, const Segment& s2);

    /*
     * Axis-aligned bounding boxes: the curves module's box, so curve bounds and shape bounds mix.
     */
    using Aabb = curves::Aabb;

    // tight box around a set of points (empty box for no points)
    inline Aabb boundsOf(std::span<const Vec2> points) { return curves::boundsOf(points); }

    // box around a segment
    Aabb boundsOf(const Segment& s);

    // box around a disc
    Aabb boundsOfCircle(Vec2 center, double radius);

    // the box grown by r on every side (an empty box stays empty)
    Aabb inflate(const Aabb& box, double r);

    // true if the boxes share at least one point (grown by tol). Empty boxes overlap nothing.
    bool overlaps(const Aabb& a, const Aabb& b, double tol = 0.0);
}
