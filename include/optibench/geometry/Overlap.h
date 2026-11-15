#pragma once

/*
 * Special Notes: Overlap tests
 *
 * "Overlap" here always means the INTERIORS share some area deeper than a tolerance. Shapes that
 * only touch (a shared edge or corner) do not overlap. That is the test an optical bench needs: two
 * glass blocks may sit flush against each other, but must not occupy the same space.
 */

#include <span>

#include "optibench/geometry/Primitives.h"

namespace ob::geom {

    /**
     * Result of a Separating Axis Theorem test between two convex shapes A and B.
     *      overlapping  true if the interiors overlap by more than the tolerance used
     *      normal       unit axis of least penetration, pointing from A towards B
     *      depth        how far B must move along `normal` to just touch A (0 if not overlapping)
     * Moving B by normal * depth (or A by -normal * depth) separates them: the minimum translation vector.
     */
    struct Separation {
        bool overlapping = false;
        Vec2 normal;
        double depth = 0.0;
    };

    /**
     * SAT for two convex polygons (either winding), O(n + m) axes, O((n + m)^2) work.
     * Also gives the correct push-out depth when one polygon contains the other.
     * Inputs that are not convex give the result for their convex hulls.
     */
    Separation separateConvex(std::span<const Vec2> a, std::span<const Vec2> b, double tol = kEps);

    // SAT for a convex polygon and a disc (the extra axis runs from the disc centre to the nearest vertex)
    Separation separateConvexCircle(std::span<const Vec2> poly, Vec2 center, double radius, double tol = kEps);

    // two discs
    Separation separateCircles(Vec2 c1, double r1, Vec2 c2, double r2, double tol = kEps);

    /**
     * true if two SIMPLE polygons (convex or concave) overlap.
     * Both are triangulated and the triangle pairs are tested with SAT, after a bounding-box reject.
     * Polygons that are not simple fall back to an edge-crossing and containment test.
     */
    bool polygonsOverlap(std::span<const Vec2> a, std::span<const Vec2> b, double tol = kEps);

    // true if a simple polygon (convex or concave) and a disc overlap
    bool polygonCircleOverlap(std::span<const Vec2> poly, Vec2 center, double radius, double tol = kEps);
}
