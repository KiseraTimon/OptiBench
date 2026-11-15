#pragma once

/*
 * Special Notes: Ray queries
 *
 * Conventions shared with the rest of the codebase (Line.h, circle_ellipse.h, OpticalElement.h):
 *      - the ray direction need not be unit length; it is normalised, so t is a true distance
 *      - hits closer than tMin (default kEps) are ignored, so a ray restarted on a surface does
 *        not immediately hit that surface again
 *      - normals returned here point OUT of the shape (the solid-element convention); compare
 *        with the ray direction to know whether the ray is entering (d . n < 0) or leaving
 */

#include <cstddef>
#include <optional>
#include <span>
#include <vector>

#include "optibench/Ray.h"
#include "optibench/geometry/Primitives.h"

namespace ob::geom {

    // ------------------------------------------------------------------ boxes

    // the stretch [tNear, tFar] of a ray inside a box (tNear is 0 when the ray starts inside)
    struct RayInterval {
        double tNear = 0.0;
        double tFar = 0.0;
    };

    /**
     * slab method: the part of the ray (t >= 0) inside the box
     * Rays parallel to a slab are handled explicitly (no 0 * infinity NaNs).
     * @return empty if the ray misses the box, the box is empty or the direction is zero
     */
    std::optional<RayInterval> rayAabb(const Ray& ray, const Aabb& box);

    // ------------------------------------------------------------------ polygons

    // one crossing of a ray with a polygon outline
    struct PolygonHit {
        double t = 0.0;         // distance along the (normalised) ray
        Vec2 point;
        Vec2 normal;            // unit, pointing OUT of the polygon (averaged at a vertex)
        std::size_t edge = 0;   // index of the edge that was crossed (the first one at a vertex)
        bool entering = false;  // true if the ray goes from outside to inside here
    };

    /**
     * every crossing of the ray with the polygon outline, nearest first
     *
     * Works for concave polygons and either winding. Vertices are handled exactly once:
     *      - a ray that crosses the outline at a vertex gives ONE hit, with the averaged normal
     *      - a ray that only grazes a vertex (both neighbours on the same side) gives no hit
     * A ray running along an edge reports the corners where it meets the neighbouring edges.
     */
    std::vector<PolygonHit> rayPolygonAll(const Ray& ray, std::span<const Vec2> poly, double tMin = kEps);

    /**
     * nearest crossing as an ob::Hit (normal pointing OUT, as solid elements report it)
     * @return hit.hit false on a miss
     */
    Hit rayPolygon(const Ray& ray, std::span<const Vec2> poly, double tMin = kEps);

    // ------------------------------------------------------------------ convex regions (CSG)

    /**
     * Where a full line (t over all reals) is inside a convex region: [tIn, tOut], with the outward
     * normals of the boundaries it crosses there. Unbounded regions use +-infinity. Convex regions
     * combine by intersecting their spans, which is how Lens is built from discs and half-planes.
     */
    struct LineSpan {
        double tIn = 0.0, tOut = 0.0;
        Vec2 nIn, nOut;
    };

    // the line inside the disc
    std::optional<LineSpan> lineDisc(const Ray& ray, Vec2 center, double radius);

    // the line inside the half-plane {x : (x - point) . outward <= 0}
    std::optional<LineSpan> lineHalfPlane(const Ray& ray, Vec2 point, Vec2 outward);

    // the overlap of two spans (empty if they do not overlap)
    std::optional<LineSpan> intersectSpans(const LineSpan& a, const LineSpan& b);

    /**
     * first boundary crossing with t >= tMin: the entry point, or the exit point when the ray starts inside
     * @return hit.hit false if the whole span lies before tMin
     */
    Hit firstHit(const Ray& ray, const LineSpan& span, double tMin = kEps);
}
