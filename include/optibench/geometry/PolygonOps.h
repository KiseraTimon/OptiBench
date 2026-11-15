#pragma once

/*
 * Special Notes: Polygon queries
 *
 * Every function takes the polygon as std::span<const Vec2>: the vertices in order, closed
 * implicitly (the last vertex joins the first). That one signature accepts both polygon types in
 * the codebase without converting either:
 *
 *      ob::Polygon from clip_fill.h   (std::vector<Vec2>)     geom::area(poly)
 *      ob::Polygon from Polygon.h     (struct, .vertices)     geom::area(poly.vertices)
 *
 * Polygons may be given in either winding order. Functions that need an order (outward normals,
 * triangulation) read it from the sign of the area.
 */

#include <array>
#include <cstddef>
#include <span>
#include <vector>

#include "optibench/Transform2D.h"
#include "optibench/geometry/Primitives.h"

namespace ob::geom {

    // ------------------------------------------------------------------ measures

    // shoelace formula: positive for counter-clockwise (maths axes), 0 for fewer than 3 vertices
    double signedArea(std::span<const Vec2> poly);

    // |signedArea|
    double area(std::span<const Vec2> poly);

    // total edge length, closing edge included (0 for fewer than 2 vertices)
    double perimeter(std::span<const Vec2> poly);

    /**
     * area centroid (centre of mass of the filled polygon)
     * Degenerate input falls back gracefully: zero area gives the centre of the edges weighted by
     * length (the midpoint of a flat polygon), and no length at all gives the first vertex.
     * An empty polygon gives (0, 0).
     */
    Vec2 centroid(std::span<const Vec2> poly);

    // ------------------------------------------------------------------ orientation and shape

    // the polygon's winding, read from its signed area
    enum class Winding { CounterClockwise, Clockwise, Degenerate };
    Winding winding(std::span<const Vec2> poly);

    // a copy wound counter-clockwise (maths axes); a degenerate polygon is returned unchanged
    std::vector<Vec2> toCounterClockwise(std::span<const Vec2> poly);

    /**
     * removes repeated consecutive vertices (closer than tol) and vertices where the outline runs
     * straight on (collinear with both neighbours and between them). Spikes that double back are
     * kept, so a self-overlapping polygon stays visibly broken for isSimple().
     */
    std::vector<Vec2> cleaned(std::span<const Vec2> poly, double tol = kEps);

    /**
     * true if no two edges touch except neighbours at their shared vertex.
     * Requires at least 3 vertices, no zero-length edges and a non-zero area. O(n^2).
     */
    bool isSimple(std::span<const Vec2> poly);

    /**
     * true for a simple polygon with no reflex vertex. Straight-through (collinear) vertices are
     * allowed. A star pentagon, which turns the same way at every vertex but crosses itself, is
     * rejected.
     */
    bool isConvex(std::span<const Vec2> poly);

    // ------------------------------------------------------------------ points

    enum class PointLocation { Outside, Boundary, Inside };

    /**
     * where p is relative to the polygon
     * Boundary means within tol of an edge. Otherwise the winding number decides (non-zero rule:
     * the same as even-odd for a simple polygon). Works for concave polygons and either winding.
     */
    PointLocation locate(std::span<const Vec2> poly, Vec2 p, double tol = kEps);

    // true if p is inside or on the boundary
    bool contains(std::span<const Vec2> poly, Vec2 p, double tol = kEps);

    /**
     * winding number of the closed outline around p (Sunday's crossing method, no trigonometry).
     * +1 inside a CCW polygon, -1 inside a CW one, 0 outside; undefined exactly on an edge.
     */
    int windingNumber(std::span<const Vec2> poly, Vec2 p);

    // shortest distance from p to the outline
    double distanceToBoundary(std::span<const Vec2> poly, Vec2 p);

    // distance to the outline, negative inside (a signed distance field for the polygon)
    double signedDistance(std::span<const Vec2> poly, Vec2 p);

    // ------------------------------------------------------------------ edges

    // number of edges (= number of vertices for 3 or more, 1 for a 2-point polygon, else 0)
    std::size_t edgeCount(std::span<const Vec2> poly);

    // edge i runs from vertex i to vertex (i + 1) % n
    Segment edge(std::span<const Vec2> poly, std::size_t i);

    /**
     * unit normal of edge i pointing OUT of the polygon, for either winding order.
     * Correct for concave polygons too: it comes from the winding, not from the centroid.
     * Zero vector for a zero-length edge or a degenerate polygon.
     */
    Vec2 outwardNormal(std::span<const Vec2> poly, std::size_t i);

    // index of the edge closest to p (0 for an empty polygon)
    std::size_t nearestEdge(std::span<const Vec2> poly, Vec2 p);

    // ------------------------------------------------------------------ construction

    /**
     * convex hull, Andrew's monotone chain, O(n log n)
     * @return hull vertices, counter-clockwise (maths axes), no repeated or collinear points.
     *         Fewer than 3 distinct points, or all collinear: the distinct extreme points (0, 1 or 2).
     */
    std::vector<Vec2> convexHull(std::span<const Vec2> points);

    using Triangle = std::array<std::size_t, 3>;

    /**
     * ear-clipping triangulation of a simple polygon, O(n^2)
     * @return triangles as vertex indices into `poly`, each counter-clockwise (maths axes).
     *         n - 2 triangles for n vertices, fewer when straight-through vertices are skipped.
     *         The triangle areas always add up to area(poly).
     *         Empty if the polygon is not simple (isSimple() is false); never a partial result.
     */
    std::vector<Triangle> triangulate(std::span<const Vec2> poly);

    // a regular n-gon (n >= 3), counter-clockwise, first vertex at angle `phase` (radians)
    std::vector<Vec2> regularPolygon(Vec2 center, double radius, int n, double phase = 0.0);

    // an axis-aligned rectangle as a counter-clockwise polygon
    std::vector<Vec2> rectangle(const Aabb& box);

    /**
     * every vertex carried from local to world space by t (t.toWorld). A mirroring transform
     * (one negative scale) reverses the winding; every query here accepts either winding.
     */
    std::vector<Vec2> transformed(std::span<const Vec2> poly, const Transform2D& t);
}
