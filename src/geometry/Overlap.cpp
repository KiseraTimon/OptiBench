#include "optibench/geometry/Overlap.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <vector>

#include "optibench/geometry/PolygonOps.h"

namespace ob::geom {
    namespace {
        struct Projection { double lo, hi; };

        Projection project(std::span<const Vec2> pts, Vec2 axis) {
            Projection p{std::numeric_limits<double>::infinity(), -std::numeric_limits<double>::infinity()};
            for (const Vec2& v : pts) {
                const double s = v.dot(axis);
                p.lo = std::min(p.lo, s);
                p.hi = std::max(p.hi, s);
            }
            return p;
        }

        /**
         * how far the intervals overlap along one axis, counting containment correctly:
         * if one interval holds the other, it must be pushed past the nearer end.
         */
        double overlapDepth(Projection a, Projection b) {
            const double o = std::min(a.hi, b.hi) - std::max(a.lo, b.lo);
            if (o <= 0.0) return o;
            const bool contained = (a.lo <= b.lo && b.hi <= a.hi) || (b.lo <= a.lo && a.hi <= b.hi);
            if (contained) return o + std::min(std::fabs(a.lo - b.lo), std::fabs(a.hi - b.hi));
            return o;
        }

        // the SAT loop shared by every convex test: smallest overlap over the candidate axes
        template <class ProjA, class ProjB>
        Separation sat(const std::vector<Vec2>& axes, ProjA projA, ProjB projB, Vec2 centerA, Vec2 centerB, double tol) {
            Separation best;
            best.depth = std::numeric_limits<double>::infinity();
            for (const Vec2& axis : axes) {
                const double depth = overlapDepth(projA(axis), projB(axis));
                if (depth <= tol) return Separation{};      // a separating axis: no overlap
                if (depth < best.depth) {
                    best.depth = depth;
                    best.normal = axis;
                }
            }
            if (axes.empty()) return Separation{};
            if ((centerB - centerA).dot(best.normal) < 0.0) best.normal = -best.normal;
            best.overlapping = true;
            return best;
        }

        // unit edge normals of a convex outline (axes only, so the sign does not matter)
        void addEdgeAxes(std::vector<Vec2>& axes, std::span<const Vec2> poly) {
            const std::size_t n = poly.size();
            for (std::size_t i = 0; i < n; ++i) {
                const Vec2 e = poly[(i + 1) % n] - poly[i];
                if (e.length() > kEps) axes.push_back(e.perp().normalized());
            }
        }

        bool trianglesOverlap(const std::array<Vec2, 3>& a, const std::array<Vec2, 3>& b, double tol) {
            return separateConvex(a, b, tol).overlapping;
        }

        // fallback for non-simple outlines: edges cross, or a vertex of one lies inside the other
        bool roughOverlap(std::span<const Vec2> a, std::span<const Vec2> b, double tol) {
            for (std::size_t i = 0; i < edgeCount(a); ++i) {
                for (std::size_t j = 0; j < edgeCount(b); ++j) {
                    if (properlyCross(edge(a, i), edge(b, j))) return true;
                }
            }
            for (const Vec2& p : a) if (locate(b, p, tol) == PointLocation::Inside) return true;
            for (const Vec2& p : b) if (locate(a, p, tol) == PointLocation::Inside) return true;
            return false;
        }
    }

    Separation separateConvex(std::span<const Vec2> a, std::span<const Vec2> b, double tol) {
        const std::vector<Vec2> ha = convexHull(a), hb = convexHull(b);
        if (ha.size() < 3 || hb.size() < 3) return Separation{};    // no interior, nothing to overlap

        std::vector<Vec2> axes;
        axes.reserve(ha.size() + hb.size());
        addEdgeAxes(axes, ha);
        addEdgeAxes(axes, hb);
        return sat(axes,
                   [&](Vec2 ax) { return project(ha, ax); },
                   [&](Vec2 ax) { return project(hb, ax); },
                   centroid(ha), centroid(hb), tol);
    }

    Separation separateConvexCircle(std::span<const Vec2> poly, Vec2 center, double radius, double tol) {
        const std::vector<Vec2> h = convexHull(poly);
        if (h.size() < 3 || !(radius > 0.0)) return Separation{};

        std::vector<Vec2> axes;
        axes.reserve(h.size() + 1);
        addEdgeAxes(axes, h);

        // the axis from the disc centre to the nearest vertex catches the corner case
        Vec2 nearest = h[0];
        for (const Vec2& v : h) if (lengthSq(v - center) < lengthSq(nearest - center)) nearest = v;
        if ((nearest - center).length() > kEps) axes.push_back((nearest - center).normalized());

        return sat(axes,
                   [&](Vec2 ax) { return project(h, ax); },
                   [&](Vec2 ax) { const double c = center.dot(ax); return Projection{c - radius, c + radius}; },
                   centroid(h), center, tol);
    }

    Separation separateCircles(Vec2 c1, double r1, Vec2 c2, double r2, double tol) {
        Separation s;
        if (!(r1 > 0.0) || !(r2 > 0.0)) return s;
        const Vec2 d = c2 - c1;
        const double dist = d.length();
        const double depth = r1 + r2 - dist;
        if (depth <= tol) return s;
        s.overlapping = true;
        s.depth = depth;
        s.normal = dist > kEps ? d / dist : Vec2{1.0, 0.0};      // concentric: any direction separates
        return s;
    }

    bool polygonsOverlap(std::span<const Vec2> a, std::span<const Vec2> b, double tol) {
        if (!overlaps(boundsOf(a), boundsOf(b), tol)) return false;

        const std::vector<Triangle> ta = triangulate(a), tb = triangulate(b);
        if (ta.empty() || tb.empty()) return roughOverlap(a, b, tol);

        for (const Triangle& x : ta) {
            const std::array<Vec2, 3> tx{a[x[0]], a[x[1]], a[x[2]]};
            const Aabb bx = boundsOf(tx);
            for (const Triangle& y : tb) {
                const std::array<Vec2, 3> ty{b[y[0]], b[y[1]], b[y[2]]};
                if (!overlaps(bx, boundsOf(ty), tol)) continue;
                if (trianglesOverlap(tx, ty, tol)) return true;
            }
        }
        return false;
    }

    bool polygonCircleOverlap(std::span<const Vec2> poly, Vec2 center, double radius, double tol) {
        if (poly.size() < 3 || !(radius > 0.0)) return false;
        // the disc reaches into the polygon if its centre is inside, or the outline passes closer than r
        if (locate(poly, center, tol) == PointLocation::Inside) return true;
        return distanceToBoundary(poly, center) < radius - tol;
    }
}
