#include "optibench/geometry/PolygonOps.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <utility>


namespace ob::geom {

    // ------------------------------------------------------------------ measures

    double signedArea(std::span<const Vec2> poly) {
        const std::size_t n = poly.size();
        if (n < 3) return 0.0;

        // shoelace, taken relative to the first vertex: same result, much less cancellation far from (0, 0)
        double sum = 0.0;
        const Vec2 o = poly[0];
        for (std::size_t i = 1; i + 1 < n; ++i) sum += (poly[i] - o).cross(poly[i + 1] - o);
        return sum / 2.0;
    }

    double area(std::span<const Vec2> poly) { return std::fabs(signedArea(poly)); }

    double perimeter(std::span<const Vec2> poly) {
        const std::size_t n = poly.size();
        if (n < 2) return 0.0;
        if (n == 2) return 2.0 * (poly[1] - poly[0]).length();     // there and back
        double total = 0.0;
        for (std::size_t i = 0; i < n; ++i) total += (poly[(i + 1) % n] - poly[i]).length();
        return total;
    }

    Vec2 centroid(std::span<const Vec2> poly) {
        const std::size_t n = poly.size();
        if (n == 0) return {};

        // area-weighted centroid of the fan of triangles from vertex 0
        const Vec2 o = poly[0];
        double a2 = 0.0;
        Vec2 acc{};
        for (std::size_t i = 1; i + 1 < n; ++i) {
            const Vec2 p = poly[i] - o, q = poly[i + 1] - o;
            const double w = p.cross(q);
            a2 += w;
            acc = acc + (p + q) * w;
        }
        const double scale = std::max(1.0, perimeter(poly));
        if (std::fabs(a2) > kRelEps * scale * scale) return o + acc / (3.0 * a2);

        // flat polygon: centre of the outline, edges weighted by length
        double total = 0.0;
        Vec2 mid{};
        for (std::size_t i = 0; i < n; ++i) {
            const Vec2 a = poly[i], b = poly[(i + 1) % n];
            const double len = (b - a).length();
            total += len;
            mid = mid + (a + b) * (0.5 * len);
        }
        return total > 0.0 ? mid / total : o;
    }

    // ------------------------------------------------------------------ orientation and shape

    Winding winding(std::span<const Vec2> poly) {
        const double a = signedArea(poly);
        const double scale = std::max(1.0, perimeter(poly));
        if (std::fabs(a) <= kRelEps * scale * scale) return Winding::Degenerate;
        return a > 0.0 ? Winding::CounterClockwise : Winding::Clockwise;
    }

    std::vector<Vec2> toCounterClockwise(std::span<const Vec2> poly) {
        std::vector<Vec2> out(poly.begin(), poly.end());
        if (winding(poly) == Winding::Clockwise) std::reverse(out.begin(), out.end());
        return out;
    }

    std::vector<Vec2> cleaned(std::span<const Vec2> poly, double tol) {
        // 1. drop repeated consecutive vertices (closing pair included)
        std::vector<Vec2> pts;
        pts.reserve(poly.size());
        for (const Vec2& p : poly) {
            if (pts.empty() || !samePoint(pts.back(), p, tol)) pts.push_back(p);
        }
        while (pts.size() > 1 && samePoint(pts.front(), pts.back(), tol)) pts.pop_back();

        // 2. sweep away straight-through vertices until a full sweep removes nothing
        //    (removing one can make its neighbour straight, hence the repeat)
        bool changed = true;
        while (changed && pts.size() > 2) {
            changed = false;
            std::vector<Vec2> kept;
            kept.reserve(pts.size());
            const std::size_t n = pts.size();
            for (std::size_t i = 0; i < n; ++i) {
                const Vec2 prev = kept.empty() ? pts[n - 1] : kept.back();
                const Vec2 cur = pts[i], next = pts[(i + 1) % n];
                const bool straight = orientation(prev, cur, next) == Orientation::Collinear &&
                                      (cur - prev).dot(next - cur) > 0.0;
                if (straight && n - (i - kept.size()) > 3) changed = true;     // keep at least 3
                else kept.push_back(cur);
            }
            pts = std::move(kept);
        }
        return pts;
    }

    bool isSimple(std::span<const Vec2> poly) {
        const std::size_t n = poly.size();
        if (n < 3 || winding(poly) == Winding::Degenerate) return false;
        for (std::size_t i = 0; i < n; ++i) {
            if (samePoint(poly[i], poly[(i + 1) % n])) return false;
        }

        for (std::size_t i = 0; i < n; ++i) {
            const Segment ei = edge(poly, i);
            for (std::size_t j = i + 1; j < n; ++j) {
                const Segment ej = edge(poly, j);
                const SegmentIntersection x = intersect(ei, ej);
                if (!x) continue;

                const bool next = j == i + 1;               // ej starts where ei ends
                const bool wrap = i == 0 && j == n - 1;     // ej ends where ei starts
                if (!next && !wrap) return false;           // non-neighbours must not touch at all

                // neighbours may only share their one common vertex (no folding back along each other)
                if (x.kind == SegmentIntersection::Kind::Overlap) return false;
                const Vec2 shared = next ? ei.b : ei.a;
                if (!samePoint(x.a, shared)) return false;
            }
        }
        return true;
    }

    bool isConvex(std::span<const Vec2> poly) {
        const std::vector<Vec2> pts = cleaned(poly);
        const std::size_t n = pts.size();
        if (n < 3) return false;

        // every turn the same way, and the outline goes round exactly once
        int sign = 0;
        double turning = 0.0;
        for (std::size_t i = 0; i < n; ++i) {
            const Vec2 a = pts[i], b = pts[(i + 1) % n], c = pts[(i + 2) % n];
            const Orientation o = orientation(a, b, c);
            if (o == Orientation::Collinear) return false;      // cleaned() left a spike
            const int s = o == Orientation::CounterClockwise ? 1 : -1;
            if (sign == 0) sign = s;
            else if (s != sign) return false;
            const Vec2 u = b - a, v = c - b;
            turning += std::atan2(u.cross(v), u.dot(v));
        }
        return std::fabs(std::fabs(turning) - 2.0 * kPi) < 1e-6;
    }

    // ------------------------------------------------------------------ points

    int windingNumber(std::span<const Vec2> poly, Vec2 p) {
        const std::size_t n = poly.size();
        if (n < 3) return 0;

        // Sunday: count upward crossings with p on the left minus downward crossings with p on the right
        int wn = 0;
        for (std::size_t i = 0; i < n; ++i) {
            const Vec2 a = poly[i], b = poly[(i + 1) % n];
            if (a.y <= p.y) {
                if (b.y > p.y && cross(a, b, p) > 0.0) ++wn;
            } else {
                if (b.y <= p.y && cross(a, b, p) < 0.0) --wn;
            }
        }
        return wn;
    }

    double distanceToBoundary(std::span<const Vec2> poly, Vec2 p) {
        const std::size_t m = edgeCount(poly);
        if (poly.empty()) return std::numeric_limits<double>::infinity();
        if (m == 0) return (p - poly[0]).length();
        double best = std::numeric_limits<double>::infinity();
        for (std::size_t i = 0; i < m; ++i) best = std::min(best, distanceToSegment(p, edge(poly, i)));
        return best;
    }

    PointLocation locate(std::span<const Vec2> poly, Vec2 p, double tol) {
        if (poly.empty()) return PointLocation::Outside;
        if (distanceToBoundary(poly, p) <= tol) return PointLocation::Boundary;
        return windingNumber(poly, p) != 0 ? PointLocation::Inside : PointLocation::Outside;
    }

    bool contains(std::span<const Vec2> poly, Vec2 p, double tol) {
        return locate(poly, p, tol) != PointLocation::Outside;
    }

    double signedDistance(std::span<const Vec2> poly, Vec2 p) {
        const double d = distanceToBoundary(poly, p);
        return windingNumber(poly, p) != 0 ? -d : d;
    }

    // ------------------------------------------------------------------ edges

    std::size_t edgeCount(std::span<const Vec2> poly) {
        if (poly.size() >= 3) return poly.size();
        return poly.size() == 2 ? 1 : 0;
    }

    Segment edge(std::span<const Vec2> poly, std::size_t i) {
        const std::size_t n = poly.size();
        return {poly[i % n], poly[(i + 1) % n]};
    }

    Vec2 outwardNormal(std::span<const Vec2> poly, std::size_t i) {
        const Winding w = winding(poly);
        if (w == Winding::Degenerate) return {};

        // interior is on the left of each edge for CCW: outward is the right-hand normal (dy, -dx)
        const Segment e = edge(poly, i);
        const Vec2 d = e.b - e.a;
        const Vec2 right{d.y, -d.x};
        return (w == Winding::CounterClockwise ? right : -right).normalized();
    }

    std::size_t nearestEdge(std::span<const Vec2> poly, Vec2 p) {
        const std::size_t m = edgeCount(poly);
        std::size_t best = 0;
        double bestD = std::numeric_limits<double>::infinity();
        for (std::size_t i = 0; i < m; ++i) {
            const double d = distanceToSegment(p, edge(poly, i));
            if (d < bestD) { bestD = d; best = i; }
        }
        return best;
    }

    // ------------------------------------------------------------------ construction

    std::vector<Vec2> convexHull(std::span<const Vec2> points) {
        std::vector<Vec2> pts(points.begin(), points.end());
        std::sort(pts.begin(), pts.end(), [](Vec2 a, Vec2 b) { return a.x < b.x || (a.x == b.x && a.y < b.y); });
        pts.erase(std::unique(pts.begin(), pts.end(), [](Vec2 a, Vec2 b) { return samePoint(a, b); }), pts.end());
        if (pts.size() < 3) return pts;

        // lower hull left to right, then upper hull right to left; pop anything that is not a strict left turn
        std::vector<Vec2> hull(2 * pts.size());
        std::size_t k = 0;
        for (const Vec2& p : pts) {
            while (k >= 2 && orientation(hull[k - 2], hull[k - 1], p) != Orientation::CounterClockwise) --k;
            hull[k++] = p;
        }
        for (std::size_t i = pts.size() - 1, lower = k + 1; i-- > 0;) {
            const Vec2 p = pts[i];
            while (k >= lower && orientation(hull[k - 2], hull[k - 1], p) != Orientation::CounterClockwise) --k;
            hull[k++] = p;
        }
        hull.resize(k - 1);     // the last point repeats the first

        // all collinear: the two passes collapse to the two extreme points
        if (hull.size() < 3) return {pts.front(), pts.back()};
        return hull;
    }

    namespace {
        // true if p is inside or on triangle (a, b, c), which must be counter-clockwise
        bool inTriangleCCW(Vec2 p, Vec2 a, Vec2 b, Vec2 c) {
            return orientation(a, b, p) != Orientation::Clockwise &&
                   orientation(b, c, p) != Orientation::Clockwise &&
                   orientation(c, a, p) != Orientation::Clockwise;
        }
    }

    std::vector<Triangle> triangulate(std::span<const Vec2> poly) {
        std::vector<Triangle> tris;
        if (!isSimple(poly)) return tris;

        // work on indices, always counter-clockwise, so every ear is a CCW triangle
        const std::size_t n = poly.size();
        std::vector<std::size_t> idx(n);
        for (std::size_t i = 0; i < n; ++i) idx[i] = i;
        if (winding(poly) == Winding::Clockwise) std::reverse(idx.begin(), idx.end());

        // drop straight-through vertices first: clipped as part of an ear they would only add triangles
        for (bool changed = true; changed && idx.size() > 3;) {
            changed = false;
            for (std::size_t k = 0; k < idx.size() && idx.size() > 3; ++k) {
                const std::size_t m = idx.size();
                const Vec2 a = poly[idx[(k + m - 1) % m]], b = poly[idx[k]], c = poly[idx[(k + 1) % m]];
                if (orientation(a, b, c) == Orientation::Collinear && (b - a).dot(c - b) > 0.0) {
                    idx.erase(idx.begin() + static_cast<std::ptrdiff_t>(k));
                    changed = true;
                    break;      // indices shifted: rescan from the start
                }
            }
        }
        tris.reserve(idx.size() - 2);

        std::size_t guard = 0;      // each pass removes one vertex; never loop forever on bad input
        std::size_t i = 0;
        while (idx.size() > 3 && guard < 2 * n * n) {
            ++guard;
            const std::size_t m = idx.size();
            const std::size_t ip = idx[(i + m - 1) % m], ic = idx[i % m], in = idx[(i + 1) % m];
            const Vec2 a = poly[ip], b = poly[ic], c = poly[in];
            const Orientation turn = orientation(a, b, c);

            // a straight-through vertex adds no area: drop it without emitting a triangle
            if (turn == Orientation::Collinear && (b - a).dot(c - b) > 0.0) {
                idx.erase(idx.begin() + static_cast<std::ptrdiff_t>(i % m));
                i = i % idx.size();
                continue;
            }

            bool ear = turn == Orientation::CounterClockwise;
            if (ear) {
                // no other remaining vertex may sit inside (or on) the candidate ear
                for (std::size_t j = 0; j < m && ear; ++j) {
                    const std::size_t v = idx[j];
                    if (v == ip || v == ic || v == in) continue;
                    // a vertex repeated at the same position (touching outline) does not block the ear
                    if (samePoint(poly[v], a) || samePoint(poly[v], b) || samePoint(poly[v], c)) continue;
                    if (inTriangleCCW(poly[v], a, b, c)) ear = false;
                }
            }
            if (ear) {
                tris.push_back({ip, ic, in});
                idx.erase(idx.begin() + static_cast<std::ptrdiff_t>(i % m));
                i = (i + idx.size() - 1) % idx.size();      // re-test the previous vertex: it may be an ear now
            } else {
                i = (i + 1) % m;
            }
        }
        // a simple polygon always has an ear (two-ears theorem); running out of them can only mean
        // numerically hostile input. Report failure rather than a partial tiling.
        if (idx.size() != 3) return {};
        if (orientation(poly[idx[0]], poly[idx[1]], poly[idx[2]]) == Orientation::CounterClockwise) {
            tris.push_back({idx[0], idx[1], idx[2]});
        }
        return tris;
    }

    std::vector<Vec2> regularPolygon(Vec2 center, double radius, int n, double phase) {
        std::vector<Vec2> out;
        if (n < 3) return out;
        out.reserve(static_cast<std::size_t>(n));
        for (int i = 0; i < n; ++i) {
            const double a = phase + 2.0 * kPi * i / n;
            out.push_back(center + Vec2{std::cos(a), std::sin(a)} * radius);
        }
        return out;
    }

    std::vector<Vec2> rectangle(const Aabb& box) {
        if (box.empty()) return {};
        return {box.min, {box.max.x, box.min.y}, box.max, {box.min.x, box.max.y}};
    }

    std::vector<Vec2> transformed(std::span<const Vec2> poly, const Transform2D& t) {
        std::vector<Vec2> out;
        out.reserve(poly.size());
        for (const Vec2& p : poly) out.push_back(t.toWorld(p));
        return out;
    }
}
