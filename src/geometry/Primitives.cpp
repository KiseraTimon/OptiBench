#include "optibench/geometry/Primitives.h"

#include <algorithm>
#include <cmath>
#include <utility>

namespace ob::geom {

    Orientation orientation(Vec2 a, Vec2 b, Vec2 c) {
        const Vec2 ab = b - a, ac = c - a;
        const double z = ab.cross(ac);
        const double scale = std::sqrt(lengthSq(ab) * lengthSq(ac));
        if (std::fabs(z) <= kRelEps * scale) return Orientation::Collinear;
        return z > 0.0 ? Orientation::CounterClockwise : Orientation::Clockwise;
    }

    double projectParam(Vec2 p, Vec2 a, Vec2 b) {
        const Vec2 ab = b - a;
        const double len2 = lengthSq(ab);
        return len2 > 0.0 ? (p - a).dot(ab) / len2 : 0.0;
    }

    Vec2 closestPointOnSegment(Vec2 p, const Segment& s) {
        const double t = std::clamp(projectParam(p, s.a, s.b), 0.0, 1.0);
        return s.a + (s.b - s.a) * t;
    }

    bool onSegment(Vec2 p, const Segment& s, double tol) {
        return samePoint(p, closestPointOnSegment(p, s), tol);
    }

    SegmentIntersection intersect(const Segment& s1, const Segment& s2, double tol) {
        SegmentIntersection out;
        const bool point1 = samePoint(s1.a, s1.b, tol);
        const bool point2 = samePoint(s2.a, s2.b, tol);

        // degenerate cases: a point against a point or a segment
        if (point1 || point2) {
            const Vec2 p = point1 ? s1.a : s2.a;
            const Segment& other = point1 ? s2 : s1;
            if (onSegment(p, other, tol)) {
                out.kind = SegmentIntersection::Kind::Point;
                out.a = out.b = p;
            }
            return out;
        }

        const Orientation o1 = orientation(s1.a, s1.b, s2.a);
        const Orientation o2 = orientation(s1.a, s1.b, s2.b);
        const Orientation o3 = orientation(s2.a, s2.b, s1.a);
        const Orientation o4 = orientation(s2.a, s2.b, s1.b);

        // collinear: project s2 onto s1's parameter line and intersect the two intervals
        if (o1 == Orientation::Collinear && o2 == Orientation::Collinear) {
            // the predicate is relative: also require the points to be close in absolute terms
            if (!samePoint(s2.a, s1.a + (s1.b - s1.a) * projectParam(s2.a, s1.a, s1.b), tol) &&
                !samePoint(s2.b, s1.a + (s1.b - s1.a) * projectParam(s2.b, s1.a, s1.b), tol)) {
                return out;
            }
            double t0 = projectParam(s2.a, s1.a, s1.b);
            double t1 = projectParam(s2.b, s1.a, s1.b);
            if (t0 > t1) std::swap(t0, t1);
            const double lo = std::max(0.0, t0), hi = std::min(1.0, t1);
            const double tolT = tol / (s1.b - s1.a).length();
            if (lo > hi + tolT) return out;

            const Vec2 d = s1.b - s1.a;
            out.a = s1.a + d * std::clamp(lo, 0.0, 1.0);
            out.b = s1.a + d * std::clamp(hi, 0.0, 1.0);
            out.kind = samePoint(out.a, out.b, tol) ? SegmentIntersection::Kind::Point
                                                    : SegmentIntersection::Kind::Overlap;
            if (out.kind == SegmentIntersection::Kind::Point) out.b = out.a;
            return out;
        }

        // general position: each segment's endpoints must not be strictly on the same side of the other
        const bool straddle1 = o1 != o2 || o1 == Orientation::Collinear || o2 == Orientation::Collinear;
        const bool straddle2 = o3 != o4 || o3 == Orientation::Collinear || o4 == Orientation::Collinear;
        if (!straddle1 || !straddle2) {
            // still touching if an endpoint lies on the other segment within tol
            for (const auto& [p, seg] : {std::pair{s2.a, s1}, std::pair{s2.b, s1},
                                         std::pair{s1.a, s2}, std::pair{s1.b, s2}}) {
                if (onSegment(p, seg, tol)) {
                    out.kind = SegmentIntersection::Kind::Point;
                    out.a = out.b = p;
                    return out;
                }
            }
            return out;
        }

        // the lines cross inside both segments: solve s1.a + t r = s2.a + u q for t
        const Vec2 r = s1.b - s1.a, q = s2.b - s2.a;
        const double denom = r.cross(q);
        if (denom == 0.0) {
            // only reachable when one endpoint sits on the other line: report that endpoint
            for (const Vec2 p : {s2.a, s2.b, s1.a, s1.b}) {
                if (onSegment(p, s1, tol) && onSegment(p, s2, tol)) {
                    out.kind = SegmentIntersection::Kind::Point;
                    out.a = out.b = p;
                    return out;
                }
            }
            return out;
        }
        const double t = std::clamp((s2.a - s1.a).cross(q) / denom, 0.0, 1.0);
        out.kind = SegmentIntersection::Kind::Point;
        out.a = out.b = s1.a + r * t;
        return out;
    }

    bool properlyCross(const Segment& s1, const Segment& s2) {
        const Orientation o1 = orientation(s1.a, s1.b, s2.a);
        const Orientation o2 = orientation(s1.a, s1.b, s2.b);
        const Orientation o3 = orientation(s2.a, s2.b, s1.a);
        const Orientation o4 = orientation(s2.a, s2.b, s1.b);
        return o1 != Orientation::Collinear && o2 != Orientation::Collinear &&
               o3 != Orientation::Collinear && o4 != Orientation::Collinear &&
               o1 != o2 && o3 != o4;
    }

    Aabb boundsOf(const Segment& s) {
        Aabb box;
        box.expand(s.a);
        box.expand(s.b);
        return box;
    }

    Aabb boundsOfCircle(Vec2 center, double radius) {
        Aabb box;
        const double r = std::fabs(radius);
        box.expand(center - Vec2{r, r});
        box.expand(center + Vec2{r, r});
        return box;
    }

    Aabb inflate(const Aabb& box, double r) {
        if (box.empty()) return box;
        Aabb out = box;
        out.min = out.min - Vec2{r, r};
        out.max = out.max + Vec2{r, r};
        if (out.min.x > out.max.x || out.min.y > out.max.y) return Aabb{};   // shrunk past nothing
        return out;
    }

    bool overlaps(const Aabb& a, const Aabb& b, double tol) {
        if (a.empty() || b.empty()) return false;
        return a.min.x <= b.max.x + tol && b.min.x <= a.max.x + tol &&
               a.min.y <= b.max.y + tol && b.min.y <= a.max.y + tol;
    }
}
