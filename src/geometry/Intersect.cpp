#include "optibench/geometry/Intersect.h"

#include <algorithm>
#include <cmath>
#include <limits>

#include "optibench/geometry/PolygonOps.h"

namespace ob::geom {
    namespace {
        constexpr double kInf = std::numeric_limits<double>::infinity();

        // the ray's direction as a unit vector; zero if it has no direction
        Vec2 unitDir(const Ray& ray) { return ray.direction.normalized(); }
    }

    // ------------------------------------------------------------------ boxes

    std::optional<RayInterval> rayAabb(const Ray& ray, const Aabb& box) {
        if (box.empty()) return std::nullopt;
        const Vec2 d = unitDir(ray);
        if (d.x == 0.0 && d.y == 0.0) return std::nullopt;

        double tNear = 0.0, tFar = kInf;
        const double o[2] = {ray.origin.x, ray.origin.y};
        const double dir[2] = {d.x, d.y};
        const double lo[2] = {box.min.x, box.min.y};
        const double hi[2] = {box.max.x, box.max.y};

        for (int axis = 0; axis < 2; ++axis) {
            if (dir[axis] == 0.0) {
                // parallel to this slab: inside it for all t, or never
                if (o[axis] < lo[axis] || o[axis] > hi[axis]) return std::nullopt;
                continue;
            }
            double t0 = (lo[axis] - o[axis]) / dir[axis];
            double t1 = (hi[axis] - o[axis]) / dir[axis];
            if (t0 > t1) std::swap(t0, t1);
            tNear = std::max(tNear, t0);
            tFar = std::min(tFar, t1);
            if (tNear > tFar) return std::nullopt;
        }
        return RayInterval{tNear, tFar};
    }

    // ------------------------------------------------------------------ polygons

    std::vector<PolygonHit> rayPolygonAll(const Ray& ray, std::span<const Vec2> poly, double tMin) {
        std::vector<PolygonHit> hits;
        const std::size_t n = poly.size();
        const Vec2 d = unitDir(ray);
        if (n < 3 || (d.x == 0.0 && d.y == 0.0) || winding(poly) == Winding::Degenerate) return hits;

        // which side of the ray's line a point is on: +1 left, -1 right, 0 on the line
        const auto side = [&](Vec2 p) {
            const Orientation o = orientation(ray.origin, ray.origin + d, p);
            return o == Orientation::CounterClockwise ? 1 : o == Orientation::Clockwise ? -1 : 0;
        };

        // one candidate per edge, remembering whether the crossing is at the edge's start or end vertex
        struct Candidate { PolygonHit hit; int vertex; };      // vertex: -1 interior, else the vertex index
        std::vector<Candidate> cands;
        for (std::size_t i = 0; i < n; ++i) {
            const Vec2 a = poly[i], b = poly[(i + 1) % n];
            const Vec2 q = b - a;
            const double qLen = q.length();
            if (qLen == 0.0) continue;

            const double denom = d.cross(q);
            if (std::fabs(denom) <= kRelEps * qLen) continue;          // parallel: corners are found by neighbours

            const Vec2 w = a - ray.origin;
            const double t = w.cross(q) / denom;
            const double u = w.cross(d) / denom;
            const double uTol = kEps / qLen;
            if (u < -uTol || u > 1.0 + uTol || t < tMin) continue;

            Candidate c;
            c.hit.t = t;
            c.hit.point = ray.origin + d * t;
            c.hit.normal = outwardNormal(poly, i);
            c.hit.edge = i;
            c.vertex = u <= uTol ? static_cast<int>(i) : u >= 1.0 - uTol ? static_cast<int>((i + 1) % n) : -1;
            cands.push_back(c);
        }

        // resolve vertex crossings: each vertex is reported by both of its edges
        std::vector<bool> vertexDone(n, false);
        for (std::size_t k = 0; k < cands.size(); ++k) {
            Candidate& c = cands[k];
            if (c.vertex < 0) { hits.push_back(c.hit); continue; }

            const std::size_t v = static_cast<std::size_t>(c.vertex);
            if (vertexDone[v]) continue;
            vertexDone[v] = true;

            // both neighbours on the same side of the ray: it only grazes the corner
            const int sPrev = side(poly[(v + n - 1) % n]);
            const int sNext = side(poly[(v + 1) % n]);
            if (sPrev != 0 && sPrev == sNext) continue;

            // a real crossing: merge with the other edge's candidate at this vertex (if any)
            PolygonHit h = c.hit;
            for (std::size_t j = k + 1; j < cands.size(); ++j) {
                if (cands[j].vertex == c.vertex) {
                    h.normal = (h.normal + cands[j].hit.normal).normalized();
                    h.edge = std::min(h.edge, cands[j].hit.edge);
                    break;
                }
            }
            h.point = poly[v];
            hits.push_back(h);
        }

        for (PolygonHit& h : hits) h.entering = d.dot(h.normal) < 0.0;
        std::sort(hits.begin(), hits.end(), [](const PolygonHit& a, const PolygonHit& b) { return a.t < b.t; });
        return hits;
    }

    Hit rayPolygon(const Ray& ray, std::span<const Vec2> poly, double tMin) {
        Hit out;
        const std::vector<PolygonHit> hits = rayPolygonAll(ray, poly, tMin);
        if (hits.empty()) return out;
        out.hit = true;
        out.t = hits.front().t;
        out.point = hits.front().point;
        out.normal = hits.front().normal;
        return out;
    }

    // ------------------------------------------------------------------ convex regions (CSG)

    std::optional<LineSpan> lineDisc(const Ray& ray, Vec2 center, double radius) {
        const Vec2 d = unitDir(ray);
        if ((d.x == 0.0 && d.y == 0.0) || !(radius > 0.0)) return std::nullopt;

        // |o + t d - c|^2 = r^2 with |d| = 1:  t^2 + 2 h t + k = 0
        const Vec2 oc = ray.origin - center;
        const double h = d.dot(oc);
        const double k = oc.dot(oc) - radius * radius;
        const double disc = h * h - k;
        if (disc <= 0.0) return std::nullopt;       // misses, or only touches (no interior)

        // numerically stable roots: avoid subtracting nearly equal numbers
        const double s = std::sqrt(disc);
        const double qq = -h - (h >= 0.0 ? s : -s);
        double t0 = qq, t1 = qq != 0.0 ? k / qq : -qq;
        if (t0 > t1) std::swap(t0, t1);

        LineSpan span;
        span.tIn = t0;
        span.tOut = t1;
        span.nIn = (ray.origin + d * t0 - center) / radius;
        span.nOut = (ray.origin + d * t1 - center) / radius;
        return span;
    }

    std::optional<LineSpan> lineHalfPlane(const Ray& ray, Vec2 point, Vec2 outward) {
        const Vec2 d = unitDir(ray);
        const Vec2 n = outward.normalized();
        if ((d.x == 0.0 && d.y == 0.0) || (n.x == 0.0 && n.y == 0.0)) return std::nullopt;

        // signed height above the boundary line: s(t) = (o - p) . n + t (d . n), inside where s <= 0
        const double s0 = (ray.origin - point).dot(n);
        const double rate = d.dot(n);

        LineSpan span;
        span.nIn = n;
        span.nOut = n;
        if (rate == 0.0) {
            if (s0 > 0.0) return std::nullopt;      // parallel and outside
            span.tIn = -kInf;
            span.tOut = kInf;
            return span;
        }
        const double tCross = -s0 / rate;
        if (rate > 0.0) { span.tIn = -kInf; span.tOut = tCross; }     // heading out
        else            { span.tIn = tCross; span.tOut = kInf; }      // heading in
        return span;
    }

    std::optional<LineSpan> intersectSpans(const LineSpan& a, const LineSpan& b) {
        LineSpan out;
        if (a.tIn >= b.tIn) { out.tIn = a.tIn; out.nIn = a.nIn; } else { out.tIn = b.tIn; out.nIn = b.nIn; }
        if (a.tOut <= b.tOut) { out.tOut = a.tOut; out.nOut = a.nOut; } else { out.tOut = b.tOut; out.nOut = b.nOut; }
        if (!(out.tIn < out.tOut)) return std::nullopt;
        return out;
    }

    Hit firstHit(const Ray& ray, const LineSpan& span, double tMin) {
        Hit out;
        const Vec2 d = unitDir(ray);
        double t;
        Vec2 n;
        if (span.tIn >= tMin && std::isfinite(span.tIn)) { t = span.tIn; n = span.nIn; }
        else if (span.tOut >= tMin && std::isfinite(span.tOut)) { t = span.tOut; n = span.nOut; }
        else return out;

        out.hit = true;
        out.t = t;
        out.point = ray.origin + d * t;
        out.normal = n.normalized();
        return out;
    }
}
