#include "optibench/curves/OpticalCurve.h"

#include <stdexcept>

#include "optibench/clip_fill.h"

namespace ob::curves {
    namespace {
        // |cross(tangent, d)| below this marks a grazing hit
        constexpr double kTangentialSine = 1e-7;

        // hits closer than this (relative to the scene size) are the same hit
        constexpr double kDuplicateRelative = 1e-9;

        // an irrational-ish direction for inside tests, unlikely to graze a vertex
        const Vec2 kInsideProbe = Vec2(0.8191520443, 0.5735764364);

        bool closerFirst(const CurveHit& a, const CurveHit& b) { return a.distance < b.distance; }
    }

    std::vector<CurveHit> intersectRay(const Ray& ray, const Bezier& curve, const RayCurveOptions& options,
                                       double normalSign) {
        std::vector<CurveHit> hits;
        const Vec2 d = ray.direction.normalized();
        if (d.length() == 0.0 || curve.degree() < 1) return hits;

        // signed distances of the control points to the ray's line
        std::vector<double> coeffs;
        coeffs.reserve(curve.controlPoints().size());
        double reach = 1.0;
        for (const Vec2& p : curve.controlPoints()) {
            coeffs.push_back(d.cross(p - ray.origin));
            reach = std::max(reach, (p - ray.origin).length());
        }
        const double zeroTol = 1e-10 * reach;

        for (double t : bernsteinRoots(coeffs, zeroTol)) {
            CurveHit h;
            h.t = t;
            h.u = t;
            h.point = curve.evaluate(t);
            h.distance = d.dot(h.point - ray.origin);
            if (h.distance < options.minDistance || h.distance > options.maxDistance) continue;

            const Vec2 tangent = curve.tangent(t);
            h.tangential = std::fabs(tangent.cross(d)) < kTangentialSine;
            if (h.tangential && !options.includeTangential) continue;

            h.surfaceNormal = tangent.perp() * normalSign;
            h.frontFace = h.surfaceNormal.dot(d) < 0.0;
            h.normal = h.frontFace ? h.surfaceNormal : -h.surfaceNormal;
            if (h.tangential) {
                // grazing: dot(n, d) ~ 0, so the sign test above is meaningless. A grazing ray lies
                // on the convex side (the curve bends away from it), so face away from the centre of
                // curvature; at an inflection fall back to the side the ray origin is on.
                const double k = curve.curvature(t) * normalSign; // > 0: centre on the surfaceNormal side
                const double side = k != 0.0 ? -k : h.surfaceNormal.dot(ray.origin - h.point);
                h.frontFace = side >= 0.0;
                h.normal = h.frontFace ? h.surfaceNormal : -h.surfaceNormal;
            }
            hits.push_back(h);
        }
        std::sort(hits.begin(), hits.end(), closerFirst);
        return hits;
    }

    std::vector<CurveHit> intersectRay(const Ray& ray, const CompositeBezier& curve, const RayCurveOptions& options,
                                       double normalSign) {
        std::vector<CurveHit> all;
        for (std::size_t i = 0; i < curve.segmentCount(); ++i) {
            for (CurveHit h : intersectRay(ray, curve.segment(i), options, normalSign)) {
                h.segment = i;
                h.u = static_cast<double>(i) + h.t;
                all.push_back(h);
            }
        }
        std::sort(all.begin(), all.end(), closerFirst);

        // a ray through a join point is found by both neighbouring segments: keep one
        const Vec2 size = curve.controlBounds().size();
        const double tol = kDuplicateRelative * std::max({1.0, size.x, size.y});
        std::vector<CurveHit> unique;
        for (const CurveHit& h : all)
            if (unique.empty() || std::fabs(h.distance - unique.back().distance) > tol ||
                (h.point - unique.back().point).length() > tol)
                unique.push_back(h);
        return unique;
    }

    Hit toHit(const std::vector<CurveHit>& hits) {
        Hit h;
        if (hits.empty()) return h;
        h.hit = true;
        h.t = hits.front().distance;
        h.point = hits.front().point;
        h.normal = hits.front().normal;
        return h;
    }

    OpticalSurface::OpticalSurface(CompositeBezier profile, SurfaceKind kind, double refractiveIndex)
        : profile_(std::move(profile)), kind_(kind), refractiveIndex_(refractiveIndex), closed_(profile_.isClosed()) {
        if (profile_.empty()) throw std::invalid_argument("OpticalSurface: empty profile");
        if (!std::isfinite(refractiveIndex_) || refractiveIndex_ <= 0.0)
            throw std::invalid_argument("OpticalSurface: refractive index must be positive");
        if (closed_) {
            // counter-clockwise (positive shoelace area, y-up) means perp() points inward: flip it
            const Polyline outline = profile_.flatten(1e-3 * std::max(1.0, profile_.controlBounds().size().length()));
            normalSign_ = polygonArea(outline) > 0.0 ? -1.0 : 1.0;
        }
    }

    std::vector<CurveHit> OpticalSurface::intersect(const Ray& ray, const RayCurveOptions& options) const {
        return intersectRay(ray, profile_, options, normalSign_);
    }

    std::optional<CurveHit> OpticalSurface::firstHit(const Ray& ray, const RayCurveOptions& options) const {
        const std::vector<CurveHit> hits = intersect(ray, options);
        if (hits.empty()) return std::nullopt;
        return hits.front();
    }

    bool OpticalSurface::contains(Vec2 p) const {
        if (!closed_) return false;
        RayCurveOptions opts;
        opts.minDistance = 0.0;
        opts.includeTangential = false;
        return intersect(Ray{p, kInsideProbe}, opts).size() % 2 == 1;
    }

    Vec2 OpticalSurface::surfaceNormalAt(double u) const { return profile_.normal(u) * normalSign_; }

    Bezier parabolicProfile(Vec2 vertex, Vec2 axis, double focalLength, double halfAperture) {
        if (!(focalLength > 0.0) || !(halfAperture > 0.0))
            throw std::invalid_argument("parabolicProfile: focal length and aperture must be positive");
        const Vec2 ax = axis.normalized();
        if (ax.length() == 0.0) throw std::invalid_argument("parabolicProfile: zero axis");
        const Vec2 across = ax.perp();
        // in the axis frame: P0 = (a^2/4f, -a), P1 = (-a^2/4f, 0), P2 = (a^2/4f, a)
        const double depth = halfAperture * halfAperture / (4.0 * focalLength);
        auto frame = [&](double x, double y) { return vertex + ax * x + across * y; };
        return Bezier({frame(depth, -halfAperture), frame(-depth, 0.0), frame(depth, halfAperture)});
    }

    CompositeBezier lensProfile(const LensSpec& spec) {
        if (!(spec.aperture > 0.0) || !(spec.edgeThickness >= 0.0))
            throw std::invalid_argument("lensProfile: aperture must be positive and edge thickness >= 0");
        const double h = spec.aperture / 2.0;
        const double e = spec.edgeThickness / 2.0;
        const Vec2 c = spec.center;
        const Vec2 topLeft{c.x - e, c.y - h}, bottomLeft{c.x - e, c.y + h};
        const Vec2 topRight{c.x + e, c.y - h}, bottomRight{c.x + e, c.y + h};
        if (spec.edgeThickness == 0.0 && c.x - e - spec.frontSag >= c.x + e + spec.backSag)
            throw std::invalid_argument("lensProfile: faces cross at the centre");

        // a quadratic's midpoint is (P0 + 2 P1 + P2) / 4, so the control point that puts the
        // face's vertex at x = v is 2 v - x(edge)
        const double frontVertex = c.x - e - spec.frontSag;
        const double backVertex = c.x + e + spec.backSag;
        const Bezier front({topLeft, {2.0 * frontVertex - topLeft.x, c.y}, bottomLeft});
        const Bezier back({bottomRight, {2.0 * backVertex - bottomRight.x, c.y}, topRight});

        CompositeBezier lens;
        lens.append(front);
        if (e > 0.0) lens.append(Bezier::line(bottomLeft, bottomRight));
        lens.append(back);
        if (e > 0.0) lens.append(Bezier::line(topRight, topLeft));
        return lens;
    }
}
