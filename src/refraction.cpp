#include "optibench/refraction.h"

#include <algorithm>
#include <cmath>

#include "optibench/optics.h"

namespace ob {
    namespace {
        // pushes a new ray off the surface it just left (same value the tracer uses)
        constexpr double kNudge = 1e-6;

        // angle in radians whose cosine is c, safe against rounding just outside [-1, 1]
        double angleFromCos(double c) { return std::acos(std::clamp(c, -1.0, 1.0)); }
    }

    // logic finding the critical angle: sin(theta_c) = n2 / n1, which only exists when n1 > n2
    std::optional<double> criticalAngle(double n1, double n2) {
        if (n1 <= n2 || n1 <= 0.0) return std::nullopt;
        return std::asin(n2 / n1);
    }

    // logic bending a ray at a boundary, falling back to reflection on total internal reflection
    Refraction refractOrReflect(const Vec2& d, const Vec2& n, double n1, double n2) {
        Refraction out;
        const Vec2 dir = d.normalized();

        // refract() and reflect() both expect the normal to face against the ray (d.n < 0)
        const Vec2 facing = dir.dot(n) > 0.0 ? -n : n;
        out.incidence = angleFromCos(-dir.dot(facing));

        Vec2 bent;
        if (refract(dir, facing, n1, n2, bent)) {
            out.direction = bent;
            out.exitAngle = angleFromCos(bent.dot(-facing));
        } else {
            // past the critical angle: every bit of light stays on this side
            out.totalInternal = true;
            out.direction = reflect(dir, facing);
            out.exitAngle = out.incidence;
        }
        return out;
    }

    // logic tracing one ray across one flat interface
    SurfaceInteraction refractAtSegment(const Ray& ray, const Segment& boundary, double n1, double n2) {
        SurfaceInteraction out;
        out.hit = intersectRaySegment(ray, boundary);

        // nothing hit: the ray carries on unchanged
        if (!out.hit.hit) {
            out.outgoing = ray;
            return out;
        }

        out.light = refractOrReflect(ray.direction, out.hit.normal, n1, n2);
        out.outgoing = Ray{out.hit.point + out.light.direction * kNudge, out.light.direction};
        return out;
    }
}
