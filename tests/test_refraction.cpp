#include <cmath>
#include <cstdio>

#include "optibench/Mat3.h"
#include "optibench/Transform2D.h"
#include "optibench/refraction.h"

#define CHECK(cond) do { if (!(cond)) { std::printf("FAIL %s:%d  %s\n", __FILE__, __LINE__, #cond); return 1; } } while (0)

using namespace ob;

static bool near(double a, double b, double eps = 1e-9) { return std::fabs(a - b) < eps; }
static bool near(Vec2 a, Vec2 b, double eps = 1e-9) { return near(a.x, b.x, eps) && near(a.y, b.y, eps); }

static double deg(double d) { return d * kPi / 180.0; }
static double toDeg(double r) { return r * 180.0 / kPi; }

// unit direction heading down (+y) at `angle` from the vertical: hits a horizontal surface at that incidence
static Vec2 downAt(double angle) { return {std::sin(angle), std::cos(angle)}; }

int main() {
    // horizontal interface y = 0; rays arrive from above (y < 0) travelling +y
    const Segment surface{{-100, 0}, {100, 0}};
    const Vec2 up{0, -1};                                           // unit normal facing the incoming rays

    // ---------- critical angle: sin(theta_c) = n2 / n1 ----------
    {
        CHECK(near(toDeg(*criticalAngle(2.0, 1.0)), 30.0));         // sin = 1/2
        CHECK(near(toDeg(*criticalAngle(1.5, 1.0)), 41.8103149, 1e-6)); // sin = 2/3
        CHECK(!criticalAngle(1.0, 1.5).has_value());                // into a denser medium: never TIR
        CHECK(!criticalAngle(1.5, 1.5).has_value());                // same medium: never TIR
    }

    // ---------- air -> glass (n = 1.5) at 30 degrees ----------
    {
        // sin(t2) = sin(30) / 1.5 = 1/3,  cos(t2) = sqrt(8)/3
        const Refraction r = refractOrReflect(downAt(deg(30)), up, 1.0, 1.5);
        CHECK(!r.totalInternal);
        CHECK(near(r.direction, {1.0 / 3.0, std::sqrt(8.0) / 3.0}));
        CHECK(near(toDeg(r.incidence), 30.0));
        CHECK(near(toDeg(r.exitAngle), 19.4712206, 1e-6));
        CHECK(r.exitAngle < r.incidence);                           // bends towards the normal
    }

    // ---------- glass -> air at 30 degrees (below the 41.8 degree critical angle) ----------
    {
        // sin(t2) = 1.5 * sin(30) = 0.75,  cos(t2) = sqrt(1 - 0.5625) = sqrt(0.4375)
        const Refraction r = refractOrReflect(downAt(deg(30)), up, 1.5, 1.0);
        CHECK(!r.totalInternal);
        CHECK(near(r.direction, {0.75, std::sqrt(0.4375)}));
        CHECK(near(toDeg(r.exitAngle), 48.5903779, 1e-6));
        CHECK(r.exitAngle > r.incidence);                           // bends away from the normal
    }

    // ---------- total internal reflection: glass -> air at 60 degrees ----------
    {
        // 60 > 41.8: no refracted ray; the law of reflection flips the normal component
        const Refraction r = refractOrReflect(downAt(deg(60)), up, 1.5, 1.0);
        CHECK(r.totalInternal);
        CHECK(near(r.direction, {std::sin(deg(60)), -std::cos(deg(60))}));
        CHECK(near(r.exitAngle, r.incidence));                      // angle in = angle out
        CHECK(near(r.direction.length(), 1.0));
    }

    // ---------- TIR edge: either side of the critical angle ----------
    {
        // n = 2: critical angle is exactly 30 degrees
        const Refraction below = refractOrReflect(downAt(deg(29.9)), up, 2.0, 1.0);
        const Refraction above = refractOrReflect(downAt(deg(30.1)), up, 2.0, 1.0);
        CHECK(!below.totalInternal);
        CHECK(above.totalInternal);
        // just below: sin(t2) = 2 sin(29.9) = 0.9969755, so the ray leaves almost along the surface
        CHECK(near(toDeg(below.exitAngle), 85.5426635, 1e-6));

        // n = 1.5, a hair either side of asin(2/3)
        const double tc = *criticalAngle(1.5, 1.0);
        const Refraction grazing = refractOrReflect(downAt(tc - 1e-4), up, 1.5, 1.0);
        CHECK(!grazing.totalInternal && toDeg(grazing.exitAngle) > 89.0);
        CHECK(refractOrReflect(downAt(tc + 1e-4), up, 1.5, 1.0).totalInternal);

        // exactly at the critical angle: grazes along the surface (or reflects, depending on rounding)
        const Refraction at = refractOrReflect(downAt(deg(30)), up, 2.0, 1.0);
        CHECK(at.totalInternal || near(toDeg(at.exitAngle), 90.0, 1e-3));
        CHECK(std::isfinite(at.direction.x) && std::isfinite(at.direction.y));
    }

    // ---------- Snell's law holds across a sweep, and TIR starts exactly past the critical angle ----------
    {
        const double tc = *criticalAngle(1.5, 1.0);
        for (int a = 0; a <= 85; a += 5) {
            const double t1 = deg(a);

            const Refraction in = refractOrReflect(downAt(t1), up, 1.0, 1.5);
            CHECK(!in.totalInternal);
            CHECK(near(1.0 * std::sin(in.incidence), 1.5 * std::sin(in.exitAngle)));

            const Refraction out = refractOrReflect(downAt(t1), up, 1.5, 1.0);
            CHECK(out.totalInternal == (t1 > tc));
            if (!out.totalInternal) CHECK(near(1.5 * std::sin(out.incidence), 1.0 * std::sin(out.exitAngle)));
        }
    }

    // ---------- special cases ----------
    {
        // straight in along the normal: no bending
        const Refraction head = refractOrReflect({0, 1}, up, 1.0, 1.5);
        CHECK(near(head.direction, {0, 1}) && near(head.exitAngle, 0.0));

        // equal indices: no bending at any angle
        CHECK(near(refractOrReflect(downAt(deg(37)), up, 1.33, 1.33).direction, downAt(deg(37))));

        // the normal may be given facing either way, and the direction need not be unit length
        const Refraction ref = refractOrReflect(downAt(deg(30)), up, 1.0, 1.5);
        CHECK(near(refractOrReflect(downAt(deg(30)), -up, 1.0, 1.5).direction, ref.direction));
        CHECK(near(refractOrReflect(downAt(deg(30)) * 7.0, up, 1.0, 1.5).direction, ref.direction));
    }

    // ---------- at a segment: built on intersectRaySegment ----------
    {
        // aimed to hit the origin after travelling 10
        const Ray ray{Vec2{0, 0} - downAt(deg(30)) * 10.0, downAt(deg(30))};
        const SurfaceInteraction s = refractAtSegment(ray, surface, 1.0, 1.5);
        CHECK(s.hit.hit && near(s.hit.t, 10.0) && near(s.hit.point, {0, 0}));
        CHECK(near(s.light.direction, {1.0 / 3.0, std::sqrt(8.0) / 3.0}));
        CHECK(near(s.outgoing.direction, s.light.direction));
        CHECK(near(s.outgoing.origin, {0, 0}, 1e-5) && s.outgoing.origin.y > 0.0);  // nudged into the glass

        // TIR at a segment is the same bounce Mat3::reflectLine gives across that line
        // (n = 2.5: critical angle asin(0.4) = 23.6 degrees, so 30 degrees is well past it)
        const SurfaceInteraction t = refractAtSegment(ray, surface, 2.5, 1.0);
        CHECK(t.light.totalInternal);
        CHECK(near(t.light.direction, Mat3::reflectLine(surface.a, surface.b).transformDirection(ray.direction)));
        CHECK(t.outgoing.origin.y < 0.0);                           // stays on the arrival side

        // misses: running parallel, or pointing away
        const Ray parallel{{-5, -1}, {1, 0}};
        const SurfaceInteraction p = refractAtSegment(parallel, surface, 1.0, 1.5);
        CHECK(!p.hit.hit && near(p.outgoing.origin, parallel.origin) && near(p.outgoing.direction, parallel.direction));
        CHECK(!refractAtSegment(Ray{{0, -5}, {0, -1}}, surface, 1.0, 1.5).hit.hit);
    }

    // ---------- tilted interface: the 45 degree line y = x ----------
    {
        // horizontal ray meets y = x at 45 degrees incidence; sin(t2) = sin(45) / 1.5 -> t2 = 28.1255057
        const SurfaceInteraction s = refractAtSegment(Ray{{0, 5}, {1, 0}}, Segment{{0, 0}, {10, 10}}, 1.0, 1.5);
        CHECK(s.hit.hit && near(s.hit.point, {5, 5}));
        CHECK(near(toDeg(s.light.incidence), 45.0));
        CHECK(near(toDeg(s.light.exitAngle), 28.1255057, 1e-6));
        CHECK(s.light.direction.dot(s.hit.normal) < 0.0);          // crossed to the far side
    }

    // ---------- glass slab: in and out through parallel faces ----------
    {
        // slab between y = 0 and y = 5, n = 1.5, entered at 30 degrees through the origin
        const Ray ray{Vec2{0, 0} - downAt(deg(30)) * 10.0, downAt(deg(30))};
        const SurfaceInteraction in = refractAtSegment(ray, surface, 1.0, 1.5);
        const SurfaceInteraction out = refractAtSegment(in.outgoing, Segment{{-100, 5}, {100, 5}}, 1.5, 1.0);
        CHECK(out.hit.hit && !out.light.totalInternal);

        // inside it drifts 5 tan(t2) = 5 (1/3) / (sqrt8 / 3) = 5 / sqrt8 sideways, then leaves parallel to how it came
        CHECK(near(out.hit.point, {5.0 / std::sqrt(8.0), 5.0}, 1e-5));
        CHECK(near(out.light.direction, ray.direction));
    }

    // ---------- light pipe: repeated TIR keeps the ray inside ----------
    {
        // glass (n = 1.5) between y = 0 and y = 10; a ray inside at 60 degrees from the normal (> 41.8)
        const Segment walls[] = {{{-1000, 0}, {1000, 0}}, {{-1000, 10}, {1000, 10}}};
        Ray ray{{0, 5}, downAt(deg(60))};

        // first bounce at x = 5 tan(60), then every 10 tan(60) further along
        const double expectX[] = {5.0 * std::sqrt(3.0), 15.0 * std::sqrt(3.0), 25.0 * std::sqrt(3.0), 35.0 * std::sqrt(3.0)};
        for (const double x : expectX) {
            // nearest wall the ray reaches
            SurfaceInteraction best;
            for (const Segment& w : walls) {
                const SurfaceInteraction s = refractAtSegment(ray, w, 1.5, 1.0);
                if (s.hit.hit && (!best.hit.hit || s.hit.t < best.hit.t)) best = s;
            }
            CHECK(best.hit.hit && best.light.totalInternal);
            CHECK(near(best.hit.point.x, x, 1e-5));
            CHECK(best.outgoing.origin.y > 0.0 && best.outgoing.origin.y < 10.0);   // still inside
            ray = best.outgoing;
        }
    }

    std::puts("test_refraction: PASS");
    return 0;
}
