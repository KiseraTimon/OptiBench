// Lens shape: rim, validation, containment, ray hits, outline, and focal lengths checked by
// actually tracing paraxial rays through it with Snell's law (optics.h).
#include <cmath>
#include <limits>
#include <stdexcept>
#include <vector>

#include "../curves/TestHarness.h"
#include "optibench/geometry/Lens.h"
#include "optibench/geometry/PolygonOps.h"
#include "optibench/optics.h"

using namespace ob;
using namespace ob::geom;

namespace {
    constexpr double kFlat = Lens::kFlat;
    constexpr double kNaN = std::numeric_limits<double>::quiet_NaN();

    // area of the circular segment of radius r and sagitta s
    double segmentArea(double r, double s) {
        return r * r * std::acos((r - s) / r) - (r - s) * std::sqrt(2 * r * s - s * s);
    }

    bool throwsInvalid(double r1, double r2, double d, double ap = kFlat) {
        try { Lens l(r1, r2, d, ap); (void)l; } catch (const std::invalid_argument&) { return true; }
        return false;
    }

    /**
     * fires a ray parallel to the axis at height y, refracts it in and out of the lens, and returns
     * where the outgoing ray crosses the axis (NaN if something went wrong)
     */
    double axisCrossing(const Lens& lens, double y, double n) {
        Ray ray{{-1000.0, y}, {1.0, 0.0}};
        const Hit in = lens.intersect(ray);
        if (!in.hit) return kNaN;
        Vec2 bent;
        if (!refract(ray.direction, in.normal, 1.0, n, bent)) return kNaN;          // outward normal opposes the ray

        const Hit out = lens.intersect(Ray{in.point, bent});
        if (!out.hit) return kNaN;
        Vec2 exitDir;
        if (!refract(bent, -out.normal, n, 1.0, exitDir)) return kNaN;              // facing normal is inward
        if (exitDir.y == 0.0) return kNaN;
        return out.point.x - out.point.y * exitDir.x / exitDir.y;
    }

    void testRim(test::Context& ctx) {
        // symmetric biconvex: centres at x = +-90, faces meet on the y axis
        const Lens bi(100, 100, 20);
        CHECK_NEAR(bi.halfHeight(), std::sqrt(1900.0), 1e-12);
        CHECK_NEAR(bi.edgeThickness(), 0.0, 1e-9);
        CHECK_VEC_NEAR(bi.bounds().min, Vec2(-10, -std::sqrt(1900.0)), 1e-12);
        CHECK_VEC_NEAR(bi.bounds().max, Vec2(10, std::sqrt(1900.0)), 1e-12);

        // plano-convex: sqrt(d (2R - d)) = sqrt(10 * 90) = 30
        const Lens pc(50, kFlat, 10);
        CHECK_NEAR(pc.halfHeight(), 30.0, 1e-12);
        CHECK_NEAR(pc.edgeThickness(), 0.0, 1e-9);
        CHECK_NEAR(Lens(kFlat, 50, 10).halfHeight(), 30.0, 1e-12);

        // unequal radii: rim off-centre, still on both circles
        const Lens un(60, 120, 16);
        const double h = un.halfHeight();
        CHECK(h > 0.0);
        CHECK(un.contains({0, 0}));

        // an aperture cuts the rim flat: edge thickness = 2 (sqrt(R^2 - a^2) - 90)
        const Lens cut(100, 100, 20, 30);
        CHECK_NEAR(cut.halfHeight(), 30.0, 0.0);
        CHECK_NEAR(cut.edgeThickness(), 2 * (std::sqrt(10000.0 - 900.0) - 90.0), 1e-12);

        // an aperture bigger than the lens changes nothing
        CHECK_NEAR(Lens(100, 100, 20, 500).halfHeight(), std::sqrt(1900.0), 1e-12);

        // two flat faces + aperture: a rectangular slab
        const Lens slab(kFlat, kFlat, 10, 20);
        CHECK_NEAR(slab.halfHeight(), 20.0, 0.0);
        CHECK_NEAR(slab.edgeThickness(), 10.0, 0.0);
    }

    void testValidation(test::Context& ctx) {
        CHECK(throwsInvalid(-5, 100, 10));
        CHECK(throwsInvalid(100, 0, 10));
        CHECK(throwsInvalid(kNaN, 100, 10));
        CHECK(throwsInvalid(-kFlat, 100, 10));
        CHECK(throwsInvalid(100, 100, 0));
        CHECK(throwsInvalid(100, 100, -3));
        CHECK(throwsInvalid(100, 100, kFlat));
        CHECK(throwsInvalid(100, 100, 10, 0));
        CHECK(throwsInvalid(kFlat, kFlat, 10));             // unbounded slab
        CHECK(throwsInvalid(50, kFlat, 60));                // more than a hemisphere (d > R)
        CHECK(throwsInvalid(10, 10, 25));                   // circles do not even overlap
        CHECK(throwsInvalid(100, 30, 50));                  // back face past a hemisphere
        CHECK(!throwsInvalid(50, kFlat, 50));               // exactly a hemisphere is fine
        CHECK(!throwsInvalid(100, 100, 20));
    }

    void testContains(test::Context& ctx) {
        const Lens bi(100, 100, 20);
        CHECK(bi.contains({0, 0}));
        CHECK(bi.contains({-10, 0}));                       // front vertex
        CHECK(bi.contains({10, 0}));                        // back vertex
        CHECK(!bi.contains({-10.01, 0}));
        CHECK(!bi.contains({0, 44}));                       // above the rim
        CHECK(bi.contains({0, 43.5}));
        CHECK(!bi.contains({9, 20}));                       // outside the back face near the rim

        const Lens slab(kFlat, kFlat, 10, 20);
        CHECK(slab.contains({4.9, 19.9}));
        CHECK(!slab.contains({4.9, 20.1}));
    }

    void testIntersect(test::Context& ctx) {
        const Lens bi(100, 100, 20);

        Hit h = bi.intersect({{-200, 0}, {1, 0}});
        CHECK(h.hit);
        CHECK_NEAR(h.t, 190.0, 1e-9);
        CHECK_VEC_NEAR(h.point, Vec2(-10, 0), 1e-9);
        CHECK_VEC_NEAR(h.normal, Vec2(-1, 0), 1e-12);

        // from inside: the exit, normal still outward
        h = bi.intersect({{0, 0}, {1, 0}});
        CHECK(h.hit);
        CHECK_VEC_NEAR(h.point, Vec2(10, 0), 1e-9);
        CHECK_VEC_NEAR(h.normal, Vec2(1, 0), 1e-12);

        // off axis: the point lies on the front circle and the normal is radial
        h = bi.intersect({{-200, 20}, {1, 0}});
        CHECK(h.hit);
        const Vec2 c1{90, 0};
        CHECK_NEAR((h.point - c1).length(), 100.0, 1e-9);
        CHECK_NEAR(h.point.x, 90 - std::sqrt(9600.0), 1e-9);
        CHECK_VEC_NEAR(h.normal, (h.point - c1) / 100.0, 1e-12);

        CHECK(!bi.intersect({{-200, 50}, {1, 0}}).hit);      // passes above
        CHECK(!bi.intersect({{-200, 0}, {-1, 0}}).hit);      // pointing away
        CHECK(!bi.intersect({{20, 0}, {1, 0}}).hit);         // already past it

        // the trimmed rim of a slab is a real surface
        const Lens slab(kFlat, kFlat, 10, 20);
        h = slab.intersect({{0, -50}, {0, 1}});
        CHECK(h.hit);
        CHECK_VEC_NEAR(h.point, Vec2(0, -20), 1e-12);
        CHECK_VEC_NEAR(h.normal, Vec2(0, -1), 1e-12);

        // and so is the flat edge of a cut biconvex lens
        const Lens cut(100, 100, 20, 30);
        h = cut.intersect({{0, 100}, {0, -1}});
        CHECK(h.hit);
        CHECK_VEC_NEAR(h.point, Vec2(0, 30), 1e-12);
        CHECK_VEC_NEAR(h.normal, Vec2(0, 1), 1e-12);

        // every hit point lies on the boundary: inside within tol, outside a little further out
        for (double y = -40; y <= 40; y += 5) {
            const Hit g = bi.intersect({{-200, y}, {1, 0.05}});
            if (!g.hit) continue;
            CHECK(bi.contains(g.point, 1e-9));
            CHECK(!bi.contains(g.point + g.normal * 1e-3));
        }
    }

    void testOutline(test::Context& ctx) {
        const Lens bi(100, 100, 20);
        const std::vector<Vec2> o = bi.outline(64);
        CHECK(winding(o) == Winding::CounterClockwise);
        CHECK(isSimple(o));
        CHECK(isConvex(o));
        for (const Vec2& p : o) CHECK(bi.contains(p, 1e-9));
        const double exact = 2 * segmentArea(100, 10);
        CHECK(std::fabs(area(o) - exact) / exact < 1e-3);
        CHECK_VEC_NEAR(boundsOf(o).min, bi.bounds().min, 1e-9);
        CHECK_VEC_NEAR(boundsOf(o).max, bi.bounds().max, 1e-9);

        // plano-convex and cut lenses keep their flat sides as single straight edges
        const Lens pc(50, kFlat, 10);
        const std::vector<Vec2> po = pc.outline(32);
        CHECK(po.size() == 33);
        CHECK(isConvex(po));
        CHECK(std::fabs(area(po) - segmentArea(50, 10)) / segmentArea(50, 10) < 2e-3);

        const Lens cut(100, 100, 20, 30);
        const std::vector<Vec2> co = cut.outline(16);
        CHECK(co.size() == 34);         // two arcs of 17 points, joined by the flat top and bottom edges
        CHECK(isConvex(co));

        const Lens slab(kFlat, kFlat, 10, 20);
        CHECK_NEAR(area(slab.outline()), 10.0 * 40.0, 1e-9);

        CHECK(bi.outline(0).size() == 4);       // clamped to 2 segments per face: 3 + 3 points, both rims shared
    }

    void testFocalLengths(test::Context& ctx) {
        const double n = 1.5;

        // symmetric biconvex: 1/f = 0.5 (0.02 - 0.5 * 20 / (1.5 * 1e4))
        const Lens bi(100, 100, 20);
        const double f = 1.0 / (0.5 * (0.02 - 0.5 * 20.0 / (1.5 * 1e4)));
        CHECK_NEAR(bi.focalLength(n), f, 1e-9);
        CHECK_NEAR(bi.backFocalDistance(n), f * (1.0 - 0.5 * 20.0 * 0.01 / 1.5), 1e-9);
        CHECK_NEAR(bi.frontFocalDistance(n), bi.backFocalDistance(n), 1e-12);

        // plano-convex: f = R / (n - 1) whichever way round; the back focal distance differs
        const Lens pc(50, kFlat, 10), cp(kFlat, 50, 10);
        CHECK_NEAR(pc.focalLength(n), 100.0, 1e-9);
        CHECK_NEAR(cp.focalLength(n), 100.0, 1e-9);
        CHECK_NEAR(pc.backFocalDistance(n), 100.0 * (1.0 - 0.5 * 10.0 / (1.5 * 50.0)), 1e-9);
        CHECK_NEAR(cp.backFocalDistance(n), 100.0, 1e-9);
        CHECK_NEAR(cp.frontFocalDistance(n), pc.backFocalDistance(n), 1e-9);

        // a slab does not focus
        CHECK(std::isinf(Lens(kFlat, kFlat, 10, 20).focalLength(n)));

        // the real test: paraxial rays traced with Snell's law cross the axis at the predicted focus
        for (const Lens* lens : {&bi, &pc, &cp}) {
            const double focus = lens->thickness() / 2.0 + lens->backFocalDistance(n);
            for (const double y : {0.25, -0.25, 0.5}) {
                CHECK_NEAR(axisCrossing(*lens, y, n), focus, 0.02);
            }
            // a marginal ray focuses shorter (spherical aberration), as it should
            CHECK(axisCrossing(*lens, 0.6 * lens->halfHeight(), n) < focus);
        }

        // a denser glass focuses shorter
        CHECK(axisCrossing(bi, 0.5, 1.7) < axisCrossing(bi, 0.5, 1.5));
    }
}

int main() {
    return test::runTests("geometry lens", {
        {"rim and bounds", testRim},
        {"validation", testValidation},
        {"containment", testContains},
        {"ray hits", testIntersect},
        {"outline", testOutline},
        {"focal lengths (traced)", testFocalLengths},
    });
}
