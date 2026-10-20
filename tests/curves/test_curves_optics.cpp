#include <cmath>
#include <cstdint>
#include <stdexcept>
#include <vector>

#include "TestHarness.h"
#include "optibench/curves/OpticalCurve.h"
#include "optibench/curves/RayPath.h"

using namespace ob;
using namespace ob::curves;

namespace {
    // y = x^2 on [-1, 1] as a quadratic Bezier
    Bezier parabola() { return Bezier({{-1, 1}, {0, -1}, {1, 1}}); }

    // the reflection formula the physics module uses; here only to check normals
    Vec2 reflect(Vec2 d, Vec2 n) { return d - n * (2.0 * d.dot(n)); }

    // every hit's normal is unit length and faces the incoming ray
    bool normalsFaceRay(const std::vector<CurveHit>& hits, Vec2 dir) {
        for (const CurveHit& h : hits)
            if (std::fabs(h.normal.length() - 1.0) > 1e-12 || h.normal.dot(dir.normalized()) > 1e-12) return false;
        return true;
    }

    void testParabolaAnalytic(test::Context& ctx) {
        // vertical ray x = 0.3 coming down from y = 5 hits y = 0.09 at distance 4.91
        const Ray down{{0.3, 5}, {0, -1}};
        const auto hits = intersectRay(down, parabola());
        CHECK(hits.size() == 1);
        if (hits.size() == 1) {
            CHECK_NEAR(hits[0].distance, 5 - 0.09, 1e-12);
            CHECK_VEC_NEAR(hits[0].point, Vec2(0.3, 0.09), 1e-12);
            CHECK_NEAR(hits[0].t, (0.3 + 1) / 2, 1e-12);
            // the parabola's normal at x is (-2x, 1)/|.|; it faces the downward ray as-is
            CHECK_VEC_NEAR(hits[0].normal, Vec2(-0.6, 1).normalized(), 1e-12);
            CHECK(!hits[0].tangential);
        }
        CHECK(normalsFaceRay(hits, down.direction));

        // horizontal ray y = 0.25 from x = -2: two hits at x = -0.5 and 0.5, sorted by distance
        const Ray across{{-2, 0.25}, {3, 0}}; // direction need not be unit length
        const auto two = intersectRay(across, parabola());
        CHECK(two.size() == 2);
        if (two.size() == 2) {
            CHECK_NEAR(two[0].distance, 1.5, 1e-12);
            CHECK_NEAR(two[1].distance, 2.5, 1e-12);
            CHECK_VEC_NEAR(two[0].point, Vec2(-0.5, 0.25), 1e-12);
            CHECK(two[0].frontFace != two[1].frontFace); // enters one side, leaves through the other
        }
        CHECK(normalsFaceRay(two, across.direction));
    }

    void testTangentAndMissingRays(test::Context& ctx) {
        // y = 0 touches the vertex: one tangential hit
        const auto touch = intersectRay(Ray{{-3, 0}, {1, 0}}, parabola());
        CHECK(touch.size() == 1);
        if (touch.size() == 1) {
            CHECK(touch[0].tangential);
            CHECK_VEC_NEAR(touch[0].point, Vec2(0, 0), 1e-6);
            // the grazing ray lies on the convex side (below the bowl): the normal faces down
            CHECK_VEC_NEAR(touch[0].normal, Vec2(0, -1), 1e-6);
        }
        RayCurveOptions noGraze;
        noGraze.includeTangential = false;
        CHECK(intersectRay(Ray{{-3, 0}, {1, 0}}, parabola(), noGraze).empty());

        // misses: below the vertex, pointing away, beyond maxDistance, zero direction
        CHECK(intersectRay(Ray{{-3, -0.5}, {1, 0}}, parabola()).empty());
        CHECK(intersectRay(Ray{{0.3, 5}, {0, 1}}, parabola()).empty());
        RayCurveOptions shortRay;
        shortRay.maxDistance = 4.0;
        CHECK(intersectRay(Ray{{0.3, 5}, {0, -1}}, parabola(), shortRay).empty());
        CHECK(intersectRay(Ray{{0.3, 5}, {0, 0}}, parabola()).empty());

        // a ray starting on the curve does not re-hit its own origin
        const auto fromSurface = intersectRay(Ray{{-0.5, 0.25}, {1, 0}}, parabola());
        CHECK(fromSurface.size() == 1);
        if (!fromSurface.empty()) CHECK_VEC_NEAR(fromSurface[0].point, Vec2(0.5, 0.25), 1e-12);
    }

    void testNearVerticalAndDegenerate(test::Context& ctx) {
        // nearly vertical ray
        const Ray steep{{0.5, 10}, {1e-9, -1}};
        const auto hits = intersectRay(steep, parabola());
        CHECK(hits.size() == 1);
        if (!hits.empty()) CHECK_NEAR(hits[0].point.y, hits[0].point.x * hits[0].point.x, 1e-12);

        // vertical straight segment hit by a horizontal ray
        const auto wall = intersectRay(Ray{{0, 3}, {1, 0}}, Bezier::line({5, 0}, {5, 10}));
        CHECK(wall.size() == 1);
        if (!wall.empty()) {
            CHECK_NEAR(wall[0].distance, 5.0, 1e-12);
            CHECK_VEC_NEAR(wall[0].normal, Vec2(-1, 0), 1e-12);
        }

        // ray running along a straight segment: infinitely many hits, reported as none
        CHECK(intersectRay(Ray{{-1, 0}, {1, 0}}, Bezier::line({0, 0}, {10, 0})).empty());

        // a curve collapsed to a point, and a degree-0 curve: no hits, no NaN
        CHECK(intersectRay(Ray{{0, 0}, {1, 0}}, Bezier({{5, 0}, {5, 0}, {5, 0}})).size() <= 1);
        CHECK(intersectRay(Ray{{0, 0}, {1, 0}}, Bezier({{5, 0}})).empty());

        // coincident control points (zero end derivative): the normal still comes out unit length
        const Bezier pinched({{0, -5}, {0, -5}, {4, 0}, {0, 5}});
        const auto p = intersectRay(Ray{{-10, -5}, {1, 0}}, pinched);
        CHECK(!p.empty());
        if (!p.empty()) CHECK_NEAR(p[0].normal.length(), 1.0, 1e-12);
    }

    void testAgainstBruteForce(test::Context& ctx) {
        // random rays against an S-shaped cubic: compare with dense sampling of the crossings
        const Bezier c({{0, 0}, {100, 150}, {200, -150}, {300, 0}});
        std::uint32_t seed = 12345;
        auto rnd = [&seed](double lo, double hi) {
            seed = seed * 1664525u + 1013904223u;
            return lo + (hi - lo) * (seed >> 8) / 16777216.0;
        };
        int compared = 0;
        for (int k = 0; k < 200; ++k) {
            const Ray ray{{rnd(-50, 350), rnd(-120, 120)}, {rnd(-1, 1), rnd(-1, 1)}};
            const Vec2 d = ray.direction.normalized();
            if (d.length() == 0.0) continue;
            // sign changes of the signed distance along 20000 samples, ahead of the origin
            int expected = 0;
            double prev = d.cross(c.evaluate(0) - ray.origin);
            for (int i = 1; i <= 20000; ++i) {
                const Vec2 p = c.evaluate(i / 20000.0);
                const double cur = d.cross(p - ray.origin);
                if ((cur > 0) != (prev > 0) && d.dot(p - ray.origin) > 1e-6) ++expected;
                prev = cur;
            }
            const auto hits = intersectRay(ray, c);
            CHECK(static_cast<int>(hits.size()) == expected);
            CHECK(normalsFaceRay(hits, ray.direction));
            for (const CurveHit& h : hits) {
                CHECK_NEAR(d.cross(h.point - ray.origin), 0.0, 1e-8); // on the ray
                CHECK_VEC_NEAR(h.point, ray.origin + d * h.distance, 1e-8);
            }
            ++compared;
        }
        CHECK(compared > 150);
    }

    void testCompositeJoins(test::Context& ctx) {
        // two segments sharing the point (1, 0): a ray through the join is reported once
        const CompositeBezier vee({Bezier({{0, 1}, {0.5, 0.5}, {1, 0}}), Bezier({{1, 0}, {1.5, 0.5}, {2, 1}})});
        const auto hits = intersectRay(Ray{{1, -2}, {0, 1}}, vee);
        CHECK(hits.size() == 1);
        if (!hits.empty()) CHECK_VEC_NEAR(hits[0].point, Vec2(1, 0), 1e-9);
        const auto two = intersectRay(Ray{{-1, 0.5}, {1, 0}}, vee);
        CHECK(two.size() == 2);
        if (two.size() == 2) CHECK(two[0].segment == 0 && two[1].segment == 1 && two[1].u > 1.0);

        CHECK(!toHit({}).hit);
        const Hit nearest = toHit(two);
        CHECK(nearest.hit);
        if (!two.empty()) CHECK_NEAR(nearest.t, two[0].distance, 0.0);
    }

    void testParabolicMirrorFocus(test::Context& ctx) {
        // rays parallel to the axis reflect through the focus: an exact test of the normals
        const Vec2 vertex{100, 200};
        const double f = 40.0;
        for (const Vec2 axis : {Vec2(1, 0), Vec2(0, -1), Vec2(1, 1).normalized()}) {
            const OpticalSurface mirror(CompositeBezier({parabolicProfile(vertex, axis, f, 60.0)}), SurfaceKind::Reflective);
            const Vec2 focus = vertex + axis * f;
            for (double offset : {-50.0, -20.0, 5.0, 33.0, 59.0}) {
                const Vec2 origin = vertex + axis * 300.0 + axis.perp() * offset;
                const auto hit = mirror.firstHit(Ray{origin, -axis});
                CHECK(hit.has_value());
                if (!hit) continue;
                const Vec2 r = reflect(-axis, hit->normal);
                // the reflected ray passes through the focus
                CHECK_NEAR(r.cross(focus - hit->point), 0.0, 1e-9 * (focus - hit->point).length());
                CHECK(r.dot(focus - hit->point) > 0.0);
            }
        }
    }

    void testLensSurface(test::Context& ctx) {
        LensSpec spec;
        spec.center = {200, 100};
        spec.aperture = 120;
        spec.edgeThickness = 10;
        spec.frontSag = 20;
        spec.backSag = 20;
        const OpticalSurface lens(lensProfile(spec), SurfaceKind::Refractive, 1.5);
        CHECK(lens.closed());
        CHECK_NEAR(lens.refractiveIndex(), 1.5, 0.0);

        // a ray along the optical axis: enters at the front vertex, leaves at the back vertex
        const auto hits = lens.intersect(Ray{{0, 100}, {1, 0}});
        CHECK(hits.size() == 2);
        if (hits.size() == 2) {
            CHECK_VEC_NEAR(hits[0].point, Vec2(200 - 5 - 20, 100), 1e-9);
            CHECK_VEC_NEAR(hits[1].point, Vec2(200 + 5 + 20, 100), 1e-9);
            CHECK(hits[0].frontFace && !hits[1].frontFace);
            // outward surface normals: left face points -x, right face +x
            CHECK_VEC_NEAR(hits[0].surfaceNormal, Vec2(-1, 0), 1e-9);
            CHECK_VEC_NEAR(hits[1].surfaceNormal, Vec2(1, 0), 1e-9);
            // facing normals both oppose the ray
            CHECK_VEC_NEAR(hits[0].normal, Vec2(-1, 0), 1e-9);
            CHECK_VEC_NEAR(hits[1].normal, Vec2(-1, 0), 1e-9);
        }
        // an off-axis ray sees a tilted normal (converging lens: normal tilts away from the axis)
        const auto off = lens.firstHit(Ray{{0, 80}, {1, 0}});
        CHECK(off.has_value());
        if (off) CHECK(off->surfaceNormal.y < 0.0);

        CHECK(lens.contains({200, 100}));
        CHECK(lens.contains({190, 140})); // the front face is at x ~ 183.9 at this height
        CHECK(!lens.contains({180, 140}));
        CHECK(!lens.contains({100, 100}));
        CHECK(!lens.contains({200, 200}));

        // reversing the profile direction keeps normals outward (orientation is detected)
        std::vector<Bezier> reversed;
        for (std::size_t i = lens.profile().segmentCount(); i-- > 0;) {
            const auto pts = lens.profile().segment(i).controlPoints();
            reversed.emplace_back(std::vector<Vec2>(pts.rbegin(), pts.rend()));
        }
        const OpticalSurface rev{CompositeBezier(reversed)};
        const auto rh = rev.firstHit(Ray{{0, 100}, {1, 0}});
        CHECK(rh.has_value());
        if (rh) CHECK_VEC_NEAR(rh->surfaceNormal, Vec2(-1, 0), 1e-9);

        // flipping normals on an open mirror
        OpticalSurface m(CompositeBezier({Bezier::line({0, 0}, {0, 10})}), SurfaceKind::Reflective);
        const Vec2 before = m.surfaceNormalAt(0.5);
        m.flipNormals();
        CHECK_VEC_NEAR(m.surfaceNormalAt(0.5), -before, 0.0);
        CHECK(!m.contains({0, 5}));
    }

    void testInvalidOpticalInput(test::Context& ctx) {
        bool threw = false;
        try { OpticalSurface s{CompositeBezier{}}; } catch (const std::invalid_argument&) { threw = true; }
        CHECK(threw);
        threw = false;
        try { parabolicProfile({0, 0}, {1, 0}, -1.0, 5.0); } catch (const std::invalid_argument&) { threw = true; }
        CHECK(threw);
        threw = false;
        try { lensProfile(LensSpec{{0, 0}, 100, 0, -30, -30}); } catch (const std::invalid_argument&) { threw = true; }
        CHECK(threw);
    }

    /*
     * Gradient-index ray paths
     */
    void testGrinUniformMedium(test::Context& ctx) {
        GrinOptions opts;
        opts.step = 2.0;
        opts.maxSteps = 50;
        const RayPath path = traceGradientIndex([](Vec2) { return 1.5; }, {0, 0}, {3, 4}, opts);
        CHECK(path.points.size() == 51);
        CHECK_NEAR(path.length, 100.0, 1e-9);
        CHECK_VEC_NEAR(path.points.back(), Vec2(60, 80), 1e-9);
        for (const Vec2& d : path.directions) CHECK_VEC_NEAR(d, Vec2(0.6, 0.8), 1e-12);
    }

    void testGrinParabolicPath(test::Context& ctx) {
        // n^2 = n0^2 + b y  =>  the exact path is y = y0 + (Ty/Tx)(x - x0) + b/(4 Tx^2) (x - x0)^2
        const double n0 = 1.5, b = 0.02;
        auto n = [=](Vec2 p) { return std::sqrt(n0 * n0 + b * p.y); };
        GrinOptions opts;
        opts.step = 5.0;
        opts.maxSteps = 60;
        opts.gradient = [=](Vec2 p) { return Vec2(0, b / (2 * std::sqrt(n0 * n0 + b * p.y))); };
        const Vec2 d0 = Vec2(1, -0.3).normalized();
        const RayPath path = traceGradientIndex(n, {0, 0}, d0, opts);
        const double tx = n0 * d0.x, ty = n0 * d0.y;
        auto exactY = [&](double x) { return (ty / tx) * x + b / (4 * tx * tx) * x * x; };
        for (const Vec2& p : path.points) CHECK_NEAR(p.y, exactY(p.x), 1e-9);

        // the Bezier conversion is exact too (cubic Hermite through a quadratic in sigma)
        const CompositeBezier curve = path.toCurve();
        CHECK(curve.segmentCount() == path.points.size() - 1);
        CHECK(curve.continuity(1e-9) >= Continuity::G1);
        for (double u = 0.0; u <= curve.paramMax(); u += 0.37) {
            const Vec2 p = curve.evaluate(u);
            CHECK_NEAR(p.y, exactY(p.x), 1e-8);
        }
        // the ray bends towards the higher index (upwards, +y)
        CHECK(path.directions.back().y > d0.y);
    }

    void testGrinSnellInvariantAndTurning(test::Context& ctx) {
        // layered medium n(y) = 1.6 - 0.002 y^2: T.x = n cos(theta) is conserved
        auto n = [](Vec2 p) { return 1.6 - 0.002 * p.y * p.y; };
        GrinOptions opts;
        opts.step = 0.5;
        opts.maxSteps = 2000;
        const RayPath path = traceGradientIndex(n, {0, 0}, Vec2(1, 0.15).normalized(), opts);
        const double tx0 = path.opticalDirs.front().x;
        double maxY = 0.0, minY = 0.0;
        for (std::size_t i = 0; i < path.points.size(); ++i) {
            CHECK_NEAR(path.opticalDirs[i].x, tx0, 1e-9);
            CHECK_NEAR(path.opticalDirs[i].length(), n(path.points[i]), 1e-6); // |T| = n
            maxY = std::max(maxY, path.points[i].y);
            minY = std::min(minY, path.points[i].y);
        }
        // the ray is guided: it turns back towards the axis instead of escaping (GRIN fibre)
        CHECK(maxY > 1.0 && maxY < 10.0);
        CHECK(minY < -1.0);
    }

    void testGrinStopsAndDegenerate(test::Context& ctx) {
        GrinOptions opts;
        opts.stop = [](Vec2 p) { return p.x >= 10.0; };
        const RayPath stopped = traceGradientIndex([](Vec2) { return 1.0; }, {0, 0}, {1, 0}, opts);
        CHECK(stopped.points.back().x >= 10.0 && stopped.points.back().x < 11.0);

        opts = {};
        opts.maxLength = 25.0;
        CHECK(traceGradientIndex([](Vec2) { return 1.0; }, {0, 0}, {1, 0}, opts).length >= 25.0);

        CHECK(traceGradientIndex([](Vec2) { return 1.0; }, {0, 0}, {0, 0}).points.size() == 1);
        CHECK(traceGradientIndex([](Vec2) { return -1.0; }, {0, 0}, {1, 0}).points.size() == 1);
        // the index becomes invalid for x > 5: tracing stops there instead of producing NaN
        const RayPath edge = traceGradientIndex([](Vec2 p) { return p.x > 5.0 ? 0.0 : 1.0; }, {0, 0}, {1, 0});
        CHECK(edge.points.back().x <= 6.0);
        CHECK(std::isfinite(edge.points.back().x));
        CHECK(RayPath{}.toCurve().empty());
    }
}

int main() {
    return test::runTests("curves: optics", {
        {"parabola vs analytic rays", testParabolaAnalytic},
        {"tangent and missing rays", testTangentAndMissingRays},
        {"near-vertical and degenerate", testNearVerticalAndDegenerate},
        {"random rays vs brute force", testAgainstBruteForce},
        {"composite joins and toHit", testCompositeJoins},
        {"parabolic mirror focus", testParabolicMirrorFocus},
        {"lens surface", testLensSurface},
        {"invalid optical input", testInvalidOpticalInput},
        {"grin: uniform medium", testGrinUniformMedium},
        {"grin: parabolic path (exact)", testGrinParabolicPath},
        {"grin: Snell invariant and guiding", testGrinSnellInvariantAndTurning},
        {"grin: stops and degenerate input", testGrinStopsAndDegenerate},
    });
}
