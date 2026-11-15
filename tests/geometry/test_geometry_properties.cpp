// Randomised property tests (fixed seed, so every run is identical): the geometry routines must
// agree with each other on thousands of random concave polygons, rays and convex pairs.
#include <algorithm>
#include <cmath>
#include <random>
#include <vector>

#include "../curves/TestHarness.h"
#include "optibench/geometry/Geometry.h"

using namespace ob;
using namespace ob::geom;

namespace {
    using Poly = std::vector<Vec2>;

    std::mt19937& rng() {
        static std::mt19937 gen(20261115u);
        return gen;
    }
    double uniform(double lo, double hi) { return std::uniform_real_distribution<double>(lo, hi)(rng()); }
    int uniformInt(int lo, int hi) { return std::uniform_int_distribution<int>(lo, hi)(rng()); }

    // a random star-shaped (so simple, usually concave) polygon around c, randomly wound
    Poly randomStar(Vec2 c, double rMin, double rMax) {
        const int n = uniformInt(3, 24);
        std::vector<double> angles(static_cast<std::size_t>(n));
        for (double& a : angles) a = uniform(0.0, 2.0 * kPi);
        std::sort(angles.begin(), angles.end());
        Poly p;
        for (std::size_t i = 0; i < angles.size(); ++i) {
            // skip near-duplicate angles, which would make spikes thinner than the tolerances
            if (i > 0 && angles[i] - angles[i - 1] < 1e-3) continue;
            const double r = uniform(rMin, rMax);
            p.push_back(c + Vec2{std::cos(angles[i]), std::sin(angles[i])} * r);
        }
        if (uniformInt(0, 1) == 1) std::reverse(p.begin(), p.end());
        return p;
    }

    void testTriangulationProperties(test::Context& ctx) {
        int tested = 0;
        for (int iter = 0; iter < 2000; ++iter) {
            const Poly p = randomStar({uniform(-100, 100), uniform(-100, 100)}, 5, 60);
            if (!isSimple(p)) continue;
            ++tested;
            const std::vector<Triangle> tris = triangulate(p);
            if (!CHECK(tris.size() == cleaned(p).size() - 2)) continue;
            double sum = 0.0;
            bool allCcw = true;
            for (const Triangle& t : tris) {
                const double a = signedArea(Poly{p[t[0]], p[t[1]], p[t[2]]});
                allCcw = allCcw && a > 0.0;
                sum += a;
            }
            CHECK(allCcw);
            CHECK(std::fabs(sum - area(p)) <= 1e-9 * std::max(1.0, area(p)));
        }
        CHECK(tested > 1500);
    }

    void testLocateAgreesWithTriangles(test::Context& ctx) {
        for (int iter = 0; iter < 300; ++iter) {
            const Poly p = randomStar({0, 0}, 5, 60);
            const std::vector<Triangle> tris = triangulate(p);
            if (tris.empty()) continue;
            for (int k = 0; k < 40; ++k) {
                const Vec2 q{uniform(-70, 70), uniform(-70, 70)};
                if (distanceToBoundary(p, q) < 1e-6) continue;
                bool inTri = false;
                for (const Triangle& t : tris) {
                    const Poly tri{p[t[0]], p[t[1]], p[t[2]]};
                    if (locate(tri, q, 0.0) != PointLocation::Outside) { inTri = true; break; }
                }
                CHECK((locate(p, q) == PointLocation::Inside) == inTri);
            }
        }
    }

    void testRayCrossingsAlternate(test::Context& ctx) {
        for (int iter = 0; iter < 1500; ++iter) {
            const Poly p = randomStar({0, 0}, 10, 60);
            if (!isSimple(p)) continue;
            const Vec2 o{uniform(-90, 90), uniform(-90, 90)};
            if (distanceToBoundary(p, o) < 1e-6) continue;
            const double a = uniform(0, 2 * kPi);
            const Ray ray{o, {std::cos(a), std::sin(a)}};

            const std::vector<PolygonHit> hits = rayPolygonAll(ray, p);
            bool inside = locate(p, o) == PointLocation::Inside;

            // every crossing flips inside/outside, and the entering flag says which way
            bool ok = true;
            for (std::size_t i = 0; i < hits.size(); ++i) {
                ok = ok && hits[i].entering == !inside;
                ok = ok && std::fabs(hits[i].normal.length() - 1.0) < 1e-9;
                ok = ok && distanceToBoundary(p, hits[i].point) < 1e-7;
                inside = !inside;
                // the stretch up to the next crossing really is on that side
                const double tNext = i + 1 < hits.size() ? hits[i + 1].t : hits[i].t + 1000.0;
                const Vec2 mid = ray.origin + ray.direction * ((hits[i].t + tNext) / 2.0);
                if (distanceToBoundary(p, mid) > 1e-6) {
                    ok = ok && (locate(p, mid) == PointLocation::Inside) == inside;
                }
            }
            ok = ok && !inside;                 // a ray always ends up outside
            CHECK(ok);

            // the nearest hit is the first in the list
            const Hit h = rayPolygon(ray, p);
            CHECK(h.hit == !hits.empty());
            if (h.hit) CHECK(std::fabs(h.t - hits.front().t) < 1e-12);
        }
    }

    void testHullProperties(test::Context& ctx) {
        for (int iter = 0; iter < 500; ++iter) {
            Poly pts;
            const int n = uniformInt(1, 60);
            for (int i = 0; i < n; ++i) pts.push_back({uniform(-50, 50), uniform(-50, 50)});
            const Poly hull = convexHull(pts);
            if (hull.size() < 3) continue;
            CHECK(isConvex(hull));
            CHECK(winding(hull) == Winding::CounterClockwise);
            bool all = true;
            for (const Vec2& q : pts) all = all && contains(hull, q, 1e-9);
            CHECK(all);
            for (const Vec2& v : hull) {
                CHECK(std::any_of(pts.begin(), pts.end(), [&](Vec2 q) { return samePoint(q, v); }));
            }
        }
    }

    void testSatAgreement(test::Context& ctx) {
        for (int iter = 0; iter < 1500; ++iter) {
            const Poly a = regularPolygon({uniform(-20, 20), uniform(-20, 20)}, uniform(3, 15), uniformInt(3, 9), uniform(0, 6));
            const Poly b = regularPolygon({uniform(-20, 20), uniform(-20, 20)}, uniform(3, 15), uniformInt(3, 9), uniform(0, 6));

            const Separation s = separateConvex(a, b);
            CHECK(s.overlapping == polygonsOverlap(a, b));
            CHECK(polygonsOverlap(a, b) == polygonsOverlap(b, a));
            if (!s.overlapping) continue;

            // the minimum translation vector separates them, and nothing shorter along it does
            Poly pushed, half;
            for (const Vec2& v : b) {
                pushed.push_back(v + s.normal * (s.depth + 1e-7));
                half.push_back(v + s.normal * (s.depth * 0.9));
            }
            CHECK(!separateConvex(a, pushed).overlapping);
            CHECK(separateConvex(a, half).overlapping || s.depth * 0.1 <= kEps);
        }
    }

    void testLensProperties(test::Context& ctx) {
        for (int iter = 0; iter < 300; ++iter) {
            const double r1 = uniformInt(0, 3) == 0 ? Lens::kFlat : uniform(30, 200);
            const double r2 = uniformInt(0, 3) == 0 ? Lens::kFlat : uniform(30, 200);
            const double dMax = std::min(std::isinf(r1) ? 60.0 : r1, std::isinf(r2) ? 60.0 : r2);
            const double d = uniform(1, 0.8 * dMax);
            const double ap = (std::isinf(r1) && std::isinf(r2)) || uniformInt(0, 2) == 0 ? uniform(5, 40) : Lens::kFlat;
            const Lens lens(r1, r2, d, ap);

            const Poly outline = lens.outline(96);
            CHECK(isConvex(outline));

            // ray hits agree with the polygon outline (to within the outline's sampling error)
            const Vec2 o{uniform(-300, -150), uniform(-60, 60)};
            const Vec2 target{uniform(-d / 2, d / 2), uniform(-lens.halfHeight(), lens.halfHeight())};
            const Ray ray{o, target - o};
            const Hit h = lens.intersect(ray);
            const Hit hp = rayPolygon(ray, outline);
            if (lens.contains(target)) CHECK(h.hit);           // aimed at a point inside: must hit
            if (h.hit && hp.hit) CHECK(std::fabs(h.t - hp.t) < 0.05);
            if (h.hit) {
                CHECK(lens.contains(h.point, 1e-7));
                CHECK(h.normal.dot(ray.direction) < 0.0);       // entering from outside
            }
        }
    }
}

int main() {
    return test::runTests("geometry properties (randomised, fixed seed)", {
        {"triangulation of random concave polygons", testTriangulationProperties},
        {"point location agrees with the triangulation", testLocateAgreesWithTriangles},
        {"ray crossings alternate in and out", testRayCrossingsAlternate},
        {"convex hull of random point sets", testHullProperties},
        {"SAT agrees with triangulated overlap; MTV separates", testSatAgreement},
        {"lens hits agree with its outline", testLensProperties},
    });
}
