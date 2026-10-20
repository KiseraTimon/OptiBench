#include <cmath>
#include <numbers>
#include <stdexcept>
#include <vector>

#include "TestHarness.h"
#include "optibench/Line.h"
#include "optibench/circle_ellipse.h"
#include "optibench/curves/Nurbs.h"

using namespace ob;
using namespace ob::curves;

namespace {
    constexpr double kPi = std::numbers::pi;

    template <class F>
    bool throwsInvalid(F&& f) {
        try { f(); } catch (const std::invalid_argument&) { return true; }
        return false;
    }

    void testRationalBezierBasics(test::Context& ctx) {
        // all weights 1: identical to the polynomial curve
        const Bezier b({{0, 0}, {3, 7}, {8, 1}, {10, 5}});
        const RationalBezier r = RationalBezier::fromBezier(b);
        for (double t : {0.0, 0.3, 0.6, 1.0}) {
            CHECK_VEC_NEAR(r.evaluate(t), b.evaluate(t), 1e-12);
            CHECK_VEC_NEAR(r.derivative(t), b.derivative(t), 1e-10);
        }
        // quarter circle: weights (1, cos 45deg, 1)
        const RationalBezier q({{1, 0}, {1, 1}, {0, 1}}, {1, std::sqrt(0.5), 1});
        for (int k = 0; k <= 20; ++k) {
            const double t = k / 20.0;
            CHECK_NEAR(q.evaluate(t).length(), 1.0, 1e-14);
            CHECK_NEAR(q.tangent(t).dot(q.evaluate(t)), 0.0, 1e-12); // tangent is perpendicular to the radius
        }
        CHECK_NEAR(q.curvature(0.5), 1.0, 1e-6);
        // splitting reproduces the curve
        const auto [left, right] = q.split(0.3);
        for (double s : {0.0, 0.5, 1.0}) {
            CHECK_VEC_NEAR(left.evaluate(s), q.evaluate(0.3 * s), 1e-12);
            CHECK_VEC_NEAR(right.evaluate(s), q.evaluate(0.3 + 0.7 * s), 1e-12);
        }
        CHECK(throwsInvalid([] { RationalBezier({{0, 0}, {1, 1}}, {1.0}); }));
        CHECK(throwsInvalid([] { RationalBezier({{0, 0}, {1, 1}}, {1.0, 0.0}); }));
        CHECK(throwsInvalid([] { RationalBezier({}, {}); }));
    }

    void testCircleIsExact(test::Context& ctx) {
        const Vec2 c{50, -20};
        const Nurbs circle = Nurbs::circle(c, 30.0);
        CHECK(circle.degree() == 2);
        CHECK_VEC_NEAR(circle.evaluate(circle.paramMin()), circle.evaluate(circle.paramMax()), 1e-12); // closed
        for (int k = 0; k <= 400; ++k) {
            const double u = k / 400.0;
            const Vec2 p = circle.evaluate(u);
            CHECK_NEAR((p - c).length(), 30.0, 1e-12);
            CHECK_NEAR(circle.tangent(u).dot((p - c).normalized()), 0.0, 1e-9);
        }
    }

    void testArcsAndEllipse(test::Context& ctx) {
        const Nurbs arc = Nurbs::circularArc({0, 0}, 2.0, kPi / 6, kPi);
        CHECK_VEC_NEAR(arc.evaluate(0.0), Vec2(2 * std::cos(kPi / 6), 2 * std::sin(kPi / 6)), 1e-12);
        CHECK_VEC_NEAR(arc.evaluate(1.0), Vec2(-2, 0), 1e-12);
        for (int k = 0; k <= 50; ++k) CHECK_NEAR(arc.evaluate(k / 50.0).length(), 2.0, 1e-12);
        // a clockwise arc works too
        const Nurbs cw = Nurbs::circularArc({0, 0}, 1.0, 0.0, -kPi / 2);
        CHECK_VEC_NEAR(cw.evaluate(1.0), Vec2(0, -1), 1e-12);

        const Nurbs e = Nurbs::ellipse({10, 5}, 40, 15);
        for (int k = 0; k <= 100; ++k) {
            const Vec2 p = e.evaluate(k / 100.0);
            const double x = (p.x - 10) / 40, y = (p.y - 5) / 15;
            CHECK_NEAR(x * x + y * y, 1.0, 1e-12);
        }
        CHECK(throwsInvalid([] { Nurbs::circularArc({0, 0}, -1.0, 0.0, 1.0); }));
        CHECK(throwsInvalid([] { Nurbs::circularArc({0, 0}, 1.0, 1.0, 1.0); }));
        CHECK(throwsInvalid([] { Nurbs::ellipse({0, 0}, 0.0, 1.0); }));
    }

    void testUnitWeightsMatchBSpline(test::Context& ctx) {
        const std::vector<Vec2> pts{{0, 0}, {2, 3}, {5, 4}, {7, 1}, {10, 2}};
        const BSpline bs = BSpline::clamped(3, pts);
        const Nurbs nb(3, pts, std::vector<double>(pts.size(), 1.0), std::vector<double>(bs.knots().begin(), bs.knots().end()));
        for (double u : {0.0, 0.2, 0.5, 0.77, 1.0}) {
            CHECK_VEC_NEAR(nb.evaluate(u), bs.evaluate(u), 1e-12);
            CHECK_VEC_NEAR(nb.derivative(u), bs.derivative(u), 1e-9);
        }
        CHECK(throwsInvalid([] { Nurbs(2, {{0, 0}, {1, 1}, {2, 0}}, {1, 1}, {0, 0, 0, 1, 1, 1}); }));
        CHECK(throwsInvalid([] { Nurbs(2, {{0, 0}, {1, 1}, {2, 0}}, {1, -1, 1}, {0, 0, 0, 1, 1, 1}); }));
        CHECK(throwsInvalid([] { Nurbs(0, {{0, 0}}, {1}, {0, 1}); }));
    }

    void testKnotInsertionAndConversion(test::Context& ctx) {
        const Nurbs original = Nurbs::circle({0, 0}, 5.0);
        Nurbs refined = original;
        CHECK(refined.insertKnot(0.1) == 1);
        CHECK(refined.controlPoints().size() == original.controlPoints().size() + 1);
        for (int k = 0; k <= 40; ++k) CHECK_VEC_NEAR(refined.evaluate(k / 40.0), original.evaluate(k / 40.0), 1e-12);

        const std::vector<RationalBezier> pieces = original.toRationalBezier();
        CHECK(pieces.size() == 4);
        for (const RationalBezier& piece : pieces)
            for (double t : {0.0, 0.25, 0.5, 1.0}) CHECK_NEAR(piece.evaluate(t).length(), 5.0, 1e-12);
    }

    void testFlatteningWithinTolerance(test::Context& ctx) {
        const double r = 100.0;
        for (double tol : {1.0, 0.1, 0.01}) {
            const Polyline poly = Nurbs::circle({0, 0}, r).flatten(tol);
            // every vertex is on the circle and every chord's sagitta is within tolerance
            double worst = 0.0;
            for (std::size_t i = 0; i + 1 < poly.size(); ++i) {
                CHECK_NEAR(poly[i].length(), r, 1e-9);
                const double half = (poly[i + 1] - poly[i]).length() / 2;
                worst = std::max(worst, r - std::sqrt(r * r - half * half));
            }
            CHECK(worst <= tol);
        }
    }

    void testRayIntersectionMatchesCircleModule(test::Context& ctx) {
        // the NURBS circle and the circles module agree on hit points and distances
        const Vec2 c{200, 150};
        const double radius = 60.0;
        const Nurbs circle = Nurbs::circle(c, radius);
        const Ray rays[] = {{{0, 150}, {1, 0}}, {{0, 100}, {1, 0.1}}, {{200, 0}, {0.05, 1}}, {{350, 300}, {-1, -1}}};
        for (const Ray& ray : rays) {
            const Hit expected = rayCircleIntersect(ray, c, radius);
            const auto hits = circle.intersect(ray);
            CHECK(expected.hit);
            CHECK(hits.size() == 2);
            if (hits.empty() || !expected.hit) continue;
            CHECK_VEC_NEAR(hits[0].point, expected.point, 1e-9);
            // rayCircleIntersect's t is in units of |direction| (not a distance as Ray.h documents
            // and the line module implements); curves follow Ray.h, so rescale for the comparison
            CHECK_NEAR(hits[0].distance, expected.t * ray.direction.length(), 1e-9);
            // the facing normal points out of the circle, towards the incoming ray
            CHECK_VEC_NEAR(hits[0].normal, (hits[0].point - c).normalized(), 1e-9);
        }
        // tangent and missing rays
        const auto graze = circle.intersect(Ray{{0, c.y - radius}, {1, 0}});
        CHECK(graze.size() == 1);
        if (!graze.empty()) {
            CHECK(graze[0].tangential);
            CHECK_VEC_NEAR(graze[0].normal, Vec2(0, -1), 1e-6);
        }
        CHECK(circle.intersect(Ray{{0, c.y - radius - 1}, {1, 0}}).empty());
        // through a span join (angle 0 is where pieces meet): reported once per crossing
        CHECK(circle.intersect(Ray{{c.x + radius, 0}, {0, 1}}).size() == 1);
    }
}

int main() {
    return test::runTests("curves: nurbs", {
        {"rational Bezier basics", testRationalBezierBasics},
        {"circle is exact", testCircleIsExact},
        {"arcs and ellipse", testArcsAndEllipse},
        {"unit weights = B-spline", testUnitWeightsMatchBSpline},
        {"knot insertion and conversion", testKnotInsertionAndConversion},
        {"flattening within tolerance", testFlatteningWithinTolerance},
        {"ray hits match circle module", testRayIntersectionMatchesCircleModule},
    });
}
