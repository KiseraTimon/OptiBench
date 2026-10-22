#include <cmath>
#include <numbers>
#include <stdexcept>
#include <vector>

#include "TestHarness.h"
#include "optibench/Line.h"
#include "optibench/curves/Bezier.h"

using namespace ob;
using namespace ob::curves;

namespace {
    // the parabola y = x^2 for x in [-1, 1], exactly, as a quadratic Bezier
    Bezier parabola() { return Bezier({{-1, 1}, {0, -1}, {1, 1}}); }

    // an S-shaped cubic and a self-intersecting (looping) cubic
    Bezier sCurve() { return Bezier({{0, 0}, {100, 150}, {200, -150}, {300, 0}}); }
    Bezier loopCurve() { return Bezier({{0, 0}, {300, 200}, {-100, 200}, {200, 0}}); }

    // one curve of each degree 1..7 with irregular control points
    std::vector<Bezier> curvesOfEachDegree() {
        const std::vector<Vec2> pool{{0, 0}, {3, 7}, {-2, 5}, {8, 1}, {4, -6}, {10, 3}, {-5, -2}, {7, 9}};
        std::vector<Bezier> out;
        for (std::size_t n = 2; n <= pool.size(); ++n) out.emplace_back(std::vector<Vec2>(pool.begin(), pool.begin() + n));
        return out;
    }

    // largest distance from densely sampled curve points to the polyline
    double maxDistanceToPolyline(const Bezier& c, const Polyline& poly) {
        double worst = 0.0;
        for (int i = 0; i <= 4000; ++i) {
            const Vec2 p = c.evaluate(i / 4000.0);
            double best = std::numeric_limits<double>::infinity();
            for (std::size_t k = 0; k + 1 < poly.size(); ++k)
                best = std::min(best, distanceToSegment(p, Segment{poly[k], poly[k + 1]}));
            worst = std::max(worst, best);
        }
        return worst;
    }

    void testConstruction(test::Context& ctx) {
        bool threw = false;
        try { Bezier empty({}); } catch (const std::invalid_argument&) { threw = true; }
        CHECK(threw);

        const Bezier point({{2, 3}});
        CHECK(point.degree() == 0);
        CHECK_VEC_NEAR(point.evaluate(0.7), Vec2(2, 3), 0.0);
        CHECK_VEC_NEAR(point.tangent(0.5), Vec2(), 0.0);
        CHECK_NEAR(point.arcLength(), 0.0, 0.0);
        CHECK(point.flatten(0.1).size() == 1);
    }

    void testEndpointInterpolation(test::Context& ctx) {
        for (const Bezier& c : curvesOfEachDegree()) {
            CHECK_VEC_NEAR(c.evaluate(0.0), c.front(), 1e-12);
            CHECK_VEC_NEAR(c.evaluate(1.0), c.back(), 1e-12);
            CHECK_VEC_NEAR(c.evaluateDeCasteljau(1.0), c.back(), 1e-12);
            CHECK_VEC_NEAR(c.evaluateBernstein(0.0), c.front(), 1e-12);
        }
    }

    void testPartitionOfUnity(test::Context& ctx) {
        for (int n = 0; n <= 10; ++n) {
            for (int k = 0; k <= 20; ++k) {
                const double t = k / 20.0;
                double sum = 0.0;
                bool nonNegative = true;
                for (int i = 0; i <= n; ++i) {
                    const double b = bernstein(n, i, t);
                    sum += b;
                    nonNegative = nonNegative && b >= 0.0;
                }
                CHECK_NEAR(sum, 1.0, 1e-12);
                CHECK(nonNegative);
            }
        }
        CHECK(bernstein(3, 4, 0.5) == 0.0);
        CHECK(bernstein(3, -1, 0.5) == 0.0);
    }

    void testEvaluationMethodsAgree(test::Context& ctx) {
        for (const Bezier& c : curvesOfEachDegree()) {
            for (int k = 0; k <= 10; ++k) {
                const double t = k / 10.0;
                CHECK_VEC_NEAR(c.evaluate(t), c.evaluateDeCasteljau(t), 1e-10);
                CHECK_VEC_NEAR(c.evaluateBernstein(t), c.evaluateDeCasteljau(t), 1e-10);
            }
        }
        // the quadratic really is y = x^2
        const Bezier p = parabola();
        for (int k = 0; k <= 10; ++k) {
            const Vec2 q = p.evaluate(k / 10.0);
            CHECK_NEAR(q.y, q.x * q.x, 1e-12);
        }
    }

    void testDerivatives(test::Context& ctx) {
        constexpr double h = 1e-6;
        for (const Bezier& c : curvesOfEachDegree()) {
            CHECK(c.hodograph().degree() == std::max(c.degree() - 1, 0));
            for (double t : {0.1, 0.5, 0.9}) {
                const Vec2 fd = (c.evaluate(t + h) - c.evaluate(t - h)) / (2 * h);
                CHECK_VEC_NEAR(c.derivative(t), fd, 1e-5);
                const Vec2 fd2 = (c.derivative(t + h) - c.derivative(t - h)) / (2 * h);
                CHECK_VEC_NEAR(c.derivative(t, 2), fd2, 1e-4);
            }
            CHECK_VEC_NEAR(c.derivative(0.3, c.degree() + 1), Vec2(), 0.0);
        }
    }

    void testTangentNormalCurvature(test::Context& ctx) {
        const Bezier p = parabola();
        // at the vertex (0, 0): tangent +x, normal +y, curvature of y = x^2 is 2
        CHECK_VEC_NEAR(p.tangent(0.5), Vec2(1, 0), 1e-12);
        CHECK_VEC_NEAR(p.normal(0.5), Vec2(0, 1), 1e-12);
        CHECK_NEAR(p.curvature(0.5), 2.0, 1e-12);
        // at x = 1: curvature of y = x^2 is 2 / (1 + 4)^(3/2)
        CHECK_NEAR(p.curvature(1.0), 2.0 / std::pow(5.0, 1.5), 1e-12);

        for (const Bezier& c : curvesOfEachDegree()) {
            for (double t : {0.0, 0.25, 0.75, 1.0}) {
                const Vec2 tg = c.tangent(t);
                CHECK_NEAR(tg.length(), 1.0, 1e-12);
                CHECK_NEAR(tg.dot(c.normal(t)), 0.0, 1e-12);
            }
        }

        // a cubic approximating a unit quarter circle has curvature ~ 1 (within ~2.2%: it is not a true arc)
        constexpr double k = 0.5522847498;
        const Bezier arc({{1, 0}, {1, k}, {k, 1}, {0, 1}});
        for (double t : {0.0, 0.5, 1.0}) CHECK_NEAR(arc.curvature(t), 1.0, 0.03);
    }

    void testDegenerateTangents(test::Context& ctx) {
        // P0 = P1: B'(0) = 0, the tangent must come from the next derivative and point to P2
        const Bezier c({{0, 0}, {0, 0}, {10, 0}, {10, 10}});
        CHECK_VEC_NEAR(c.derivative(0.0), Vec2(), 1e-12);
        CHECK_VEC_NEAR(c.tangent(0.0), Vec2(1, 0), 1e-12);
        CHECK_NEAR(c.curvature(0.0), 0.0, 0.0);

        // P2 = P3: the tangent at t = 1 is the direction arriving from P1
        const Bezier e({{0, 0}, {0, 10}, {10, 10}, {10, 10}});
        CHECK_VEC_NEAR(e.tangent(1.0), Vec2(1, 0), 1e-12);

        // all control points coincident: no direction anywhere, but no NaN either
        const Bezier dot({{5, 5}, {5, 5}, {5, 5}});
        CHECK_VEC_NEAR(dot.tangent(0.5), Vec2(), 0.0);
        CHECK_VEC_NEAR(dot.normal(0.5), Vec2(), 0.0);
        CHECK_NEAR(dot.curvature(0.5), 0.0, 0.0);
        CHECK_NEAR(dot.arcLength(), 0.0, 1e-12);
        CHECK(dot.flatten(0.1).size() == 2);

        // a cusp: B'(0.5) = 0 at the interior; must stay finite
        const Bezier cusp({{0, 0}, {10, 10}, {0, 10}, {10, 0}});
        CHECK(std::isfinite(cusp.tangent(0.5).x));
        CHECK(std::isfinite(cusp.curvature(0.5)));
    }

    void testSplitReproducesCurve(test::Context& ctx) {
        for (const Bezier& c : curvesOfEachDegree()) {
            for (double t : {0.0, 0.3, 0.5, 1.0}) {
                const auto [left, right] = c.split(t);
                CHECK(left.degree() == c.degree() && right.degree() == c.degree());
                for (int k = 0; k <= 10; ++k) {
                    const double s = k / 10.0;
                    CHECK_VEC_NEAR(left.evaluate(s), c.evaluate(t * s), 1e-10);
                    CHECK_VEC_NEAR(right.evaluate(s), c.evaluate(t + (1 - t) * s), 1e-10);
                }
            }
            const Bezier mid = c.subCurve(0.2, 0.7);
            for (int k = 0; k <= 10; ++k) CHECK_VEC_NEAR(mid.evaluate(k / 10.0), c.evaluate(0.2 + 0.5 * k / 10.0), 1e-10);
            CHECK_VEC_NEAR(c.subCurve(0.7, 0.2).evaluate(0.0), c.evaluate(0.2), 1e-10);
            CHECK_VEC_NEAR(c.subCurve(0.0, 0.0).evaluate(0.5), c.front(), 1e-12);
        }
    }

    void testDegreeElevation(test::Context& ctx) {
        for (const Bezier& c : curvesOfEachDegree()) {
            const Bezier e1 = c.elevated();
            const Bezier e3 = e1.elevated().elevated();
            CHECK(e1.degree() == c.degree() + 1 && e3.degree() == c.degree() + 3);
            for (int k = 0; k <= 10; ++k) {
                CHECK_VEC_NEAR(e1.evaluate(k / 10.0), c.evaluate(k / 10.0), 1e-10);
                CHECK_VEC_NEAR(e3.evaluate(k / 10.0), c.evaluate(k / 10.0), 1e-10);
            }
        }
    }

    void testAdaptiveFlattening(test::Context& ctx) {
        for (const Bezier& c : {sCurve(), loopCurve(), parabola()}) {
            std::size_t previous = 0;
            for (double tol : {1.0, 0.1, 0.01}) {
                const Polyline poly = c.flatten(tol);
                CHECK_VEC_NEAR(poly.front(), c.front(), 0.0);
                CHECK_VEC_NEAR(poly.back(), c.back(), 0.0);
                CHECK(maxDistanceToPolyline(c, poly) <= tol);
                CHECK(poly.size() >= previous);
                previous = poly.size();
            }
        }
        // a straight cubic is flat immediately: a single segment
        CHECK(Bezier({{0, 0}, {1, 1}, {2, 2}, {3, 3}}).flatten(0.01).size() == 2);
        // nonsensical tolerances are clamped instead of recursing forever
        CHECK(sCurve().flatten(0.0).size() > 2);
        CHECK(sCurve().flatten(-1.0).size() > 2);
        CHECK(sCurve().flatten(std::nan("")).size() > 2);
    }

    void testUniformFlattening(test::Context& ctx) {
        const Polyline poly = sCurve().flattenUniform(16);
        CHECK(poly.size() == 17);
        CHECK_VEC_NEAR(poly[8], sCurve().evaluate(0.5), 1e-12);
        CHECK(sCurve().flattenUniform(0).size() == 2);

        // for the same tolerance, adaptive flattening needs fewer points than the uniform step
        // that first meets it (flat parts are not oversampled)
        const double tol = 0.05;
        const std::size_t adaptive = sCurve().flatten(tol).size();
        int n = 1;
        while (maxDistanceToPolyline(sCurve(), sCurve().flattenUniform(n)) > tol) n *= 2;
        CHECK(adaptive <= static_cast<std::size_t>(n) + 1);
    }

    void testBounds(test::Context& ctx) {
        const Aabb pb = parabola().bounds();
        CHECK_VEC_NEAR(pb.min, Vec2(-1, 0), 1e-12);
        CHECK_VEC_NEAR(pb.max, Vec2(1, 1), 1e-12);
        CHECK_VEC_NEAR(parabola().controlBounds().min, Vec2(-1, -1), 0.0);

        for (const Bezier& c : {sCurve(), loopCurve()}) {
            const Aabb tight = c.bounds();
            const Aabb loose = c.controlBounds();
            CHECK(loose.contains(tight.min) && loose.contains(tight.max));
            Aabb sampled;
            for (int i = 0; i <= 20000; ++i) sampled.expand(c.evaluate(i / 20000.0));
            // tight box contains every sample and is touched by samples on every side
            CHECK(tight.contains(sampled.min, 1e-9) && tight.contains(sampled.max, 1e-9));
            CHECK_VEC_NEAR(tight.min, sampled.min, 1e-3);
            CHECK_VEC_NEAR(tight.max, sampled.max, 1e-3);
        }
        // a horizontal segment has a zero y-derivative everywhere: must not explode
        const Aabb flat = Bezier({{0, 2}, {5, 2}, {10, 2}}).bounds();
        CHECK_VEC_NEAR(flat.min, Vec2(0, 2), 1e-12);
        CHECK_VEC_NEAR(flat.max, Vec2(10, 2), 1e-12);
    }

    void testArcLength(test::Context& ctx) {
        CHECK_NEAR(Bezier::line({0, 0}, {3, 4}).arcLength(), 5.0, 1e-12);
        CHECK_NEAR(Bezier::line({0, 0}, {3, 4}).arcLength(0.5, 1.0), 2.5, 1e-12);
        // y = x^2 on [-1, 1]: 2 * (sqrt(5)/2 + asinh(2)/4)
        const double exact = std::sqrt(5.0) + std::asinh(2.0) / 2.0;
        CHECK_NEAR(parabola().arcLength(), exact, 1e-9);
        CHECK_NEAR(parabola().arcLength(0.5, 1.0), exact / 2.0, 1e-9);
        // the elevated curve has the same length
        CHECK_NEAR(parabola().elevated().arcLength(), exact, 1e-9);
        CHECK_NEAR(parabola().arcLengthTable().totalLength(), exact, 1e-9);
    }

    void testArcLengthReparametrization(test::Context& ctx) {
        const Bezier c = sCurve();
        const double total = c.arcLength();
        for (double t : {0.0, 0.1, 0.37, 0.5, 0.92, 1.0}) CHECK_NEAR(c.paramAtArcLength(c.arcLength(0.0, t)), t, 1e-9);
        CHECK_NEAR(c.paramAtArcLength(-5.0), 0.0, 0.0);
        CHECK_NEAR(c.paramAtArcLength(total * 2), 1.0, 0.0);

        // evenly spaced samples: every piece has the same arc length
        const ArcLengthTable table = c.arcLengthTable();
        const Polyline pts = c.sampleByArcLength(10);
        CHECK(pts.size() == 11);
        for (std::size_t i = 0; i < pts.size(); ++i)
            CHECK_NEAR(table.lengthAt(c.closestParam(pts[i])), total * static_cast<double>(i) / 10.0, 1e-6);
    }

    void testClosestParam(test::Context& ctx) {
        const Bezier p = parabola();
        CHECK_NEAR(p.closestParam({0, -3}), 0.5, 1e-9);
        CHECK_NEAR(p.closestParam({5, 1}), 1.0, 1e-12);
        const double t = p.closestParam({0.3, 0.09 + 1e-3});
        CHECK_VEC_NEAR(p.evaluate(t), Vec2(0.3, 0.09), 1e-3);
    }

    void testBernsteinRoots(test::Context& ctx) {
        // f(t) = (t - 0.25)(t - 0.75) in Bernstein form: c0 = f(0), c1 = f(0) + f'(0)/2, c2 = f(1)
        const std::vector<double> two{0.1875, -0.3125, 0.1875};
        const std::vector<double> r = bernsteinRoots(two, 1e-14);
        CHECK(r.size() == 2);
        if (r.size() == 2) {
            CHECK_NEAR(r[0], 0.25, 1e-12);
            CHECK_NEAR(r[1], 0.75, 1e-12);
        }
        // (2t - 1)^2: a double root at 0.5 must be reported exactly once
        const std::vector<double> dbl{1, -1, 1};
        const std::vector<double> d = bernsteinRoots(dbl, 1e-14);
        CHECK(d.size() == 1);
        if (!d.empty()) CHECK_NEAR(d[0], 0.5, 1e-6);
        // no roots, roots at the ends, and the zero polynomial
        CHECK(bernsteinRoots(std::vector<double>{1, 2, 3}, 1e-14).empty());
        CHECK(bernsteinRoots(std::vector<double>{0, 0, 0}, 1e-14).empty());
        const std::vector<double> ends = bernsteinRoots(std::vector<double>{0, 1, 0}, 1e-14);
        CHECK(ends.size() == 2);
        if (ends.size() == 2) {
            CHECK_NEAR(ends[0], 0.0, 1e-9);
            CHECK_NEAR(ends[1], 1.0, 1e-9);
        }
    }

    void testTridiagonal(test::Context& ctx) {
        const std::vector<double> sub{0, 1, 1, 1}, diag{4, 4, 4, 4}, sup{1, 1, 1, 0};
        const std::vector<double> rhs{5, 6, 6, 5}; // solution is all ones
        const std::vector<double> x = solveTridiagonal<double>(sub, diag, sup, rhs);
        CHECK(x.size() == 4);
        for (double v : x) CHECK_NEAR(v, 1.0, 1e-12);
        const std::vector<Vec2> rhs2{{5, 10}, {6, 12}, {6, 12}, {5, 10}};
        const std::vector<Vec2> x2 = solveTridiagonal<Vec2>(sub, diag, sup, rhs2);
        CHECK(x2.size() == 4);
        for (const Vec2& v : x2) CHECK_VEC_NEAR(v, Vec2(1, 2), 1e-12);
        CHECK(solveTridiagonal<double>(sub, std::vector<double>{0, 4, 4, 4}, sup, rhs).empty());
    }
}

int main() {
    return test::runTests("curves: bezier", {
        {"construction & degree 0", testConstruction},
        {"endpoint interpolation", testEndpointInterpolation},
        {"partition of unity", testPartitionOfUnity},
        {"evaluation methods agree", testEvaluationMethodsAgree},
        {"derivatives", testDerivatives},
        {"tangent, normal, curvature", testTangentNormalCurvature},
        {"degenerate tangents", testDegenerateTangents},
        {"split reproduces curve", testSplitReproducesCurve},
        {"degree elevation", testDegreeElevation},
        {"adaptive flattening", testAdaptiveFlattening},
        {"uniform flattening", testUniformFlattening},
        {"bounding boxes", testBounds},
        {"arc length", testArcLength},
        {"arc-length reparametrization", testArcLengthReparametrization},
        {"closest point", testClosestParam},
        {"bernstein roots", testBernsteinRoots},
        {"tridiagonal solver", testTridiagonal},
    });
}
