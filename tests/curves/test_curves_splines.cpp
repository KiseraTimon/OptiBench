#include <cmath>
#include <stdexcept>
#include <vector>

#include "TestHarness.h"
#include "optibench/curves/BSpline.h"
#include "optibench/curves/CatmullRom.h"
#include "optibench/curves/CubicSpline.h"
#include "optibench/curves/Hermite.h"

using namespace ob;
using namespace ob::curves;

namespace {
    const std::vector<Vec2> kPoints{{0, 0}, {2, 3}, {5, 4}, {7, 1}, {10, 2}, {12, 6}};

    // true if calling f throws std::invalid_argument
    template <class F>
    bool throwsInvalid(F&& f) {
        try { f(); } catch (const std::invalid_argument&) { return true; }
        return false;
    }

    /*
     * Natural cubic spline
     */
    void testNaturalInterpolation(test::Context& ctx) {
        for (Parametrization par : {Parametrization::Uniform, Parametrization::Centripetal, Parametrization::Chordal}) {
            const NaturalCubicSpline s(kPoints, par);
            for (std::size_t i = 0; i < kPoints.size(); ++i) CHECK_VEC_NEAR(s.evaluate(s.knots()[i]), kPoints[i], 1e-12);
            // natural end conditions
            CHECK_VEC_NEAR(s.derivative(s.paramMin(), 2), Vec2(), 1e-12);
            CHECK_VEC_NEAR(s.derivative(s.paramMax(), 2), Vec2(), 1e-12);
        }
    }

    void testNaturalC2Continuity(test::Context& ctx) {
        const NaturalCubicSpline s(kPoints);
        constexpr double h = 1e-9;
        for (std::size_t i = 1; i + 1 < kPoints.size(); ++i) {
            const double t = s.knots()[i];
            CHECK_VEC_NEAR(s.derivative(t - h, 1), s.derivative(t + h, 1), 1e-6);
            CHECK_VEC_NEAR(s.derivative(t - h, 2), s.derivative(t + h, 2), 1e-6);
        }
    }

    void testNaturalKnownSolution(test::Context& ctx) {
        // (0,0), (1,1), (2,0) uniform: 4 M1 = 6 ((-1) - 1)  =>  M1.y = -3, x is linear so M1.x = 0
        const NaturalCubicSpline s({{0, 0}, {1, 1}, {2, 0}}, Parametrization::Uniform);
        CHECK_VEC_NEAR(s.secondDerivatives()[1], Vec2(0, -3), 1e-12);
        // collinear points give a straight line
        const NaturalCubicSpline line({{0, 0}, {1, 2}, {3, 6}, {4, 8}});
        for (double t = line.paramMin(); t <= line.paramMax(); t += 0.37) CHECK_NEAR(line.evaluate(t).y, 2 * line.evaluate(t).x, 1e-9);
        // two points: a segment
        const NaturalCubicSpline two({{0, 0}, {4, 0}});
        CHECK_VEC_NEAR(two.evaluate(2.0), Vec2(2, 0), 1e-12);
    }

    void testNaturalToBezier(test::Context& ctx) {
        const NaturalCubicSpline s(kPoints);
        const CompositeBezier b = s.toBezier();
        CHECK(b.segmentCount() == kPoints.size() - 1);
        for (std::size_t i = 0; i < b.segmentCount(); ++i) {
            const double t0 = s.knots()[i], t1 = s.knots()[i + 1];
            for (double f : {0.0, 0.3, 0.5, 1.0})
                CHECK_VEC_NEAR(b.evaluate(static_cast<double>(i) + f), s.evaluate(t0 + f * (t1 - t0)), 1e-10);
        }
        CHECK(b.continuity(1e-9) >= Continuity::G1);
    }

    void testNaturalDegenerate(test::Context& ctx) {
        CHECK(throwsInvalid([] { NaturalCubicSpline({{1, 1}}); }));
        CHECK(throwsInvalid([] { NaturalCubicSpline({{1, 1}, {1, 1}, {1, 1}}); }));
        // duplicates are removed rather than dividing by a zero knot gap
        const NaturalCubicSpline s({{0, 0}, {1, 1}, {1, 1}, {2, 0}});
        CHECK(s.points().size() == 3);
        CHECK(std::isfinite(s.evaluate(0.5).x));
    }

    /*
     * Hermite
     */
    void testHermite(test::Context& ctx) {
        // partition of unity of the position basis, and endpoint values
        for (double t : {0.0, 0.2, 0.5, 0.8, 1.0}) {
            const auto h = hermiteBasis(t);
            CHECK_NEAR(h[0] + h[2], 1.0, 1e-15);
        }
        const HermiteSegment seg{{0, 0}, {3, 6}, {4, 0}, {3, -6}};
        CHECK_VEC_NEAR(seg.evaluate(0), Vec2(0, 0), 0.0);
        CHECK_VEC_NEAR(seg.evaluate(1), Vec2(4, 0), 1e-15);
        CHECK_VEC_NEAR(seg.derivative(0), Vec2(3, 6), 1e-15);
        CHECK_VEC_NEAR(seg.derivative(1), Vec2(3, -6), 1e-15);
        const Bezier bz = seg.toBezier();
        for (double t : {0.1, 0.4, 0.9}) {
            CHECK_VEC_NEAR(bz.evaluate(t), seg.evaluate(t), 1e-12);
            CHECK_VEC_NEAR(bz.derivative(t), seg.derivative(t), 1e-12);
        }

        const HermiteSpline spline(kPoints, {{1, 1}, {2, 0}, {1, -1}, {2, 0}, {1, 2}, {0, 3}});
        CHECK(spline.segmentCount() == kPoints.size() - 1);
        for (std::size_t i = 0; i < kPoints.size(); ++i) CHECK_VEC_NEAR(spline.evaluate(static_cast<double>(i)), kPoints[i], 1e-12);
        CHECK(spline.toBezier().continuity() == Continuity::C1);
        CHECK(throwsInvalid([] { HermiteSpline({{0, 0}, {1, 1}}, {{1, 0}}); }));
        CHECK(throwsInvalid([] { HermiteSpline({{0, 0}}, {{1, 0}}); }));
    }

    /*
     * Catmull-Rom
     */
    void testCatmullRomInterpolation(test::Context& ctx) {
        for (double alpha : {0.0, 0.5, 1.0}) {
            const CatmullRom cr(kPoints, alpha);
            CHECK(cr.segmentCount() == kPoints.size() - 1);
            for (std::size_t i = 0; i < kPoints.size(); ++i) CHECK_VEC_NEAR(cr.evaluate(static_cast<double>(i)), kPoints[i], 1e-12);
            CHECK(cr.toBezier().continuity() >= Continuity::G1);
        }
        CHECK(CatmullRom(kPoints, Parametrization::Uniform).toBezier().continuity() == Continuity::C1);
    }

    void testCatmullRomUniformMatchesClassicFormula(test::Context& ctx) {
        const Vec2 p0{0, 0}, p1{1, 2}, p2{3, 3}, p3{4, 1};
        const CatmullRom cr({p0, p1, p2, p3}, Parametrization::Uniform);
        for (double t : {0.0, 0.25, 0.5, 0.75, 1.0}) {
            const Vec2 classic = (p1 * 2.0 + (p2 - p0) * t + (p0 * 2.0 - p1 * 5.0 + p2 * 4.0 - p3) * (t * t) +
                                  (p1 * 3.0 - p0 - p2 * 3.0 + p3) * (t * t * t)) * 0.5;
            CHECK_VEC_NEAR(cr.evaluate(1.0 + t), classic, 1e-12);
        }
        // the tangent at an interior point is (P(i+1) - P(i-1)) / 2
        CHECK_VEC_NEAR(cr.derivative(1.0), (p2 - p0) * 0.5, 1e-12);
    }

    void testCatmullRomTensionAndCentripetal(test::Context& ctx) {
        // tension 1: zero tangents, every segment lies on its chord
        const CatmullRom tight(kPoints, 0.5, 1.0);
        for (double u = 0.0; u <= 5.0; u += 0.13) {
            const auto [i, t] = tight.toBezier().locate(u);
            const Vec2 a = kPoints[i], b = kPoints[i + 1];
            CHECK_NEAR((tight.evaluate(u) - a).cross(b - a), 0.0, 1e-9);
        }
        // a short segment next to long ones: uniform overshoots far more than centripetal
        const std::vector<Vec2> uneven{{0, 0}, {10, 0}, {10.5, 0.5}, {20, 0}};
        auto deviation = [&](double alpha) {
            const CatmullRom cr(uneven, alpha);
            double worst = 0.0;
            for (int k = 0; k <= 100; ++k) {
                const Vec2 p = cr.evaluate(1.0 + k / 100.0);
                worst = std::max(worst, (p - uneven[1]).length() + (p - uneven[2]).length() - (uneven[2] - uneven[1]).length());
            }
            return worst;
        };
        CHECK(deviation(0.5) < deviation(0.0));
    }

    void testCatmullRomClosedAndDegenerate(test::Context& ctx) {
        const CatmullRom loop({{0, 0}, {4, 0}, {4, 4}, {0, 4}, {0, 0}}, 0.5, 0.0, true);
        CHECK(loop.points().size() == 4); // repeated closing point dropped
        CHECK(loop.segmentCount() == 4);
        CHECK(loop.toBezier().isClosed());
        CHECK(loop.toBezier().continuity() >= Continuity::G1);

        CHECK(throwsInvalid([] { CatmullRom({{0, 0}}); }));
        CHECK(throwsInvalid([] { CatmullRom({{0, 0}, {1, 0}}, 0.5, 0.0, true); }));
        CHECK(throwsInvalid([] { CatmullRom({{0, 0}, {1, 0}}, -1.0); }));
        CHECK(throwsInvalid([] { CatmullRom({{0, 0}, {1, 0}}, 0.5, std::nan("")); }));
        const CatmullRom dup({{0, 0}, {0, 0}, {5, 5}, {5, 5}, {10, 0}});
        CHECK(dup.segmentCount() == 2);
        CHECK(std::isfinite(dup.evaluate(0.5).x));
        const CatmullRom two({{0, 0}, {4, 2}});
        CHECK_VEC_NEAR(two.evaluate(0.5), Vec2(2, 1), 1e-12);
    }

    /*
     * B-splines
     */
    void testBSplineValidation(test::Context& ctx) {
        CHECK(throwsInvalid([] { BSpline(3, {{0, 0}, {1, 1}}, {0, 0, 0, 0, 1, 1}); }));          // too few points
        CHECK(throwsInvalid([] { BSpline(1, {{0, 0}, {1, 1}}, {0, 0, 1}); }));                    // wrong knot count
        CHECK(throwsInvalid([] { BSpline(1, {{0, 0}, {1, 1}}, {0, 1, 0.5, 1}); }));               // decreasing
        CHECK(throwsInvalid([] { BSpline(1, {{0, 0}, {1, 1}}, {0, 1, 1, 2}); }));                 // empty domain
        CHECK(throwsInvalid([] { BSpline(-1, {{0, 0}}, {0}); }));
        CHECK(throwsInvalid([] { BSpline::clamped(3, {{0, 0}, {1, 1}}); }));
    }

    void testBSplineClamped(test::Context& ctx) {
        for (int p = 1; p <= 4; ++p) {
            const BSpline s = BSpline::clamped(p, kPoints);
            CHECK(s.isClamped());
            CHECK_VEC_NEAR(s.evaluate(s.paramMin()), kPoints.front(), 1e-12);
            CHECK_VEC_NEAR(s.evaluate(s.paramMax()), kPoints.back(), 1e-12);
            // end tangents follow the control polygon
            CHECK_NEAR(s.tangent(0.0).cross((kPoints[1] - kPoints[0]).normalized()), 0.0, 1e-12);
            CHECK(s.tangent(0.0).dot(kPoints[1] - kPoints[0]) > 0.0);
        }
        // degree 1: the control polygon itself
        const BSpline lin = BSpline::clamped(1, kPoints);
        CHECK_VEC_NEAR(lin.evaluate(0.1), lerp(kPoints[0], kPoints[1], 0.5), 1e-12);
        // degree n with n + 1 points and Bezier knots: exactly the Bezier curve
        const std::vector<Vec2> four(kPoints.begin(), kPoints.begin() + 4);
        const BSpline asBezier = BSpline::clamped(3, four);
        const Bezier bz(four);
        for (double t : {0.0, 0.3, 0.7, 1.0}) CHECK_VEC_NEAR(asBezier.evaluate(t), bz.evaluate(t), 1e-12);
    }

    void testBSplineBasis(test::Context& ctx) {
        const BSpline s(3, kPoints, {0, 0, 0, 0, 0.2, 0.7, 1, 1, 1, 1});
        for (double u = 0.0; u <= 1.0; u += 0.05) {
            std::size_t span = 0;
            const std::vector<double> n = s.basis(u, span);
            double sum = 0.0;
            Vec2 viaBasis;
            for (std::size_t j = 0; j < n.size(); ++j) {
                CHECK(n[j] >= -1e-15);
                sum += n[j];
                viaBasis = viaBasis + kPoints[span - 3 + j] * n[j];
            }
            CHECK_NEAR(sum, 1.0, 1e-12);
            CHECK_VEC_NEAR(viaBasis, s.evaluate(u), 1e-12);
        }
    }

    void testBSplineUniform(test::Context& ctx) {
        const BSpline s = BSpline::uniform(3, kPoints);
        CHECK(!s.isClamped());
        CHECK_NEAR(s.paramMin(), 3.0, 0.0);
        CHECK_NEAR(s.paramMax(), 6.0, 0.0);
        // at the knots a uniform cubic B-spline is (P(i) + 4 P(i+1) + P(i+2)) / 6
        for (int i = 0; i + 2 < static_cast<int>(kPoints.size()); ++i) {
            const auto k = static_cast<std::size_t>(i);
            CHECK_VEC_NEAR(s.evaluate(3.0 + i), (kPoints[k] + kPoints[k + 1] * 4.0 + kPoints[k + 2]) / 6.0, 1e-12);
        }
    }

    void testBSplineDerivative(test::Context& ctx) {
        const BSpline s(3, kPoints, {0, 0, 0, 0, 0.2, 0.7, 1, 1, 1, 1});
        constexpr double h = 1e-6;
        for (double u : {0.1, 0.3, 0.5, 0.9}) {
            CHECK_VEC_NEAR(s.derivative(u), (s.evaluate(u + h) - s.evaluate(u - h)) / (2 * h), 1e-4);
            CHECK_VEC_NEAR(s.derivative(u, 2), (s.derivative(u + h) - s.derivative(u - h)) / (2 * h), 1e-3);
        }
        CHECK_VEC_NEAR(s.derivative(0.5, 4), Vec2(), 0.0);
    }

    void testBSplineKnotInsertion(test::Context& ctx) {
        for (const BSpline& original : {BSpline(3, kPoints, {0, 0, 0, 0, 0.2, 0.7, 1, 1, 1, 1}), BSpline::uniform(3, kPoints),
                                        BSpline::clamped(2, kPoints)}) {
            BSpline s = original;
            const double mid = 0.5 * (s.paramMin() + s.paramMax());
            CHECK(s.insertKnot(mid) == 1);
            CHECK(s.controlPoints().size() == original.controlPoints().size() + 1);
            const int before = s.multiplicity(mid); // mid may already have been a knot
            CHECK(s.insertKnot(mid, 10) == s.degree() - before); // capped at multiplicity p
            CHECK(s.multiplicity(mid) == s.degree());
            CHECK(s.insertKnot(s.paramMax() + 1.0) == 0);
            for (int k = 0; k <= 40; ++k) {
                const double u = s.paramMin() + (s.paramMax() - s.paramMin()) * k / 40.0;
                CHECK_VEC_NEAR(s.evaluate(u), original.evaluate(u), 1e-10);
            }
            // at multiplicity p the curve passes through a control point
            bool throughControlPoint = false;
            for (const Vec2& p : s.controlPoints()) throughControlPoint = throughControlPoint || (p - s.evaluate(mid)).length() < 1e-10;
            CHECK(throughControlPoint);
        }
    }

    void testBSplineToBezier(test::Context& ctx) {
        for (const BSpline& s : {BSpline(3, kPoints, {0, 0, 0, 0, 0.2, 0.7, 1, 1, 1, 1}), BSpline::uniform(3, kPoints),
                                 BSpline::uniform(2, kPoints), BSpline::clamped(4, kPoints)}) {
            const CompositeBezier b = s.toBezier();
            // distinct domain knots
            std::vector<double> knots;
            for (double k : s.knots())
                if (k >= s.paramMin() && k <= s.paramMax() && (knots.empty() || k != knots.back())) knots.push_back(k);
            CHECK(b.segmentCount() == knots.size() - 1);
            for (std::size_t i = 0; i < b.segmentCount() && i + 1 < knots.size(); ++i) {
                CHECK(b.segment(i).degree() == s.degree());
                for (double f : {0.0, 0.25, 0.5, 1.0})
                    CHECK_VEC_NEAR(b.evaluate(static_cast<double>(i) + f), s.evaluate(knots[i] + f * (knots[i + 1] - knots[i])), 1e-10);
            }
        }
    }

    void testBSplineLocalControl(test::Context& ctx) {
        const BSpline original = BSpline::uniform(3, kPoints);
        BSpline moved = original;
        moved.setControlPoint(0, {-50, -50});
        // P0 only influences [u0, u4) = [0, 4): the curve on [4, 6] is unchanged
        for (double u = 4.0; u <= 6.0; u += 0.25) CHECK_VEC_NEAR(moved.evaluate(u), original.evaluate(u), 1e-12);
        CHECK((moved.evaluate(3.2) - original.evaluate(3.2)).length() > 1.0);
    }
}

int main() {
    return test::runTests("curves: splines", {
        {"natural: interpolation and end conditions", testNaturalInterpolation},
        {"natural: C2 continuity", testNaturalC2Continuity},
        {"natural: known solution", testNaturalKnownSolution},
        {"natural: Bezier conversion", testNaturalToBezier},
        {"natural: degenerate input", testNaturalDegenerate},
        {"hermite", testHermite},
        {"catmull-rom: interpolation", testCatmullRomInterpolation},
        {"catmull-rom: uniform = classic formula", testCatmullRomUniformMatchesClassicFormula},
        {"catmull-rom: tension and centripetal", testCatmullRomTensionAndCentripetal},
        {"catmull-rom: closed and degenerate", testCatmullRomClosedAndDegenerate},
        {"b-spline: validation", testBSplineValidation},
        {"b-spline: clamped", testBSplineClamped},
        {"b-spline: basis partition of unity", testBSplineBasis},
        {"b-spline: uniform", testBSplineUniform},
        {"b-spline: derivatives", testBSplineDerivative},
        {"b-spline: knot insertion", testBSplineKnotInsertion},
        {"b-spline: Bezier extraction", testBSplineToBezier},
        {"b-spline: local control", testBSplineLocalControl},
    });
}
