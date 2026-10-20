#include <cmath>

#include "TestHarness.h"
#include "optibench/curves/Composite.h"

using namespace ob;
using namespace ob::curves;

namespace {
    // two cubics meeting at (3, 0) with a deliberate kink
    CompositeBezier kinked() {
        return CompositeBezier({Bezier({{0, 0}, {1, 2}, {2, 2}, {3, 0}}), Bezier({{3, 0}, {4, 1}, {5, -1}, {6, 0}})});
    }

    void testEvaluationAndLocate(test::Context& ctx) {
        const CompositeBezier c = kinked();
        CHECK(c.segmentCount() == 2);
        CHECK_NEAR(c.paramMax(), 2.0, 0.0);
        CHECK_VEC_NEAR(c.evaluate(0.0), Vec2(0, 0), 0.0);
        CHECK_VEC_NEAR(c.evaluate(1.0), Vec2(3, 0), 1e-12);
        CHECK_VEC_NEAR(c.evaluate(2.0), Vec2(6, 0), 1e-12);
        CHECK_VEC_NEAR(c.evaluate(1.5), c.segment(1).evaluate(0.5), 1e-12);
        CHECK_VEC_NEAR(c.evaluate(-3.0), Vec2(0, 0), 0.0); // clamped
        CHECK_VEC_NEAR(c.evaluate(9.0), Vec2(6, 0), 1e-12);
        const auto [i, t] = c.locate(1.25);
        CHECK(i == 1);
        CHECK_NEAR(t, 0.25, 1e-15);

        const CompositeBezier empty;
        CHECK_VEC_NEAR(empty.evaluate(0.5), Vec2(), 0.0);
        CHECK(empty.flatten(0.1).empty());
        CHECK(empty.continuity() == Continuity::C1);
        CHECK(!empty.isClosed());
    }

    void testContinuityDetection(test::Context& ctx) {
        CHECK(kinked().continuityAt(0) == Continuity::C0);

        // broken chain
        CompositeBezier gap({Bezier::line({0, 0}, {1, 0}), Bezier::line({1, 1}, {2, 1})});
        CHECK(gap.continuityAt(0) == Continuity::None);
        CHECK(gap.continuity() == Continuity::None);

        // same direction, different speed: G1 but not C1
        CompositeBezier g1({Bezier({{0, 0}, {1, 0}, {2, 0}}), Bezier({{2, 0}, {5, 0}, {6, 1}})});
        CHECK(g1.continuityAt(0) == Continuity::G1);

        // equal derivatives across degrees: quadratic end derivative 2*(P2-P1) = (2,0); cubic needs Q1 - Q0 = (2/3, 0)
        CompositeBezier c1({Bezier({{0, 0}, {1, 1}, {2, 1}}), Bezier({{2, 1}, {2 + 2.0 / 3.0, 1}, {4, 0}, {5, 0}})});
        CHECK(c1.continuityAt(0) == Continuity::C1);
        CHECK(c1.continuity() == Continuity::C1);

        // out-of-range join
        CHECK(c1.continuityAt(5) == Continuity::None);
    }

    void testEnforceContinuity(test::Context& ctx) {
        for (Continuity target : {Continuity::C0, Continuity::G1, Continuity::C1}) {
            CompositeBezier c({Bezier({{0, 0}, {1, 2}, {2, 2}, {3, 0}}), Bezier({{3.5, 0.5}, {4, 1}, {5, -1}, {6, 0}}),
                               Bezier({{6, 0}, {7, 3}, {8, 0}})});
            CHECK(c.continuity() == Continuity::None);
            c.enforceAll(target);
            CHECK(c.continuity() >= target);
            // the first segment is never moved
            CHECK_VEC_NEAR(c.segment(0).back(), Vec2(3, 0), 0.0);
            for (std::size_t j = 0; j < c.joinCount(); ++j) {
                const Vec2 a = c.segment(j).back();
                const Vec2 b = c.segment(j + 1).front();
                CHECK_VEC_NEAR(a, b, 1e-12);
                if (target >= Continuity::G1)
                    CHECK_NEAR(c.segment(j).tangent(1.0).dot(c.segment(j + 1).tangent(0.0)), 1.0, 1e-12);
                if (target == Continuity::C1)
                    CHECK_VEC_NEAR(c.segment(j).derivative(1.0), c.segment(j + 1).derivative(0.0), 1e-9);
            }
        }
        // G1 keeps the later handle length
        CompositeBezier g({Bezier({{0, 0}, {1, 0}, {2, 0}}), Bezier({{2, 0}, {2, 3}, {4, 4}})});
        g.enforce(0, Continuity::G1);
        CHECK_VEC_NEAR(g.segment(1).controlPoint(1), Vec2(5, 0), 1e-12);

        // appending with continuity
        CompositeBezier a;
        a.append(Bezier({{0, 0}, {1, 1}, {2, 1}, {3, 0}}));
        a.append(Bezier({{9, 9}, {9, 9}, {4, -1}, {5, 0}}), Continuity::C1);
        CHECK(a.continuity() == Continuity::C1);
    }

    void testDegenerateJoins(test::Context& ctx) {
        // the previous segment ends with coincident points: its tangent comes from higher derivatives
        CompositeBezier c({Bezier({{0, 0}, {1, 1}, {2, 0}, {2, 0}}), Bezier({{2, 0}, {2, 5}, {4, 4}})});
        c.enforce(0, Continuity::G1);
        CHECK_NEAR(c.segment(0).tangent(1.0).dot(c.segment(1).tangent(0.0)), 1.0, 1e-12);

        // a zero-length previous segment has no direction: G1 leaves the handle alone, no NaN
        CompositeBezier d({Bezier({{1, 1}, {1, 1}}), Bezier({{1, 1}, {2, 3}, {4, 4}})});
        d.enforce(0, Continuity::G1);
        CHECK_VEC_NEAR(d.segment(1).controlPoint(1), Vec2(2, 3), 0.0);
    }

    void testClosedCurves(test::Context& ctx) {
        // a closed loop of three cubics (independent handles at each end of every segment)
        CompositeBezier loop({Bezier({{0, 0}, {3, -3}, {7, -3}, {10, 0}}), Bezier({{10, 0}, {11, 4}, {8, 9}, {5, 10}}),
                              Bezier({{5, 10}, {1, 9}, {-2, 4}, {0, 0}})});
        CHECK(loop.isClosed());
        CHECK(loop.joinCount() == 3);
        loop.enforceAll(Continuity::G1);
        CHECK(loop.continuityAt(0) >= Continuity::G1);
        CHECK(loop.continuityAt(1) >= Continuity::G1);
        CHECK(loop.continuityAt(2) >= Continuity::G1);
    }

    void testGeometry(test::Context& ctx) {
        const CompositeBezier c = kinked();
        const Polyline poly = c.flatten(0.01);
        CHECK_VEC_NEAR(poly.front(), Vec2(0, 0), 0.0);
        CHECK_VEC_NEAR(poly.back(), Vec2(6, 0), 1e-12);
        // join point appears once
        int joins = 0;
        for (const Vec2& p : poly) joins += (p - Vec2(3, 0)).length() < 1e-12 ? 1 : 0;
        CHECK(joins == 1);
        CHECK(c.flattenUniform(4).size() == 9);

        CHECK_NEAR(c.arcLength(), c.segment(0).arcLength() + c.segment(1).arcLength(), 1e-12);
        const Aabb b = c.bounds();
        CHECK(c.controlBounds().contains(b.min) && c.controlBounds().contains(b.max));
        CHECK_NEAR(b.max.y, c.segment(0).bounds().max.y, 1e-12);
        CHECK_NEAR(c.closestParam(c.evaluate(1.7)), 1.7, 1e-8);
        CHECK(c.controlPolygon().size() == 8);
    }
}

int main() {
    return test::runTests("curves: composite", {
        {"evaluation and locate", testEvaluationAndLocate},
        {"continuity detection", testContinuityDetection},
        {"enforce C0 / G1 / C1", testEnforceContinuity},
        {"degenerate joins", testDegenerateJoins},
        {"closed curves", testClosedCurves},
        {"geometry", testGeometry},
    });
}
