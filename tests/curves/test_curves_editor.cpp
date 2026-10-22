#include <stdexcept>
#include <vector>

#include "TestHarness.h"
#include "optibench/curves/CurveEditor.h"

using namespace ob;
using namespace ob::curves;

namespace {
    const std::vector<Vec2> kPts{{0, 0}, {100, 0}, {100, 100}, {200, 100}};

    void testConstruction(test::Context& ctx) {
        const CurveEditor e(kPts, builders::bezier());
        CHECK(e.points().size() == 4);
        CHECK(e.curve().segmentCount() == 1);
        CHECK(e.curve().segment(0).degree() == 3);
        CHECK(e.revision() == 0);

        bool threw = false;
        try { CurveEditor bad({{0, 0}}, builders::bezier()); } catch (const std::invalid_argument&) { threw = true; }
        CHECK(threw);
        threw = false;
        try { CurveEditor bad(kPts, CurveBuilder{}); } catch (const std::invalid_argument&) { threw = true; }
        CHECK(threw);
    }

    void testHitTesting(test::Context& ctx) {
        const CurveEditor e(kPts, builders::catmullRom());
        CHECK(e.hitTest({101, 2}, 5.0) == std::optional<std::size_t>(1));
        CHECK(!e.hitTest({50, 50}, 5.0));
        CHECK(e.hitTest({100, 50}, 60.0) == std::optional<std::size_t>(1)); // tie between 1 and 2: lowest index

        // a Catmull-Rom curve passes through its points, so picking the curve there hits exactly
        const auto pick = e.hitTestCurve({100, 100}, 1.0);
        CHECK(pick.has_value());
        if (pick) {
            CHECK_NEAR(pick->distance, 0.0, 1e-6);
            CHECK_NEAR(pick->u, 2.0, 1e-6);
        }
        CHECK(!e.hitTestCurve({500, 500}, 10.0));
    }

    void testMoveRecomputes(test::Context& ctx) {
        CurveEditor e(kPts, builders::bezier());
        const Vec2 before = e.curve().evaluate(0.5);
        CHECK(e.move(1, {100, -100}));
        CHECK(e.revision() == 1);
        CHECK_VEC_NEAR(e.points()[1], Vec2(100, -100), 0.0);
        CHECK_VEC_NEAR(e.curve().segment(0).controlPoint(1), Vec2(100, -100), 0.0);
        CHECK((e.curve().evaluate(0.5) - before).length() > 1.0);
        CHECK(!e.move(10, {0, 0}));
        CHECK(e.revision() == 1);
    }

    void testInsertAndRemove(test::Context& ctx) {
        CurveEditor e(kPts, builders::bezier());
        CHECK(e.insert(4, {300, 100}));
        CHECK(e.points().size() == 5 && e.curve().segment(0).degree() == 4);
        CHECK(!e.insert(99, {0, 0}));

        // insertNear picks the nearest control polygon edge
        CurveEditor n(kPts, builders::catmullRom());
        CHECK(n.insertNear({103, 50}) == std::optional<std::size_t>(2)); // edge 1-2
        CHECK_VEC_NEAR(n.points()[2], Vec2(103, 50), 0.0);
        CHECK(n.insertNear({-50, 0}) == std::optional<std::size_t>(0));  // before the start
        const std::size_t countBefore = n.points().size();
        CHECK(n.insertNear({300, 100}) == std::optional<std::size_t>(countBefore)); // after the end

        CHECK(n.remove(0));
        CHECK_VEC_NEAR(n.points()[0], Vec2(0, 0), 0.0);
        CHECK(!n.remove(99));
        // never below the minimum point count
        CurveEditor small({{0, 0}, {1, 1}}, builders::bezier());
        CHECK(!small.remove(0));
        CHECK(small.points().size() == 2);
    }

    void testDragging(test::Context& ctx) {
        CurveEditor e(kPts, builders::naturalCubic());
        CHECK(!e.beginDrag({50, 50}, 5.0));
        CHECK(!e.dragTo({0, 0}));
        CHECK(e.beginDrag({102, 3}, 5.0)); // grabbed 2 px right, 3 px down of point 1
        CHECK(e.dragged() == std::optional<std::size_t>(1));
        CHECK(e.dragTo({152, 3}));
        CHECK_VEC_NEAR(e.points()[1], Vec2(150, 0), 1e-12); // grab offset preserved
        CHECK_VEC_NEAR(e.curve().evaluate(1.0), Vec2(150, 0), 1e-9); // interpolating: curve follows
        // inserting before the dragged point keeps the drag on the same point
        CHECK(e.insert(0, {-100, 0}));
        CHECK(e.dragged() == std::optional<std::size_t>(2));
        e.endDrag();
        CHECK(!e.dragged());
    }

    void testInvalidEditsRollBack(test::Context& ctx) {
        // a closed Catmull-Rom needs 3 distinct points: collapsing one onto another is refused
        CurveEditor e({{0, 0}, {100, 0}, {50, 80}}, builders::catmullRom(0.5, 0.0, true));
        const std::uint64_t rev = e.revision();
        const Vec2 mid = e.curve().evaluate(1.5);
        CHECK(!e.move(2, {100, 0}));
        CHECK(e.revision() == rev);
        CHECK_VEC_NEAR(e.points()[2], Vec2(50, 80), 0.0);
        CHECK_VEC_NEAR(e.curve().evaluate(1.5), mid, 0.0);
    }

    void testSwitchCurveType(test::Context& ctx) {
        CurveEditor e(kPts, builders::bezier());
        CHECK(e.setBuilder(builders::clampedBSpline(3)));
        CHECK_VEC_NEAR(e.curve().front(), kPts.front(), 1e-12);
        CHECK_VEC_NEAR(e.curve().back(), kPts.back(), 1e-12);
        CHECK(e.setBuilder(builders::catmullRom()));
        CHECK(e.curve().segmentCount() == 3);
        // a B-spline builder with few points falls back to a lower degree instead of failing
        CurveEditor two({{0, 0}, {10, 0}}, builders::clampedBSpline(3));
        CHECK(two.curve().segment(0).degree() == 1);
        CHECK(!e.setBuilder(CurveBuilder{}));
        CHECK(e.curve().segmentCount() == 3);
    }
}

int main() {
    return test::runTests("curves: editor", {
        {"construction", testConstruction},
        {"hit-testing", testHitTesting},
        {"move recomputes the curve", testMoveRecomputes},
        {"insert and remove", testInsertAndRemove},
        {"dragging", testDragging},
        {"invalid edits roll back", testInvalidEditsRollBack},
        {"switching curve type", testSwitchCurveType},
    });
}
