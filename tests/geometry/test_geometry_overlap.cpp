// Overlap tests: SAT with minimum translation vector, discs, concave polygons.
#include <cmath>
#include <vector>

#include "../curves/TestHarness.h"
#include "optibench/geometry/Overlap.h"
#include "optibench/geometry/PolygonOps.h"

using namespace ob;
using namespace ob::geom;

namespace {
    using Poly = std::vector<Vec2>;

    Poly box(double x0, double y0, double x1, double y1) { return rectangle(boundsOf(Segment{{x0, y0}, {x1, y1}})); }

    Poly moved(const Poly& p, Vec2 d) {
        Poly out;
        for (const Vec2& v : p) out.push_back(v + d);
        return out;
    }

    const Poly kL{{0, 0}, {4, 0}, {4, 1}, {1, 1}, {1, 3}, {0, 3}};
    const Poly kU{{0, 0}, {5, 0}, {5, 5}, {4, 5}, {4, 1}, {1, 1}, {1, 5}, {0, 5}};

    void testSat(test::Context& ctx) {
        const Poly a = box(0, 0, 4, 4);

        // overlapping by 1 along x (y fully overlaps: 2 + 1 to push out)
        Separation s = separateConvex(a, box(3, 1, 7, 3));
        CHECK(s.overlapping);
        CHECK_NEAR(s.depth, 1.0, 1e-12);
        CHECK_VEC_NEAR(s.normal, Vec2(1, 0), 1e-12);

        // the MTV really separates them: afterwards they only touch
        const Poly pushed = moved(box(3, 1, 7, 3), s.normal * s.depth);
        CHECK(!separateConvex(a, pushed).overlapping);
        CHECK(!polygonsOverlap(a, pushed));

        // normal points from A to B: B on the left gives -x
        s = separateConvex(a, box(-3, 1, 1, 3));
        CHECK(s.overlapping);
        CHECK_VEC_NEAR(s.normal, Vec2(-1, 0), 1e-12);

        // touching and apart do not overlap
        CHECK(!separateConvex(a, box(4, 0, 8, 4)).overlapping);
        CHECK(!separateConvex(a, box(5, 0, 8, 4)).overlapping);
        CHECK(!separateConvex(a, box(4, 4, 8, 8)).overlapping);         // corner to corner

        // containment: a small centred square must travel 4 + 2 = 6 to get out
        s = separateConvex(box(0, 0, 10, 10), box(4, 4, 6, 6));
        CHECK(s.overlapping);
        CHECK_NEAR(s.depth, 6.0, 1e-12);

        // a diamond whose tip pokes 1 into the square. The square's corner (4, 4) is also inside the
        // diamond, so the cheapest way out is diagonal: 1/sqrt(2) along (1, 1)/sqrt(2), not 1 along x
        const Poly diamond{{5, 2}, {7, 4}, {5, 6}, {3, 4}};
        s = separateConvex(a, diamond);
        CHECK(s.overlapping);
        CHECK_NEAR(s.depth, std::sqrt(0.5), 1e-12);
        CHECK_VEC_NEAR(s.normal, Vec2(1, 1) * std::sqrt(0.5), 1e-12);

        // winding does not matter; degenerate input never overlaps
        CHECK(separateConvex(box(0, 0, 4, 4), Poly{{3, 3}, {3, 6}, {6, 6}, {6, 3}}).overlapping);
        CHECK(!separateConvex(a, Poly{{1, 1}, {2, 2}}).overlapping);
    }

    void testCircles(test::Context& ctx) {
        Separation s = separateCircles({0, 0}, 3, {4, 0}, 2);
        CHECK(s.overlapping);
        CHECK_NEAR(s.depth, 1.0, 1e-12);
        CHECK_VEC_NEAR(s.normal, Vec2(1, 0), 1e-12);
        CHECK(!separateCircles({0, 0}, 3, {5, 0}, 2).overlapping);        // touching
        s = separateCircles({1, 1}, 2, {1, 1}, 1);                         // concentric
        CHECK(s.overlapping && std::fabs(s.depth - 3.0) < 1e-12 && std::fabs(s.normal.length() - 1.0) < 1e-12);

        const Poly a = box(0, 0, 4, 4);
        s = separateConvexCircle(a, {5, 2}, 2);
        CHECK(s.overlapping);
        CHECK_NEAR(s.depth, 1.0, 1e-12);
        CHECK_VEC_NEAR(s.normal, Vec2(1, 0), 1e-12);

        // near a corner the vertex axis decides: (5, 5) is sqrt(2) from the corner (4, 4)
        CHECK(!separateConvexCircle(a, {5, 5}, 1.4).overlapping);
        s = separateConvexCircle(a, {5, 5}, 1.5);
        CHECK(s.overlapping);
        CHECK_NEAR(s.depth, 1.5 - std::sqrt(2.0), 1e-12);

        CHECK(polygonCircleOverlap(kU, {2.5, 3}, 2.0));         // reaches both arms from the notch
        CHECK(!polygonCircleOverlap(kU, {2.5, 3}, 1.4));        // fits in the notch
        CHECK(!polygonCircleOverlap(kU, {2.5, 3}, 1.5));        // exactly touches: not an overlap
        CHECK(polygonCircleOverlap(kU, {0.5, 3}, 0.1));         // inside an arm
        CHECK(!polygonCircleOverlap(kU, {0.5, 3}, 0.0));
    }

    void testConcave(test::Context& ctx) {
        // a square resting in the L's notch: boxes overlap, shapes do not
        CHECK(!polygonsOverlap(kL, box(1, 1, 3, 3)));
        CHECK(polygonsOverlap(kL, box(0.5, 0.5, 2, 2)));
        // flush against the L's right end
        CHECK(!polygonsOverlap(kL, box(4, 0, 5, 1)));

        // a block in the U's notch, touching both walls and the floor: still no overlap
        CHECK(!polygonsOverlap(kU, box(1, 1, 4, 6)));
        CHECK(polygonsOverlap(kU, box(3.5, 2, 4.5, 3)));        // through the inner wall
        // the U's own convex SAT result would be wrong here: its hull covers the notch
        CHECK(separateConvex(kU, box(2, 2, 3, 3)).overlapping);
        CHECK(!polygonsOverlap(kU, box(2, 2, 3, 3)));

        // identical shapes overlap; far apart do not
        CHECK(polygonsOverlap(kU, kU));
        CHECK(!polygonsOverlap(kU, moved(kU, {20, 0})));

        // an upside-down U standing in the U's notch: legs on the floor, sides against the walls
        const Poly flipped{{1, 1}, {2, 1}, {2, 4}, {3, 4}, {3, 1}, {4, 1}, {4, 6}, {1, 6}};
        CHECK(isSimple(flipped));
        CHECK_NEAR(signedArea(flipped), 12.0, 1e-12);
        CHECK(!polygonsOverlap(kU, flipped));
        CHECK(!polygonsOverlap(flipped, kU));
        CHECK(polygonsOverlap(kU, moved(flipped, {0, -0.5})));     // pushed into the floor

        // non-simple input still gets a sensible answer from the fallback
        const Poly bowtie{{0, 0}, {2, 2}, {2, 0}, {0, 2}};
        CHECK(polygonsOverlap(bowtie, box(0.5, 0.9, 1.5, 1.1)));
        CHECK(!polygonsOverlap(bowtie, box(10, 10, 11, 11)));
    }
}

int main() {
    return test::runTests("geometry overlap", {
        {"separating axis theorem", testSat},
        {"discs", testCircles},
        {"concave polygons", testConcave},
    });
}
