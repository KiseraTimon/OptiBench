// Geometry primitives: orientation, closest points, segment intersection, bounding boxes.
#include <cmath>

#include "../curves/TestHarness.h"
#include "optibench/geometry/Primitives.h"

using namespace ob;
using namespace ob::geom;

namespace {
    void testOrientation(test::Context& ctx) {
        CHECK(orientation({0, 0}, {1, 0}, {0, 1}) == Orientation::CounterClockwise);
        CHECK(orientation({0, 0}, {0, 1}, {1, 0}) == Orientation::Clockwise);
        CHECK(orientation({0, 0}, {1, 1}, {3, 3}) == Orientation::Collinear);
        CHECK(orientation({0, 0}, {1, 1}, {-2, -2}) == Orientation::Collinear);     // behind a, still on the line
        CHECK(orientation({2, 2}, {2, 2}, {5, 1}) == Orientation::Collinear);        // coincident points

        // relative tolerance: the same shape gives the same answer at any scale
        for (const double s : {1e-6, 1.0, 1e6}) {
            CHECK(orientation(Vec2{0, 0} * s, Vec2{10, 0} * s, Vec2{5, 1e-3} * s) == Orientation::CounterClockwise);
            CHECK(orientation(Vec2{0, 0} * s, Vec2{10, 0} * s, Vec2{5, -1e-3} * s) == Orientation::Clockwise);
        }

        // a point off the line by far less than the relative tolerance counts as on it
        CHECK(orientation({0, 0}, {1000, 0}, {500, 1e-13}) == Orientation::Collinear);

        CHECK_NEAR(cross({0, 0}, {4, 0}, {0, 3}), 12.0, 1e-12);                     // twice the 3-4-5 area
    }

    void testClosestPoints(test::Context& ctx) {
        const Segment s{{0, 0}, {10, 0}};
        CHECK_NEAR(projectParam({5, 7}, s.a, s.b), 0.5, 1e-12);
        CHECK_NEAR(projectParam({-5, 1}, s.a, s.b), -0.5, 1e-12);
        CHECK_NEAR(projectParam({3, 3}, {1, 1}, {1, 1}), 0.0, 0.0);                 // degenerate line

        CHECK_VEC_NEAR(closestPointOnSegment({4, 3}, s), Vec2(4, 0), 1e-12);
        CHECK_VEC_NEAR(closestPointOnSegment({-4, 3}, s), Vec2(0, 0), 1e-12);       // clamped to a
        CHECK_VEC_NEAR(closestPointOnSegment({14, -3}, s), Vec2(10, 0), 1e-12);     // clamped to b
        CHECK_VEC_NEAR(closestPointOnSegment({7, 7}, Segment{{2, 2}, {2, 2}}), Vec2(2, 2), 0.0);

        CHECK(onSegment({3, 0}, s));
        CHECK(onSegment({10, 0}, s));
        CHECK(!onSegment({11, 0}, s));
        CHECK(!onSegment({3, 1e-6}, s));
        CHECK(onSegment({3, 1e-6}, s, 1e-5));

        CHECK(samePoint({1, 1}, {1 + 1e-10, 1}));
        CHECK(!samePoint({1, 1}, {1 + 1e-8, 1}));
        CHECK_NEAR(lengthSq({3, 4}), 25.0, 0.0);
    }

    void testSegmentIntersection(test::Context& ctx) {
        using K = SegmentIntersection::Kind;

        // a proper X crossing
        SegmentIntersection x = intersect({{0, 0}, {2, 2}}, {{0, 2}, {2, 0}});
        CHECK(x.kind == K::Point);
        CHECK_VEC_NEAR(x.a, Vec2(1, 1), 1e-12);
        CHECK(properlyCross({{0, 0}, {2, 2}}, {{0, 2}, {2, 0}}));

        // a T junction: an endpoint on the other segment's interior
        x = intersect({{0, 0}, {4, 0}}, {{2, 0}, {2, 3}});
        CHECK(x.kind == K::Point);
        CHECK_VEC_NEAR(x.a, Vec2(2, 0), 1e-12);
        CHECK(!properlyCross({{0, 0}, {4, 0}}, {{2, 0}, {2, 3}}));

        // sharing one endpoint at an angle
        x = intersect({{0, 0}, {1, 0}}, {{1, 0}, {1, 1}});
        CHECK(x.kind == K::Point);
        CHECK_VEC_NEAR(x.a, Vec2(1, 0), 1e-12);

        // apart, and parallel apart
        CHECK(!intersect({{0, 0}, {1, 0}}, {{2, 1}, {3, 5}}));
        CHECK(!intersect({{0, 0}, {4, 0}}, {{0, 1}, {4, 1}}));

        // lines cross, but outside the second segment
        CHECK(!intersect({{0, 0}, {4, 0}}, {{2, 1}, {2, 3}}));

        // collinear overlap: the shared stretch, in s1's direction
        x = intersect({{0, 0}, {10, 0}}, {{12, 0}, {4, 0}});
        CHECK(x.kind == K::Overlap);
        CHECK_VEC_NEAR(x.a, Vec2(4, 0), 1e-12);
        CHECK_VEC_NEAR(x.b, Vec2(10, 0), 1e-12);

        // collinear, one segment inside the other
        x = intersect({{0, 0}, {10, 10}}, {{2, 2}, {3, 3}});
        CHECK(x.kind == K::Overlap);
        CHECK_VEC_NEAR(x.a, Vec2(2, 2), 1e-12);
        CHECK_VEC_NEAR(x.b, Vec2(3, 3), 1e-12);

        // collinear, touching end to end: a single point, not an overlap
        x = intersect({{0, 0}, {5, 0}}, {{5, 0}, {9, 0}});
        CHECK(x.kind == K::Point);
        CHECK_VEC_NEAR(x.a, Vec2(5, 0), 1e-12);

        // collinear with a gap
        CHECK(!intersect({{0, 0}, {5, 0}}, {{6, 0}, {9, 0}}));

        // zero-length segments act as points
        x = intersect({{3, 0}, {3, 0}}, {{0, 0}, {5, 0}});
        CHECK(x.kind == K::Point);
        CHECK_VEC_NEAR(x.a, Vec2(3, 0), 0.0);
        CHECK(!intersect({{3, 1}, {3, 1}}, {{0, 0}, {5, 0}}));

        // a near miss stays a miss; within tolerance it touches
        CHECK(!intersect({{0, 0}, {4, 0}}, {{2, 1e-6}, {2, 3}}));
        CHECK(intersect({{0, 0}, {4, 0}}, {{2, 1e-6}, {2, 3}}, 1e-5).kind == K::Point);

        // symmetric: swapping the segments reports the same point
        const SegmentIntersection p = intersect({{0, 2}, {2, 0}}, {{0, 0}, {2, 2}});
        CHECK(p.kind == K::Point);
        CHECK_VEC_NEAR(p.a, Vec2(1, 1), 1e-12);
    }

    void testBoxes(test::Context& ctx) {
        const Aabb s = boundsOf(Segment{{3, -1}, {-2, 4}});
        CHECK_VEC_NEAR(s.min, Vec2(-2, -1), 0.0);
        CHECK_VEC_NEAR(s.max, Vec2(3, 4), 0.0);

        const Aabb c = boundsOfCircle({10, 20}, -5);       // the sign of the radius is ignored
        CHECK_VEC_NEAR(c.min, Vec2(5, 15), 0.0);
        CHECK_VEC_NEAR(c.max, Vec2(15, 25), 0.0);

        const Vec2 pts[] = {{1, 1}, {4, -2}, {0, 3}};
        const Aabb b = boundsOf(pts);
        CHECK_VEC_NEAR(b.min, Vec2(0, -2), 0.0);
        CHECK_VEC_NEAR(b.max, Vec2(4, 3), 0.0);

        const Aabb g = inflate(b, 1.0);
        CHECK_VEC_NEAR(g.min, Vec2(-1, -3), 0.0);
        CHECK_VEC_NEAR(g.max, Vec2(5, 4), 0.0);
        CHECK(inflate(Aabb{}, 3.0).empty());
        CHECK(inflate(b, -10.0).empty());                   // shrunk past nothing

        const Aabb unit = boundsOf(Segment{{0, 0}, {1, 1}});
        CHECK(overlaps(unit, boundsOf(Segment{{0.5, 0.5}, {2, 2}})));
        CHECK(overlaps(unit, boundsOf(Segment{{1, 0}, {2, 1}})));                // touching counts
        CHECK(!overlaps(unit, boundsOf(Segment{{1.1, 0}, {2, 1}})));
        CHECK(overlaps(unit, boundsOf(Segment{{1.1, 0}, {2, 1}}), 0.2));         // within tolerance
        CHECK(!overlaps(unit, Aabb{}));
    }
}

int main() {
    return test::runTests("geometry primitives", {
        {"orientation predicate", testOrientation},
        {"closest points", testClosestPoints},
        {"segment intersection", testSegmentIntersection},
        {"bounding boxes", testBoxes},
    });
}
