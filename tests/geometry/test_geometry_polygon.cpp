// Polygon queries: measures, winding, simplicity, convexity, point location, normals, hull, triangulation.
#include <algorithm>
#include <cmath>
#include <vector>

#include "../curves/TestHarness.h"
#include "optibench/Polygon.h"          // the struct Polygon: accepted through .vertices
#include "optibench/geometry/PolygonOps.h"

using namespace ob;
using namespace ob::geom;

namespace {
    using Poly = std::vector<Vec2>;

    const Poly kSquare{{0, 0}, {10, 0}, {10, 10}, {0, 10}};                    // CCW, area 100
    const Poly kL{{0, 0}, {4, 0}, {4, 1}, {1, 1}, {1, 3}, {0, 3}};              // CCW, area 6
    const Poly kU{{0, 0}, {5, 0}, {5, 5}, {4, 5}, {4, 1}, {1, 1}, {1, 5}, {0, 5}};  // CCW, area 13

    Poly reversed(Poly p) { std::reverse(p.begin(), p.end()); return p; }

    void testMeasures(test::Context& ctx) {
        CHECK_NEAR(signedArea(kSquare), 100.0, 1e-12);
        CHECK_NEAR(signedArea(reversed(kSquare)), -100.0, 1e-12);
        CHECK_NEAR(area(kL), 6.0, 1e-12);
        CHECK_NEAR(area(kU), 13.0, 1e-12);
        CHECK_NEAR(area(Poly{{0, 0}, {1, 1}}), 0.0, 0.0);

        // far from the origin the fan-from-vertex-0 form keeps full precision
        Poly far;
        for (const Vec2& v : kSquare) far.push_back(v + Vec2{1e8, -3e8});
        CHECK_NEAR(area(far), 100.0, 1e-6);

        CHECK_NEAR(perimeter(kSquare), 40.0, 1e-12);
        CHECK_NEAR(perimeter(kL), 14.0, 1e-12);
        CHECK_NEAR(perimeter(Poly{{0, 0}, {3, 4}}), 10.0, 1e-12);       // there and back
        CHECK_NEAR(perimeter(Poly{{1, 1}}), 0.0, 0.0);

        CHECK_VEC_NEAR(centroid(kSquare), Vec2(5, 5), 1e-12);
        CHECK_VEC_NEAR(centroid(reversed(kSquare)), Vec2(5, 5), 1e-12);
        // L = 4x1 bar (centre (2, 0.5)) + 1x2 bar (centre (0.5, 2)) -> ((8 + 1) / 6, (2 + 4) / 6)
        CHECK_VEC_NEAR(centroid(kL), Vec2(1.5, 1.0), 1e-12);
        CHECK_VEC_NEAR(centroid(Poly{{0, 0}, {6, 0}, {0, 3}}), Vec2(2, 1), 1e-12);
        CHECK_VEC_NEAR(centroid(Poly{{0, 0}, {4, 0}, {8, 0}}), Vec2(4, 0), 1e-12);    // flat: middle
        CHECK_VEC_NEAR(centroid(Poly{{2, 3}, {2, 3}}), Vec2(2, 3), 0.0);
        CHECK_VEC_NEAR(centroid(Poly{}), Vec2(0, 0), 0.0);

        // the struct Polygon from Polygon.h goes in through its vertices
        ob::Polygon tri;
        tri.vertices = {{0, 0}, {6, 0}, {0, 3}};
        CHECK_NEAR(area(tri.vertices), tri.area(), 1e-12);
        CHECK_NEAR(perimeter(tri.vertices), tri.perimeter(), 1e-12);
    }

    void testWindingAndCleaning(test::Context& ctx) {
        CHECK(winding(kSquare) == Winding::CounterClockwise);
        CHECK(winding(reversed(kSquare)) == Winding::Clockwise);
        CHECK(winding(Poly{{0, 0}, {1, 1}, {2, 2}}) == Winding::Degenerate);
        CHECK(winding(Poly{}) == Winding::Degenerate);

        const Poly ccw = toCounterClockwise(reversed(kL));
        CHECK(winding(ccw) == Winding::CounterClockwise);
        CHECK_NEAR(signedArea(ccw), 6.0, 1e-12);

        // repeated points and straight-through vertices go; corners stay
        const Poly messy{{0, 0}, {0, 0}, {5, 0}, {10, 0}, {10, 10}, {10, 10}, {5, 10}, {0, 10}, {0, 5}, {0, 0}};
        const Poly clean = cleaned(messy);
        CHECK(clean.size() == 4);
        CHECK_NEAR(signedArea(clean), 100.0, 1e-12);

        // a run of three collinear midpoints in a row
        const Poly run{{0, 0}, {1, 0}, {2, 0}, {3, 0}, {4, 0}, {4, 4}};
        CHECK(cleaned(run).size() == 3);

        // never shrinks below a triangle
        CHECK(cleaned(Poly{{0, 0}, {1, 0}, {0, 1}}).size() == 3);
    }

    void testSimpleAndConvex(test::Context& ctx) {
        CHECK(isSimple(kSquare));
        CHECK(isSimple(kL));
        CHECK(isSimple(kU));
        CHECK(isSimple(reversed(kU)));
        CHECK(!isSimple(Poly{{0, 0}, {2, 2}, {2, 0}, {0, 2}}));                // bow tie
        CHECK(!isSimple(Poly{{0, 0}, {1, 0}, {1, 0}, {0, 1}}));                // zero-length edge
        CHECK(!isSimple(Poly{{0, 0}, {1, 1}}));
        CHECK(!isSimple(Poly{{0, 0}, {2, 0}, {4, 0}}));                        // flat
        // figure eight touching itself at (2, 2)
        CHECK(!isSimple(Poly{{0, 0}, {2, 2}, {4, 0}, {4, 4}, {2, 2}, {0, 4}}));
        // a spike that folds back along its neighbour
        CHECK(!isSimple(Poly{{0, 0}, {4, 0}, {2, 0}, {2, 3}}));

        CHECK(isConvex(kSquare));
        CHECK(isConvex(reversed(kSquare)));
        CHECK(isConvex(Poly{{0, 0}, {5, 0}, {10, 0}, {10, 10}, {0, 10}}));     // a straight-through vertex
        CHECK(!isConvex(kL));
        CHECK(!isConvex(kU));
        // pentagram: every turn the same way, but it winds twice
        const Poly star = {{0, 10}, {5.878, -8.09}, {-9.511, 3.09}, {9.511, 3.09}, {-5.878, -8.09}};
        CHECK(!isConvex(star));
        CHECK(!isConvex(Poly{{0, 0}, {1, 1}}));
    }

    void testPointLocation(test::Context& ctx) {
        CHECK(locate(kSquare, {5, 5}) == PointLocation::Inside);
        CHECK(locate(kSquare, {15, 5}) == PointLocation::Outside);
        CHECK(locate(kSquare, {10, 5}) == PointLocation::Boundary);
        CHECK(locate(kSquare, {0, 0}) == PointLocation::Boundary);
        CHECK(locate(reversed(kSquare), {5, 5}) == PointLocation::Inside);

        // inside the U's notch is outside the polygon; the arms are inside
        CHECK(locate(kU, {2.5, 3}) == PointLocation::Outside);
        CHECK(locate(kU, {0.5, 3}) == PointLocation::Inside);
        CHECK(locate(kU, {4.5, 4.9}) == PointLocation::Inside);
        CHECK(locate(kU, {2.5, 0.5}) == PointLocation::Inside);
        // on the same row as two horizontal edges: a classic ray-casting trap
        CHECK(locate(kU, {2.5, 1}) == PointLocation::Boundary);
        CHECK(locate(kU, {-1, 1}) == PointLocation::Outside);
        CHECK(locate(kU, {0.5, 1}) == PointLocation::Inside);

        CHECK(contains(kL, {0.5, 2.5}));
        CHECK(contains(kL, {4, 0.5}));
        CHECK(!contains(kL, {2, 2}));
        CHECK(!contains(Poly{}, {0, 0}));

        CHECK(windingNumber(kSquare, {5, 5}) == 1);
        CHECK(windingNumber(reversed(kSquare), {5, 5}) == -1);
        CHECK(windingNumber(kSquare, {-5, 5}) == 0);
        // the square traced twice winds twice
        Poly twice = kSquare;
        twice.insert(twice.end(), kSquare.begin(), kSquare.end());
        CHECK(windingNumber(twice, {5, 5}) == 2);

        CHECK_NEAR(distanceToBoundary(kSquare, {5, 3}), 3.0, 1e-12);
        CHECK_NEAR(signedDistance(kSquare, {5, 3}), -3.0, 1e-12);
        CHECK_NEAR(signedDistance(kSquare, {13, 14}), 5.0, 1e-12);
        CHECK_NEAR(signedDistance(kU, {2.5, 3}), 1.5, 1e-12);       // in the notch, 1.5 from either arm
    }

    void testEdgesAndNormals(test::Context& ctx) {
        CHECK(edgeCount(kSquare) == 4);
        CHECK(edgeCount(Poly{{0, 0}, {1, 0}}) == 1);
        CHECK(edgeCount(Poly{{0, 0}}) == 0);
        const Segment last = edge(kSquare, 3);
        CHECK_VEC_NEAR(last.a, Vec2(0, 10), 0.0);
        CHECK_VEC_NEAR(last.b, Vec2(0, 0), 0.0);

        // square: bottom edge faces -y, right edge +x, either winding
        CHECK_VEC_NEAR(outwardNormal(kSquare, 0), Vec2(0, -1), 1e-12);
        CHECK_VEC_NEAR(outwardNormal(kSquare, 1), Vec2(1, 0), 1e-12);
        const Poly cw = reversed(kSquare);              // edge 0 of cw is (0,10) -> (10,10): the top
        CHECK_VEC_NEAR(outwardNormal(cw, 0), Vec2(0, 1), 1e-12);
        CHECK_VEC_NEAR(outwardNormal(Poly{{0, 0}, {1, 1}}, 0), Vec2(0, 0), 0.0);    // degenerate polygon

        // every edge of concave polygons, both windings: a step along the normal leaves the polygon,
        // a step against it enters (the centroid trick would get the U's inner edges wrong)
        for (const Poly& p : {kL, kU, reversed(kL), reversed(kU)}) {
            for (std::size_t i = 0; i < p.size(); ++i) {
                const Segment e = edge(p, i);
                const Vec2 mid = (e.a + e.b) * 0.5;
                const Vec2 n = outwardNormal(p, i);
                CHECK_NEAR(n.length(), 1.0, 1e-12);
                CHECK(locate(p, mid + n * 1e-3) == PointLocation::Outside);
                CHECK(locate(p, mid - n * 1e-3) == PointLocation::Inside);
            }
        }
        // the U's inner right wall (edge 3: (4,5) -> (4,1)) faces left, into the notch
        CHECK_VEC_NEAR(outwardNormal(kU, 3), Vec2(-1, 0), 1e-12);

        CHECK(nearestEdge(kSquare, {5, -1}) == 0);
        CHECK(nearestEdge(kSquare, {11, 5}) == 1);
        CHECK(nearestEdge(kU, {2.5, 1.2}) == 4);        // the bottom of the notch
    }

    void testConvexHull(test::Context& ctx) {
        // corners, interior points, points on the edges and duplicates
        const Poly cloud{{0, 0}, {4, 0}, {4, 4}, {0, 4}, {2, 2}, {1, 3}, {2, 0}, {4, 2}, {0, 0}, {3, 1}};
        const Poly hull = convexHull(cloud);
        CHECK(hull.size() == 4);
        CHECK(winding(hull) == Winding::CounterClockwise);
        CHECK_NEAR(area(hull), 16.0, 1e-12);
        CHECK(isConvex(hull));
        for (const Vec2& p : cloud) CHECK(contains(hull, p));

        // hull of a concave polygon fills in its notch
        CHECK_NEAR(area(convexHull(kU)), 25.0, 1e-12);

        // degenerate inputs
        CHECK(convexHull(Poly{}).empty());
        CHECK(convexHull(Poly{{1, 1}, {1, 1}}).size() == 1);
        const Poly line = convexHull(Poly{{3, 3}, {0, 0}, {1, 1}, {2, 2}});
        CHECK(line.size() == 2);
        CHECK_VEC_NEAR(line[0], Vec2(0, 0), 0.0);
        CHECK_VEC_NEAR(line[1], Vec2(3, 3), 0.0);

        // a regular polygon is its own hull
        const Poly hex = regularPolygon({5, 5}, 3, 6);
        CHECK(convexHull(hex).size() == 6);
    }

    // triangles must be CCW, inside the polygon, and add up to its area
    void checkTriangulation(test::Context& ctx, const Poly& p) {
        const std::vector<Triangle> tris = triangulate(p);
        CHECK(tris.size() == cleaned(p).size() - 2);
        double sum = 0.0;
        for (const Triangle& t : tris) {
            const Poly tri{p[t[0]], p[t[1]], p[t[2]]};
            CHECK(signedArea(tri) > 0.0);
            CHECK(locate(p, centroid(tri)) == PointLocation::Inside);
            sum += signedArea(tri);
        }
        CHECK_NEAR(sum, area(p), 1e-9);
    }

    void testTriangulation(test::Context& ctx) {
        checkTriangulation(ctx, kSquare);
        checkTriangulation(ctx, kL);
        checkTriangulation(ctx, kU);
        checkTriangulation(ctx, reversed(kU));
        checkTriangulation(ctx, regularPolygon({0, 0}, 7, 9));

        // a comb: many reflex vertices
        // a comb with five teeth of width 1 on a 9x1 base: area 9 + 5 * 4 = 29
        const Poly comb{{0, 0}, {9, 0}, {9, 5}, {8, 5}, {8, 1}, {7, 1}, {7, 5}, {6, 5}, {6, 1}, {5, 1},
                        {5, 5}, {4, 5}, {4, 1}, {3, 1}, {3, 5}, {2, 5}, {2, 1}, {1, 1}, {1, 5}, {0, 5}};
        CHECK(isSimple(comb));
        CHECK_NEAR(signedArea(comb), 29.0, 1e-12);
        checkTriangulation(ctx, comb);

        // straight-through vertices are skipped, not turned into slivers
        const Poly straight{{0, 0}, {5, 0}, {10, 0}, {10, 10}, {0, 10}};
        CHECK(triangulate(straight).size() == 2);

        // not simple: no triangulation
        CHECK(triangulate(Poly{{0, 0}, {2, 2}, {2, 0}, {0, 2}}).empty());
        CHECK(triangulate(Poly{{0, 0}, {1, 1}}).empty());
    }

    void testConstruction(test::Context& ctx) {
        // regular n-gon area = n r^2 sin(2 pi / n) / 2
        const Poly hex = regularPolygon({1, 2}, 4, 6, 0.3);
        CHECK(hex.size() == 6);
        CHECK_NEAR(area(hex), 6 * 16 * std::sin(2 * kPi / 6) / 2, 1e-9);
        CHECK(winding(hex) == Winding::CounterClockwise);
        CHECK_VEC_NEAR(centroid(hex), Vec2(1, 2), 1e-12);
        CHECK(regularPolygon({0, 0}, 1, 2).empty());

        const Poly rect = rectangle(boundsOf(kL));
        CHECK(rect.size() == 4);
        CHECK_NEAR(signedArea(rect), 12.0, 1e-12);
        CHECK(rectangle(Aabb{}).empty());

        // transforms: area scales by |sx sy|, a mirror flips the winding
        const Transform2D t{.position = {50, -20}, .rotation = 0.7, .scale = {2, 3}};
        const Poly moved = transformed(kL, t);
        CHECK_NEAR(signedArea(moved), 6.0 * 6.0, 1e-9);
        CHECK_VEC_NEAR(centroid(moved), t.toWorld(centroid(kL)), 1e-9);
        const Poly mirrored = transformed(kL, Transform2D{.scale = {-1, 1}});
        CHECK(winding(mirrored) == Winding::Clockwise);
        CHECK(isSimple(mirrored));
    }
}

int main() {
    return test::runTests("geometry polygon queries", {
        {"area, perimeter, centroid", testMeasures},
        {"winding and cleaning", testWindingAndCleaning},
        {"simple and convex", testSimpleAndConvex},
        {"point location", testPointLocation},
        {"edges and outward normals", testEdgesAndNormals},
        {"convex hull", testConvexHull},
        {"triangulation", testTriangulation},
        {"construction and transforms", testConstruction},
    });
}
