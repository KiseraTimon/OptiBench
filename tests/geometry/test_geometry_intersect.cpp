// Ray queries: ray vs box, ray vs polygon (concave, vertices, grazing), convex-region spans.
#include <cmath>
#include <limits>
#include <vector>

#include "../curves/TestHarness.h"
#include "optibench/circle_ellipse.h"   // rayCircleIntersect, to cross-check lineDisc
#include "optibench/geometry/Intersect.h"

using namespace ob;
using namespace ob::geom;

namespace {
    using Poly = std::vector<Vec2>;
    const Poly kSquare{{0, 0}, {10, 0}, {10, 10}, {0, 10}};
    const Poly kU{{0, 0}, {5, 0}, {5, 5}, {4, 5}, {4, 1}, {1, 1}, {1, 5}, {0, 5}};

    void testRayBox(test::Context& ctx) {
        const Aabb box = boundsOf(Segment{{0, 0}, {10, 10}});

        auto r = rayAabb({{-5, 5}, {1, 0}}, box);
        CHECK(r.has_value());
        if (r) { CHECK_NEAR(r->tNear, 5.0, 1e-12); CHECK_NEAR(r->tFar, 15.0, 1e-12); }

        // t is a distance even for a long direction vector
        r = rayAabb({{-5, 5}, {100, 0}}, box);
        CHECK(r && std::fabs(r->tNear - 5.0) < 1e-12);

        // starting inside: tNear = 0
        r = rayAabb({{5, 5}, {0, -1}}, box);
        CHECK(r && r->tNear == 0.0 && std::fabs(r->tFar - 5.0) < 1e-12);

        // diagonal through the corners
        r = rayAabb({{-1, -1}, {1, 1}}, box);
        CHECK(r && std::fabs(r->tNear - std::sqrt(2.0)) < 1e-12 && std::fabs(r->tFar - 11 * std::sqrt(2.0)) < 1e-12);

        CHECK(!rayAabb({{-5, 5}, {-1, 0}}, box));          // pointing away
        CHECK(!rayAabb({{-5, 15}, {1, 0}}, box));          // parallel, outside the y slab
        CHECK(rayAabb({{-5, 10}, {1, 0}}, box));           // parallel along the top face
        CHECK(!rayAabb({{-5, 5}, {0, 0}}, box));           // no direction
        CHECK(!rayAabb({{-5, 5}, {1, 0}}, Aabb{}));        // empty box
    }

    void testRaySquare(test::Context& ctx) {
        const auto hits = rayPolygonAll({{-5, 4}, {1, 0}}, kSquare);
        CHECK(hits.size() == 2);
        if (hits.size() == 2) {
            CHECK_NEAR(hits[0].t, 5.0, 1e-12);
            CHECK_VEC_NEAR(hits[0].point, Vec2(0, 4), 1e-12);
            CHECK_VEC_NEAR(hits[0].normal, Vec2(-1, 0), 1e-12);
            CHECK(hits[0].entering);
            CHECK(hits[0].edge == 3);
            CHECK_NEAR(hits[1].t, 15.0, 1e-12);
            CHECK_VEC_NEAR(hits[1].normal, Vec2(1, 0), 1e-12);
            CHECK(!hits[1].entering);
        }

        // the same answer for the reversed winding
        const Poly cw{{0, 10}, {10, 10}, {10, 0}, {0, 0}};
        const auto cwHits = rayPolygonAll({{-5, 4}, {1, 0}}, cw);
        CHECK(cwHits.size() == 2 && cwHits[0].entering && !cwHits[1].entering);
        if (!cwHits.empty()) CHECK_VEC_NEAR(cwHits[0].normal, Vec2(-1, 0), 1e-12);

        // starting inside: one exit hit
        const Hit out = rayPolygon({{5, 5}, {0, 1}}, kSquare);
        CHECK(out.hit);
        CHECK_NEAR(out.t, 5.0, 1e-12);
        CHECK_VEC_NEAR(out.normal, Vec2(0, 1), 1e-12);

        // starting exactly on the boundary: that crossing is skipped (t < tMin)
        const auto fromEdge = rayPolygonAll({{0, 5}, {1, 0}}, kSquare);
        CHECK(fromEdge.size() == 1);
        if (!fromEdge.empty()) CHECK_NEAR(fromEdge[0].t, 10.0, 1e-12);

        CHECK(!rayPolygon({{-5, 4}, {-1, 0}}, kSquare).hit);
        CHECK(!rayPolygon({{-5, 4}, {0, 0}}, kSquare).hit);
        CHECK(!rayPolygon({{-5, 4}, {1, 0}}, Poly{{0, 0}, {1, 1}}).hit);
    }

    void testRayConcave(test::Context& ctx) {
        // through both arms of the U: in, out, in, out
        const auto hits = rayPolygonAll({{-1, 3}, {1, 0}}, kU);
        CHECK(hits.size() == 4);
        const double ts[] = {1, 2, 5, 6};
        const bool in[] = {true, false, true, false};
        const double nx[] = {-1, 1, -1, 1};
        for (std::size_t i = 0; i < hits.size() && i < 4; ++i) {
            CHECK_NEAR(hits[i].t, ts[i], 1e-12);
            CHECK(hits[i].entering == in[i]);
            CHECK_VEC_NEAR(hits[i].normal, Vec2(nx[i], 0), 1e-12);
        }

        // from inside the notch the nearest hit is the inner wall, whose normal faces the notch
        const Hit wall = rayPolygon({{2.5, 3}, {1, 0}}, kU);
        CHECK(wall.hit);
        CHECK_NEAR(wall.t, 1.5, 1e-12);
        CHECK_VEC_NEAR(wall.normal, Vec2(-1, 0), 1e-12);

        // straight down the notch: the first thing hit is the notch floor at y = 1, facing up (+y)
        const Hit floor = rayPolygon({{2.5, 9}, {0, -1}}, kU);
        CHECK(floor.hit);
        CHECK_NEAR(floor.t, 8.0, 1e-12);
        CHECK_VEC_NEAR(floor.normal, Vec2(0, 1), 1e-12);
    }

    void testRayVertices(test::Context& ctx) {
        // a diamond crossed exactly through two opposite vertices: one hit at each, averaged normals
        const Poly diamond{{0, -1}, {1, 0}, {0, 1}, {-1, 0}};
        const auto hits = rayPolygonAll({{-3, 0}, {1, 0}}, diamond);
        CHECK(hits.size() == 2);
        if (hits.size() == 2) {
            CHECK_NEAR(hits[0].t, 2.0, 1e-12);
            CHECK_VEC_NEAR(hits[0].point, Vec2(-1, 0), 1e-12);
            CHECK_VEC_NEAR(hits[0].normal, Vec2(-1, 0), 1e-12);
            CHECK(hits[0].entering);
            CHECK_NEAR(hits[1].t, 4.0, 1e-12);
            CHECK_VEC_NEAR(hits[1].normal, Vec2(1, 0), 1e-12);
            CHECK(!hits[1].entering);
        }

        // grazing a corner from outside: no hit at all
        CHECK(rayPolygonAll({{-1, 1}, {1, -1}}, Poly{{0, 0}, {2, 0}, {2, 2}, {0, 2}}).empty());
        CHECK(rayPolygonAll({{-3, 1}, {1, 0}}, diamond).empty());       // touches the top vertex only

        // grazing the U's reflex corner (1, 1) from INSIDE: the ray runs from the left arm into the
        // bottom bar without ever leaving the polygon, so the corner is not a crossing. The only
        // crossing is the exit through the bottom edge at (2, 0).
        const auto inner = rayPolygonAll({{0.5, 1.5}, {1, -1}}, kU);
        CHECK(inner.size() == 1);
        if (!inner.empty()) {
            CHECK_VEC_NEAR(inner[0].point, Vec2(2, 0), 1e-12);
            CHECK(!inner[0].entering);
        }

        // whereas through the same corner at a steeper angle the ray really leaves (into the notch)
        const auto through = rayPolygonAll({{0.5, -1}, {0.25, 1}}, kU);
        CHECK(through.size() == 2);
        if (through.size() == 2) {
            CHECK(through[0].entering);
            CHECK_VEC_NEAR(through[1].point, Vec2(1, 1), 1e-12);
            CHECK(!through[1].entering);
        }

        // running along an edge: the two corners where it meets the neighbouring edges
        const auto along = rayPolygonAll({{-1, 0}, {1, 0}}, kSquare);
        CHECK(along.size() == 2);
        if (along.size() == 2) {
            CHECK_VEC_NEAR(along[0].point, Vec2(0, 0), 1e-12);
            CHECK(along[0].entering);
            CHECK_VEC_NEAR(along[1].point, Vec2(10, 0), 1e-12);
            CHECK(!along[1].entering);
        }
    }

    void testSpans(test::Context& ctx) {
        // the disc span agrees with the circle module's ray-circle intersection
        const Ray ray{{-20, 3}, {2, 0.5}};
        const auto disc = lineDisc(ray, {1, 7}, 6);
        const Hit ref = rayCircleIntersect(Ray{ray.origin, ray.direction.normalized()}, {1, 7}, 6);
        CHECK(disc.has_value() && ref.hit);
        if (disc && ref.hit) {
            CHECK_NEAR(disc->tIn, ref.t, 1e-9);
            CHECK_VEC_NEAR(disc->nIn, ref.normal, 1e-9);
        }
        CHECK(!lineDisc({{-20, 6}, {1, 0}}, {0, 0}, 6));     // tangent: no interior
        CHECK(!lineDisc({{-20, 7}, {1, 0}}, {0, 0}, 6));
        CHECK(!lineDisc({{-20, 0}, {1, 0}}, {0, 0}, 0));

        // a line through a disc centre far from the origin keeps both roots accurate
        const auto farDisc = lineDisc({{1e6 - 50, 0}, {1, 0}}, {1e6, 0}, 1);
        CHECK(farDisc && std::fabs(farDisc->tIn - 49.0) < 1e-9 && std::fabs(farDisc->tOut - 51.0) < 1e-9);

        // half-plane x <= 4 (outward +x)
        const auto hp = lineHalfPlane({{0, 0}, {1, 0}}, {4, 0}, {1, 0});
        CHECK(hp && std::isinf(hp->tIn) && hp->tIn < 0 && std::fabs(hp->tOut - 4.0) < 1e-12);
        const auto hpIn = lineHalfPlane({{10, 0}, {-2, 0}}, {4, 0}, {1, 0});
        CHECK(hpIn && std::fabs(hpIn->tIn - 6.0) < 1e-12 && std::isinf(hpIn->tOut));
        CHECK(!lineHalfPlane({{10, 0}, {0, 1}}, {4, 0}, {1, 0}));                  // parallel, outside
        const auto inside = lineHalfPlane({{0, 0}, {0, 1}}, {4, 0}, {1, 0});       // parallel, inside
        CHECK(inside && std::isinf(inside->tIn) && std::isinf(inside->tOut));

        // a slab 0 <= x <= 4 as two half-planes, entered at 2 and left at 6 along x from -2
        const Ray r{{-2, 1}, {1, 0}};
        const auto slab = intersectSpans(*lineHalfPlane(r, {0, 0}, {-1, 0}), *lineHalfPlane(r, {4, 0}, {1, 0}));
        CHECK(slab.has_value());
        if (slab) {
            CHECK_NEAR(slab->tIn, 2.0, 1e-12);
            CHECK_NEAR(slab->tOut, 6.0, 1e-12);
            const Hit h = firstHit(r, *slab);
            CHECK(h.hit && std::fabs(h.t - 2.0) < 1e-12);
            CHECK_VEC_NEAR(h.normal, Vec2(-1, 0), 1e-12);

            // from inside the slab the first hit is the exit
            const Hit e = firstHit(Ray{{1, 1}, {1, 0}},
                                   *intersectSpans(*lineHalfPlane({{1, 1}, {1, 0}}, {0, 0}, {-1, 0}),
                                                   *lineHalfPlane({{1, 1}, {1, 0}}, {4, 0}, {1, 0})));
            CHECK(e.hit && std::fabs(e.t - 3.0) < 1e-12);
            CHECK_VEC_NEAR(e.normal, Vec2(1, 0), 1e-12);
        }

        // disjoint spans
        LineSpan a{0, 1, {}, {}}, b{2, 3, {}, {}};
        CHECK(!intersectSpans(a, b));
        // a span entirely behind the ray
        CHECK(!firstHit(Ray{{0, 0}, {1, 0}}, LineSpan{-5, -1, {1, 0}, {1, 0}}).hit);
    }
}

int main() {
    return test::runTests("geometry ray queries", {
        {"ray vs box", testRayBox},
        {"ray vs square", testRaySquare},
        {"ray vs concave polygon", testRayConcave},
        {"vertices, grazing, edges", testRayVertices},
        {"convex spans", testSpans},
    });
}
