// Polygon.h's centroid(), contains_point() and draw(): declared from the start but only defined
// once the geometry module existed. Kept apart from Scene.h, whose clip_fill.h defines a
// different ob::Polygon.
#include "curves/TestHarness.h"
#include "optibench/Framebuffer.h"
#include "optibench/Polygon.h"

using namespace ob;

namespace {
    void testCentroidAndContains(test::Context& ctx) {
        Polygon l;
        l.vertices = {{0, 0}, {4, 0}, {4, 1}, {1, 1}, {1, 3}, {0, 3}};
        CHECK_VEC_NEAR(l.centroid(), Vec2(1.5, 1.0), 1e-12);
        CHECK(l.contains_point({0.5, 2}));
        CHECK(!l.contains_point({2, 2}));                   // the L's notch
        CHECK(l.contains_point({4, 0.5}));                  // on the boundary
        CHECK(!l.contains_point({-1, 0.5}));

        Polygon empty;
        CHECK_VEC_NEAR(empty.centroid(), Vec2(0, 0), 0.0);
        CHECK(!empty.contains_point({0, 0}));
    }

    void testDraw(test::Context& ctx) {
        Framebuffer fb(20, 20, colors::Black);
        Polygon box;
        box.vertices = {{2, 2}, {12, 2}, {12, 8}, {2, 8}};
        box.draw(fb, colors::White);
        const auto lit = [&](int px, int py) {
            const Color c = fb.getPixel(px, py);
            return c.r == 255 && c.g == 255 && c.b == 255;
        };
        CHECK(lit(2, 2) && lit(12, 2) && lit(12, 8) && lit(2, 8));      // corners
        CHECK(lit(7, 2) && lit(12, 5) && lit(7, 8) && lit(2, 5));       // every edge, closing edge included
        CHECK(!lit(7, 5));                                              // outline only

        Framebuffer one(5, 5, colors::Black);
        Polygon dot;
        dot.vertices = {{2, 2}};
        dot.draw(one, colors::White);
        CHECK(one.getPixel(2, 2).r == 255);
    }
}

int main() {
    return test::runTests("Polygon struct methods", {
        {"centroid and contains_point", testCentroidAndContains},
        {"draw", testDraw},
    });
}
