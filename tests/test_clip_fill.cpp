#include <cmath>
#include <cstdio>
#include "optibench/clip_fill.h"


// test definition
#define CHECK(cond) do { if (!(cond)) { std::printf("FAIL %s:%d  %s\n", __FILE__, __LINE__, #cond); return 1; } } while (0)


using namespace ob;


/**
 * helper method to check if a specific pixel has been drawn on
 * @param fb Framebuffer
 * @param x int
 * @param y int
 * @return bool
 */
static bool lit(const Framebuffer& fb, int x, int y) {
    const Color p = fb.getPixel(x, y);
    return p.r != colors::Black.r || p.g != colors::Black.g || p.b != colors::Black.b;
}


/**
 * helper method to check if a specific pixel holds an exact colour
 * @param fb Framebuffer
 * @param x int
 * @param y int
 * @param c Color
 * @return bool
 */
static bool is(const Framebuffer& fb, int x, int y, Color c) {
    const Color p = fb.getPixel(x, y);
    return p.r == c.r && p.g == c.g && p.b == c.b;
}


/**
 * helper method counting how many pixels on the canvas have been drawn on
 * @param fb Framebuffer
 * @return int
 */
static int countLit(const Framebuffer& fb) {
    int n = 0;
    for (int y = 0; y < fb.height(); ++y)
        for (int x = 0; x < fb.width(); ++x)
            if (lit(fb, x, y)) ++n;
    return n;
}


/**
 * helper method comparing two decimals
 * @param a double
 * @param b double
 * @return bool
 */
static bool near(double a, double b) { return std::fabs(a - b) < 1e-9; }


int main() {

    // test validating that a filled square covers exactly its area; no off-by-one on the edges
    {
        Framebuffer fb(50, 50, colors::Black);

        // filling a 10x10 square
        fillPolygon(fb, {{10, 10}, {20, 10}, {20, 20}, {10, 20}}, colors::White);

        // test validation: exactly 100 pixels painted
        CHECK(countLit(fb) == 100);

        // test validation: the corners inside are painted; the pixels just past the edges are not
        CHECK(lit(fb, 10, 10)); CHECK(lit(fb, 19, 19));
        CHECK(!lit(fb, 20, 15)); CHECK(!lit(fb, 15, 20)); CHECK(!lit(fb, 9, 15));
    }


    // test validating that a filled triangle (a prism) covers its inside only
    {
        Framebuffer fb(100, 100, colors::Black);

        fillPolygon(fb, {{50, 10}, {90, 80}, {10, 80}}, colors::White);

        // test validation: the centroid is painted
        CHECK(lit(fb, 50, 56));

        // test validation: points beside the slanted edges are not painted
        CHECK(!lit(fb, 15, 20)); CHECK(!lit(fb, 85, 20)); CHECK(!lit(fb, 50, 90));
    }


    // test validating the even-odd rule on a concave (U-shaped) polygon
    {
        Framebuffer fb(50, 50, colors::Black);

        // a U: two 10px-wide arms joined along the top, with a notch between them
        fillPolygon(fb, {{0, 0}, {30, 0}, {30, 30}, {20, 30}, {20, 10}, {10, 10}, {10, 30}, {0, 30}}, colors::White);

        // test validation: the arms are painted
        CHECK(lit(fb, 5, 20)); CHECK(lit(fb, 25, 20));

        // test validation: the notch is left empty
        CHECK(!lit(fb, 15, 20));

        // test validation: the painted area matches the polygon's area
        CHECK(countLit(fb) == 700);
    }


    // test validating that a polygon reaching off the canvas is filled without errors
    {
        Framebuffer fb(20, 20, colors::Black);

        fillPolygon(fb, {{-10, -10}, {30, -10}, {30, 30}, {-10, 30}}, colors::White);

        // test validation: the whole canvas is covered
        CHECK(countLit(fb) == 400);
    }


    // test validating that flood fill stays inside a closed outline
    {
        Framebuffer fb(40, 40, colors::Black);

        // a red outline with a black inside
        drawPolygon(fb, {{5, 5}, {30, 5}, {30, 30}, {5, 30}}, colors::Red);

        // repainting the black region around the seed
        floodFill(fb, 15, 15, colors::Blue);

        // test validation: the inside is painted; the outline and the outside are untouched
        CHECK(is(fb, 15, 15, colors::Blue)); CHECK(is(fb, 6, 29, colors::Blue));
        CHECK(is(fb, 5, 5, colors::Red)); CHECK(is(fb, 30, 30, colors::Red));
        CHECK(!lit(fb, 35, 35)); CHECK(!lit(fb, 2, 2));
    }


    // test validating that flood fill on a black canvas stops at the canvas edges
    {
        // getPixel reports out-of-bounds pixels as Black, so this would never end without bounds checks
        Framebuffer fb(20, 20, colors::Black);

        floodFill(fb, 5, 5, colors::White);

        // test validation: every pixel is painted exactly once
        CHECK(countLit(fb) == 400);

        // test validation: filling with the colour already there changes nothing
        floodFill(fb, 5, 5, colors::White);
        CHECK(countLit(fb) == 400);
    }


    // test validating that boundary fill stops at the boundary colour
    {
        Framebuffer fb(40, 40, colors::Black);

        drawPolygon(fb, {{5, 5}, {30, 5}, {30, 30}, {5, 30}}, colors::Red);

        // a stray yellow pixel inside: boundary fill paints over it, unlike flood fill
        fb.setPixel(20, 20, colors::Yellow);

        boundaryFill(fb, 15, 15, colors::Blue, colors::Red);

        // test validation: the inside, including the stray pixel, is painted
        CHECK(is(fb, 15, 15, colors::Blue)); CHECK(is(fb, 20, 20, colors::Blue));

        // test validation: the boundary and the outside are untouched
        CHECK(is(fb, 30, 15, colors::Red)); CHECK(!lit(fb, 35, 35));
    }


    // tests validating Cohen-Sutherland line clipping
    {
        const ClipRect r{0, 0, 100, 100};

        // test validation: a segment fully inside is unchanged
        Vec2 a{10, 10}, b{90, 50};
        CHECK(clipLineCohenSutherland(a, b, r));
        CHECK(near(a.x, 10) && near(a.y, 10) && near(b.x, 90) && near(b.y, 50));

        // test validation: a segment fully outside (same side) is rejected
        Vec2 c{-20, 10}, d{-5, 90};
        CHECK(!clipLineCohenSutherland(c, d, r));

        // test validation: a segment outside two different sides, missing the corner, is rejected
        Vec2 e{-10, 5}, f{5, -10};
        CHECK(!clipLineCohenSutherland(e, f, r));

        // test validation: a segment crossing the left edge ends exactly on it
        Vec2 g{-50, 50}, h{50, 50};
        CHECK(clipLineCohenSutherland(g, h, r));
        CHECK(near(g.x, 0) && near(g.y, 50)); CHECK(near(h.x, 50));

        // test validation: a diagonal crossing the whole window is trimmed to its corners
        Vec2 i{-50, -50}, j{150, 150};
        CHECK(clipLineCohenSutherland(i, j, r));
        CHECK(near(i.x, 0) && near(i.y, 0) && near(j.x, 100) && near(j.y, 100));
    }


    // tests validating Sutherland-Hodgman polygon clipping
    {
        const ClipRect r{0, 0, 100, 100};

        // test validation: the shoelace area of a 100x100 square
        CHECK(near(std::fabs(polygonArea({{0, 0}, {100, 0}, {100, 100}, {0, 100}})), 10000));

        // test validation: a square hanging half off the window keeps half its area
        const Polygon half = clipPolygonToRect({{50, 0}, {150, 0}, {150, 100}, {50, 100}}, r);
        CHECK(near(std::fabs(polygonArea(half)), 5000));

        // test validation: a square overlapping a corner keeps only the overlap
        const Polygon corner = clipPolygonToRect({{50, 50}, {150, 50}, {150, 150}, {50, 150}}, r);
        CHECK(near(std::fabs(polygonArea(corner)), 2500));

        // test validation: a polygon fully outside clips to nothing
        CHECK(clipPolygonToRect({{200, 200}, {300, 200}, {250, 300}}, r).empty());

        // test validation: a triangle inside a convex clip polygon comes back unchanged
        const Polygon tri{{20, 20}, {40, 20}, {30, 40}};
        const Polygon ccw{{0, 0}, {100, 0}, {100, 100}, {0, 100}};
        CHECK(near(polygonArea(clipPolygonConvex(tri, ccw)), polygonArea(tri)));

        // test validation: the clip polygon's vertex order does not matter
        const Polygon cw{{0, 0}, {0, 100}, {100, 100}, {100, 0}};
        CHECK(near(polygonArea(clipPolygonConvex(tri, cw)), polygonArea(tri)));

        // test validation: a beam (rectangle) through a triangular prism keeps only the part inside the glass
        const Polygon prism{{0, 0}, {100, 0}, {0, 100}};
        const Polygon beam{{-10, 40}, {110, 40}, {110, 60}, {-10, 60}};

        // the glass spans x = 0 .. (100 - y), so the strip 40 <= y <= 60 has area = integral of (100 - y) dy = 1000
        CHECK(near(std::fabs(polygonArea(clipPolygonConvex(beam, prism))), 1000));
    }


    // message for successful tests
    std::puts("test_clip_fill: PASS");
    return 0;
}
