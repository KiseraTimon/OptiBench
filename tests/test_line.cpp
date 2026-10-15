#include <algorithm>
#include <cmath>
#include <cstdio>
#include "../include/optibench/Line.h"


// test definition
#define CHECK(cond) do { if (!(cond)) { std::printf("FAIL %s:%d  %s\n", __FILE__, __LINE__, #cond); return 1; } } while (0)


namespace {
    // checks two decimal numbers are equal (allowing tiny rounding errors)
    bool approx(double a, double b) { return std::fabs(a - b) < 1e-6; }

    // checks two colors are the same
    bool sameColor(ob::Color a, ob::Color b) { return a.r == b.r && a.g == b.g && a.b == b.b; }

    // counts the pixels that are not the background color
    int countLit(const ob::Framebuffer& fb, ob::Color bg) {
        int n = 0;
        for (int y = 0; y < fb.height(); ++y)
            for (int x = 0; x < fb.width(); ++x)
                if (!sameColor(fb.getPixel(x, y), bg)) ++n;
        return n;
    }

    // checks two canvases have exactly the same pixels
    bool samePixels(const ob::Framebuffer& f1, const ob::Framebuffer& f2) {
        for (int y = 0; y < f1.height(); ++y)
            for (int x = 0; x < f1.width(); ++x)
                if (!sameColor(f1.getPixel(x, y), f2.getPixel(x, y))) return false;
        return true;
    }
}


int main() {
    using namespace ob;

    // Bresenham and DDA take the same inputs, so both are tested together
    using DrawFn = void (*)(Framebuffer&, int, int, int, int, Color);
    const DrawFn algorithms[] = {drawLineBresenham, drawLineDDA};

    // end points in every direction; every line starts from the centre (10, 10)
    const int ends[][2] = {{17, 13}, {13, 17}, {7, 17}, {3, 13}, {3, 7}, {7, 3}, {13, 3}, {17, 7},
                           {18, 10}, {10, 18}, {2, 10}, {10, 2}, {16, 16}};

    for (DrawFn draw : algorithms) {
        for (const auto& e : ends) {
            // framebuffer instance
            Framebuffer fb(21, 21, colors::Black);
            draw(fb, 10, 10, e[0], e[1], colors::White);

            // testing both end points are drawn
            CHECK(sameColor(fb.getPixel(10, 10), colors::White));
            CHECK(sameColor(fb.getPixel(e[0], e[1]), colors::White));

            // testing there are no gaps or extra pixels; one pixel per step along the longer side
            const int steps = std::max(std::abs(e[0] - 10), std::abs(e[1] - 10));
            CHECK(countLit(fb, colors::Black) == steps + 1);

            // testing a line drawn backwards gives the same pixels
            Framebuffer rev(21, 21, colors::Black);
            draw(rev, e[0], e[1], 10, 10, colors::White);
            CHECK(samePixels(fb, rev));
        }

        // testing a line with no length draws a single pixel
        Framebuffer dot(5, 5, colors::Black);
        draw(dot, 2, 2, 2, 2, colors::Red);
        CHECK(countLit(dot, colors::Black) == 1);

        // testing off-screen lines do not induce a crash and still draw the visible part
        Framebuffer off(10, 10, colors::Black);
        draw(off, -50, -50, 100, 100, colors::Red);
        CHECK(sameColor(off.getPixel(5, 5), colors::Red));
    }

    // testing 2D points are rounded to the nearest pixel
    {
        Framebuffer fb(5, 5, colors::Black);
        drawLine(fb, Vec2{1.4, 1.6}, Vec2{1.4, 1.6}, colors::White);
        CHECK(sameColor(fb.getPixel(1, 2), colors::White));
    }

    // testing a smooth line lying exactly on a pixel row fully colors that row only
    {
        Framebuffer fb(20, 10, colors::Black);
        drawLineWu(fb, Vec2{2, 5}, Vec2{12, 5}, colors::White);
        CHECK(sameColor(fb.getPixel(7, 5), colors::White));
        CHECK(sameColor(fb.getPixel(7, 6), colors::Black));
    }

    // testing a smooth line halfway between two rows shares its color between both
    {
        Framebuffer fb(20, 10, colors::Black);
        drawLineWu(fb, Vec2{2, 5.5}, Vec2{12, 5.5}, colors::White);
        CHECK(fb.getPixel(7, 5).r > 100 && fb.getPixel(7, 5).r < 155);
        CHECK(fb.getPixel(7, 6).r > 100 && fb.getPixel(7, 6).r < 155);
    }

    // testing a 5 pixel thick line covers 2 pixels on each side and no more
    {
        Framebuffer fb(41, 21, colors::Black);
        drawThickLine(fb, Vec2{10, 10}, Vec2{30, 10}, 5, colors::Green);
        CHECK(sameColor(fb.getPixel(20, 12), colors::Green));
        CHECK(sameColor(fb.getPixel(20, 8), colors::Green));
        CHECK(sameColor(fb.getPixel(20, 13), colors::Black));
    }

    // testing a dashed line switches between drawn and empty parts
    {
        Framebuffer fb(20, 10, colors::Black);
        drawDashedLine(fb, Vec2{0, 5}, Vec2{19, 5}, 4, 4, colors::Yellow);
        CHECK(sameColor(fb.getPixel(2, 5), colors::Yellow));
        CHECK(sameColor(fb.getPixel(6, 5), colors::Black));
        CHECK(sameColor(fb.getPixel(10, 5), colors::Yellow));
    }

    // testing length, midpoint and normal of a segment
    {
        const Segment s{Vec2{0, 0}, Vec2{3, 4}};
        CHECK(approx(length(s), 5.0));
        CHECK(approx(midpoint(s).x, 1.5) && approx(midpoint(s).y, 2.0));

        const Vec2 n = normal(Segment{Vec2{0, 0}, Vec2{1, 0}});
        CHECK(approx(n.x, 0.0) && approx(n.y, 1.0));
    }

    // testing distance from a point beside the segment and from a point past its end
    {
        const Segment s{Vec2{0, 0}, Vec2{10, 0}};
        CHECK(approx(distanceToSegment(Vec2{5, 5}, s), 5.0));
        CHECK(approx(distanceToSegment(Vec2{-3, 4}, s), 5.0));
    }

    // testing two crossing segments meet at the right point
    {
        Vec2 p;
        CHECK(intersectSegments(Segment{Vec2{0, 0}, Vec2{10, 10}}, Segment{Vec2{0, 10}, Vec2{10, 0}}, p));
        CHECK(approx(p.x, 5.0) && approx(p.y, 5.0));

        // testing parallel segments do not cross
        CHECK(!intersectSegments(Segment{Vec2{0, 0}, Vec2{10, 0}}, Segment{Vec2{0, 5}, Vec2{10, 5}}, p));

        // testing segments that would only cross if they were longer
        CHECK(!intersectSegments(Segment{Vec2{0, 0}, Vec2{1, 1}}, Segment{Vec2{5, 0}, Vec2{4, 1}}, p));
    }

    // testing a laser ray hitting a vertical mirror
    {
        const Segment mirror{Vec2{10, 0}, Vec2{10, 10}};

        const Hit h = intersectRaySegment(Ray{Vec2{0, 5}, Vec2{2, 0}}, mirror);
        CHECK(h.hit);
        CHECK(approx(h.t, 10.0));
        CHECK(approx(h.point.x, 10.0) && approx(h.point.y, 5.0));
        CHECK(approx(h.normal.x, -1.0) && approx(h.normal.y, 0.0));    // validation; normal faces the laser

        // testing a mirror behind the laser is not hit
        CHECK(!intersectRaySegment(Ray{Vec2{20, 5}, Vec2{1, 0}}, mirror).hit);

        // testing a laser running alongside the mirror is not hit
        CHECK(!intersectRaySegment(Ray{Vec2{0, 5}, Vec2{0, 1}}, mirror).hit);
    }

    // testing file saves; a laser bouncing between two mirrors
    {
        Framebuffer fb(40, 20);

        // mirrors
        drawLineBresenham(fb, 0, 2, 39, 2, colors::White);
        drawLineBresenham(fb, 0, 17, 39, 17, colors::White);

        // laser beam
        drawLineBresenham(fb, 0, 10, 7, 3, colors::Red);
        drawLineBresenham(fb, 7, 3, 20, 16, colors::Red);
        drawLineBresenham(fb, 20, 16, 33, 3, colors::Red);
        drawLineBresenham(fb, 33, 3, 39, 9, colors::Red);

        CHECK(fb.saveBMP("output/test_line.bmp"));
    }

    // successful test
    std::puts("test_line: PASS");
    return 0;
}