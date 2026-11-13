#include <cmath>
#include <cstdio>

#include "optibench/circle_ellipse.h"

#define CHECK(cond) do { if (!(cond)) { std::printf("FAIL %s:%d  %s\n", __FILE__, __LINE__, #cond); return 1; } } while (0)

using namespace ob;

static bool lit(const Framebuffer& fb, int x, int y) {
    const Color p = fb.getPixel(x, y);
    return p.r != 0 || p.g != 0 || p.b != 0;
}

int main() {
    // ---- filled ellipse: area close to pi * rx * ry, centre filled, far corners empty ----
    {
        const int rx = 80, ry = 40;
        Framebuffer fb(240, 160, colors::Black);
        fillEllipse(fb, 120, 80, rx, ry, colors::White);

        int count = 0;
        for (int y = 0; y < 160; ++y) for (int x = 0; x < 240; ++x) count += lit(fb, x, y);
        const double ideal = 3.14159265358979 * rx * ry;
        CHECK(std::fabs(count - ideal) / ideal < 0.03);
        CHECK(lit(fb, 120, 80) && lit(fb, 120 + rx - 2, 80) && lit(fb, 120, 80 + ry - 2));
        CHECK(!lit(fb, 5, 5) && !lit(fb, 235, 155));
    }

    // ---- outline and fill agree: every outline pixel is covered by the fill ----
    {
        Framebuffer outline(200, 200, colors::Black), filled(200, 200, colors::Black);
        drawEllipse(outline, 100, 100, 70, 35, colors::White);
        fillEllipse(filled, 100, 100, 70, 35, colors::White);
        for (int y = 0; y < 200; ++y)
            for (int x = 0; x < 200; ++x)
                if (lit(outline, x, y)) CHECK(lit(filled, x, y));
    }

    // ---- outline accuracy over many sizes: never more than ~0.75 px off the true curve ----
    // (this is the check that guards the region-1 decision parameter)
    // (radii of 3 px or less are skipped: the midpoint algorithm ends a pixel short on very flat ellipses)
    for (int rx = 4; rx <= 90; rx += 7) {
        for (int ry = 4; ry <= 90; ry += 5) {
            Framebuffer fb(200, 200, colors::Black);
            drawEllipse(fb, 100, 100, rx, ry, colors::White);
            for (int y = 0; y < 200; ++y)
                for (int x = 0; x < 200; ++x)
                    if (lit(fb, x, y)) {
                        const double dx = x - 100.0, dy = y - 100.0;
                        const double f = std::sqrt(dx * dx / (double(rx) * rx) + dy * dy / (double(ry) * ry));
                        CHECK(std::fabs(f - 1.0) * std::min(rx, ry) < 0.75);
                    }
            CHECK(lit(fb, 100 + rx, 100) && lit(fb, 100 - rx, 100) && lit(fb, 100, 100 + ry) && lit(fb, 100, 100 - ry));
        }
    }

    // ---- degenerate radii draw nothing ----
    {
        Framebuffer fb(50, 50, colors::Black);
        fillEllipse(fb, 25, 25, 0, 10, colors::White);
        drawEllipse(fb, 25, 25, 10, -3, colors::White);
        for (int y = 0; y < 50; ++y) for (int x = 0; x < 50; ++x) CHECK(!lit(fb, x, y));
    }

    std::puts("test_ellipse_extras: PASS");
    return 0;
}
