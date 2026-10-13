#include <cmath>
#include <cstdio>
#include "optibench/circle_ellipse.h"

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

    // reading the colour of the pixel at coordinates (x, y).
    const Color p = fb.getPixel(x, y);

    // returns true if the Red, Green, or Blue value is different from pure Black (0, 0, 0).
    return p.r != colors::Black.r || p.g != colors::Black.g || p.b != colors::Black.b;
}


int main() {

    // this test validates if the circle outline is drawn exactly where it should
    {
        // initializing the canvas
        Framebuffer fb(101, 101, colors::Black);

        // painting a hollow circle with a white outline; and radius of 50
        drawCircle(fb, 50, 50, 30, colors::White);

        // setting up a counter to keep track of how many pixels got painted.
        int count = 0;

        // looping through every single row (y) on the canvas.
        for (int y = 0; y < 101; ++y)

            // looping through every single column (x) in that row.
            for (int x = 0; x < 101; ++x)

                // checking if the pixel at this (x, y) location was painted white.
                if (lit(fb, x, y)) {

                    // incrementing our painted pixel counter.
                    ++count;

                    // calculating the exact distance from the center
                    const double d = std::hypot(x - 50.0, y - 50.0);

                    // test validation: verifying this pixel is exactly on the circle line.
                    CHECK(std::fabs(d - 30.0) < 1.0);
                }

        // test validation: checking to make sure we actually drew a circle and not a few random dots
        CHECK(count > 100);

        // test validation: checking if the left/right extremes were painted.
        CHECK(lit(fb, 80, 50)); CHECK(lit(fb, 20, 50));

        // test validation: checking if the top/bottoms extremes were painted.
        CHECK(lit(fb, 50, 80)); CHECK(lit(fb, 50, 20));
    }


    // test validating if the solid, filled-in circle is drawn correctly
    {
        // initializing an empty canvas
        Framebuffer fb(101, 101, colors::Black);

        // painting a solid white circle from the center with a radius of 30
        fillCircle(fb, 50, 50, 30, colors::White);

        // test validation: the exact center is painted
        CHECK(lit(fb, 50, 50)); CHECK(lit(fb, 70, 50));

        // test validation: pixels beyond the specified coords are not painted
        CHECK(!lit(fb, 85, 50)); CHECK(!lit(fb, 5, 5));
    }
}