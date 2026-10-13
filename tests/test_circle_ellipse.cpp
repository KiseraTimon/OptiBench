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


    // test validating if the ellipse is drawn correctly
    {
        // initializing an empty canvas
        Framebuffer fb(201, 121, colors::Black);

        // drawing a white ellipse (oval); width radius of 80, height radius of 40
        drawEllipse(fb, 100, 60, 80, 40, colors::White);

        // test validation: verifying the right/left extremes are painted
        CHECK(lit(fb, 180, 60)); CHECK(lit(fb, 20, 60));

        // test validation: verifying top/bottom extremes are painted
        CHECK(lit(fb, 100, 100)); CHECK(lit(fb, 100, 20));

        // looping through every row on the canvas.
        for (int y = 0; y < 121; ++y) {
            // looping through every column on the canvas.
            for (int x = 0; x < 201; ++x) {
                // for painted pixels:
                if (lit(fb, x, y)) {

                    /*
                     * ellipse equation:
                     *      x^2/a^2 + y^2/b^2 = 1
                     */

                    // calculating the mathematical ellipse equation for the pixel
                    const double f = std::pow((x - 100.0) / 80.0, 2) + std::pow((y - 60.0) / 40.0, 2);

                    // test validation: checking the math result is close to 1.0; implying the pixel lies on the correct curve
                    CHECK(std::fabs(f - 1.0) < 0.1);
                }
            }
        }
    }

    // test validating the math that detects collisions between a laser and a circle
    {

        // shooting a ray moving directly right
        Hit h = rayCircleIntersect({{-5, 0}, {1, 0}}, {0, 0}, 1.0);

        // test validation: verifying the function correctly reported a collision.
        CHECK(h.hit);

        // test validation: verifying the distance travelled is as expected
        CHECK(std::fabs(h.t - 4.0) < 1e-9);

        // test validation: verifying the exact X coordinate of the hit is as expected.
        CHECK(std::fabs(h.point.x + 1.0) < 1e-9);

        // test validation: verifying the normal (bounce direction) is pointing straight left; right back at the laser.
        CHECK(std::fabs(h.normal.x + 1.0) < 1e-9);

        // test validation: verifying a laser shooting parallel but too high, missing the circle.
        CHECK(!rayCircleIntersect({{-5, 2}, {1, 0}}, {0, 0}, 1.0).hit);

        // test validation: verifying that shooting away from the circle correctly returns a miss.
        CHECK(!rayCircleIntersect({{5, 0}, {1, 0}}, {0, 0}, 1.0).hit);

        // Shooting a ray starting at exactly {0,0} (inside the circle) moving right.
        Hit e = rayCircleIntersect({{0, 0}, {1, 0}}, {0, 0}, 1.0);

        // test validation: verifying a hit is detected, verifies it hit the right-side wall at x=1.0, and verifies the normal points out to the right. (Restored missing '-').
        CHECK(e.hit); CHECK(std::fabs(e.point.x - 1.0) < 1e-9); CHECK(e.normal.x > 0.99);


        // test validation: verifying that a ray shot exactly at the top edge height successfully detects a hit.
        CHECK(rayCircleIntersect({{-3, 1}, {1, 0}}, {0, 0}, 1.0).hit);
    }

    // message for successful tests
    std::puts("test_circle_ellipse: PASS");
    return 0;
}