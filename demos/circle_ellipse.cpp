#include <cmath>
#include <cstdio>
#include "optibench/circle_ellipse.h"

using namespace ob;

/*
 * This demo depends on the Bresenham line.
 * Since I don't have it at the time of coding this,
 * I am creating my own helper to draw a crude line
 */

// TODO: Integrate with the lines module once it is ready

/**
 * hidden function to draw a simple line by stamping individual pixels
 * between two points 'a', and 'b'
 * @param fb Framebuffer
 * @param a Vec2
 * @param b Vec2
 * @param c Color
 */
static void crudeLine(Framebuffer& fb, Vec2 a, Vec2 b, Color c) {
    // calculating distance between the points
    const int n = static_cast<int>(std::ceil((b - a).length()));

    // looping every pixel for stamping
    for (int i = 0; i <= n; ++i) {
        // calculating exact decimal coordinates for the current pixel along the line
        const Vec2 p = a + (b - a) * (n ? static_cast<double>(i) / n : 0.0);

        // rounding the decimal coords to whole numbers and painting the pixel on the canvas
        fb.setPixel(static_cast<int>(std::lround(p.x)), static_cast<int>(std::lround(p.y)), c);
    }
}