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

int main() {
    // initializing an empty canvas
    Framebuffer fb(800, 500);

    /*
     * circular lens:
     *      drawing the main lens (circular lens): features a
     *      filled body, bright outline
     */
    const Vec2 C {400, 250};    // center point
    const double R = 120;             // radius

    // painting a solid, dark-blue circle; represents glass body of the lens
    fillCircle(fb, 400, 250, 120, Color{25, 55, 110});

    // cyan outline around the lens
    drawCircle(fb, 400, 250, 120, colors::Cyan);

    // second outline to give a 'thickness' effect
    drawCircle(fb, 400, 250, 119, colors::Cyan);


    /*
     * elliptical lense:
     *      painting an elliptical lens (right) and a concentric-circle
     *      sample (bottom-left)
     */

    // painting a yellow oval/ellipse on the right of the screen
    drawEllipse(fb, 660, 130, 90, 45, colors::Yellow);

    // drawing a smaller dark-blue oval/ellipse within the one above
    drawEllipse(fb, 660, 130, 60, 30, Color{180, 150, 40});

    // painting a bullseye using 5 grey circles
    for (int r = 15; r <=75; r += 15) drawCircle(fb, 110, 390, r, colors::Grey);

    // laser ray (left-right), aimed slightly off-center
    const Ray ray{{40, 190}, Vec2{1, 0.12}.normalized()};

    // checking if the ray collides with the circle
    const Hit in = rayCircleIntersect(ray, C, R);

    if (in.hit) {
        // line tracing the incoming ray
        crudeLine(fb, ray.origin, in.point, colors::Red);

        // line tracing the reflected ray
        crudeLine(fb, in.point, in.point + in.normal * 50, colors::Yellow);

        // refracted ray beginning inside the lens
        const Ray inside{in.point, ray.direction};

        // checking if the refracted ray appears on the opposite side of the lense
        const Hit out = rayCircleIntersect(inside, C, R);
        if (out.hit) {
            // line tracing the beginning of the refracted ray
            crudeLine(fb, in.point, out.point, colors::White);

            // line tracing the refracted ray
            crudeLine(fb, out.point, out.point + out.normal * 50, colors::Yellow);
        }
    }


    // persisted illustration of the demo
    if (!fb.saveBMP("output/circle_ellipse_demo.bmp")) {
        std::puts("wrote into 'output/circle_ellipse_demo.bmp'");
        return 0;
    }
}