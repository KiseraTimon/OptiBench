#include <cmath>
#include <cstdio>
#include "optibench/clip_fill.h"

using namespace ob;

/*
 * This demo depends on the Bresenham line.
 * Since the lines module is not merged at the time of coding this,
 * I am using a crude line helper (same as the circle/ellipse demo)
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
    const int n = static_cast<int>(std::ceil((b - a).length()));
    for (int i = 0; i <= n; ++i) {
        const Vec2 p = a + (b - a) * (n ? static_cast<double>(i) / n : 0.0);
        fb.setPixel(static_cast<int>(std::lround(p.x)), static_cast<int>(std::lround(p.y)), c);
    }
}


/**
 * hidden function building a rotated rectangle (e.g. a tilted mirror)
 * @param center Vec2
 * @param halfLength double
 * @param halfThickness double
 * @param angle double: radians
 * @return Polygon
 */
static Polygon rotatedRect(Vec2 center, double halfLength, double halfThickness, double angle) {
    const Vec2 u{std::cos(angle), std::sin(angle)};   // along the mirror
    const Vec2 v = u.perp();                          // across the mirror
    return {
        center - u * halfLength - v * halfThickness,
        center + u * halfLength - v * halfThickness,
        center + u * halfLength + v * halfThickness,
        center - u * halfLength + v * halfThickness,
    };
}


int main() {
    // initializing an empty canvas
    Framebuffer fb(800, 500);

    /*
     * viewport:
     *      the visible area of the optical bench; everything is clipped to it.
     *      the 40px margin outside it shows (faintly) what clipping removed
     */
    const ClipRect view{40, 40, 760, 460};

    // colours
    const Color glass{25, 55, 110};         // prism body
    const Color beamDim{90, 25, 30};        // part of the beam the clipper removed
    const Color beamAir{170, 45, 50};       // beam travelling through air
    const Color beamGlass{255, 120, 120};   // beam travelling through glass
    const Color rayFaint{130, 60, 50};       // part of a ray the clipper removed
    const Color silver{185, 190, 200};      // mirror coating


    /*
     * laser source (bottom-left):
     *      outline only, then coloured with a flood fill seeded inside it
     */
    const Vec2 source{110, 380};
    drawPolygon(fb, {{70, 360}, {110, 360}, {110, 400}, {70, 400}}, colors::Grey);
    floodFill(fb, 90, 380, Color{120, 40, 45});


    /*
     * mirror (top-right):
     *      a tilted slab drawn as an outline, then coloured with a boundary fill
     *      that spreads from the centre until it meets the outline colour
     */
    const Vec2 mirrorCenter{650, 150};
    drawPolygon(fb, rotatedRect(mirrorCenter, 70, 7, -0.6), colors::White);
    boundaryFill(fb, static_cast<int>(mirrorCenter.x), static_cast<int>(mirrorCenter.y), silver, colors::White);


    /*
     * light beam:
     *      a wide band of light entering from off-canvas (left) and leaving off-canvas (right).
     *      Sutherland-Hodgman clips it to the viewport; the faint outline shows the full beam
     */
    const Polygon beam{{-60, 170}, {860, 290}, {860, 330}, {-60, 210}};
    fillPolygon(fb, beam, beamDim);
    fillPolygon(fb, clipPolygonToRect(beam, view), beamAir);


    /*
     * prism:
     *      a triangle filled with the scanline algorithm
     */
    const Polygon prism{{400, 90}, {520, 330}, {280, 330}};
    fillPolygon(fb, prism, glass);

    // the part of the beam inside the glass = Sutherland-Hodgman clip of the beam against the (convex) prism
    fillPolygon(fb, clipPolygonConvex(beam, prism), beamGlass);

    // bright prism edges drawn on top
    drawPolygon(fb, prism, colors::Cyan);


    /*
     * laser rays:
     *      a fan of rays from the source, each long enough to leave the canvas.
     *      Cohen-Sutherland trims each one to the viewport: faint = removed, bright = kept
     */
    for (int i = 0; i < 6; ++i) {
        const double angle = -0.15 - 0.17 * i;   // fanning upward (y grows downward)
        Vec2 a = source;
        Vec2 b = source + Vec2{std::cos(angle), std::sin(angle)} * 1200.0;

        crudeLine(fb, a, b, rayFaint);
        if (clipLineCohenSutherland(a, b, view)) crudeLine(fb, a, b, colors::Yellow);
    }

    // a ray passing entirely through the top margin: Cohen-Sutherland rejects it, so only the faint line shows
    {
        Vec2 a{-20, 25}, b{820, 10};
        crudeLine(fb, a, b, rayFaint);
        if (clipLineCohenSutherland(a, b, view)) crudeLine(fb, a, b, colors::Yellow);
    }


    // viewport border
    drawPolygon(fb, {{view.xmin, view.ymin}, {view.xmax, view.ymin}, {view.xmax, view.ymax}, {view.xmin, view.ymax}},
                colors::Grey);


    // persisted illustration of the demo
    if (!fb.saveBMP("output/clip_fill_demo.bmp")) {
        std::puts("FAILED to write output/clip_fill_demo.bmp");
        return 1;
    }
    std::puts("OK - wrote output/clip_fill_demo.bmp");
    return 0;
}
