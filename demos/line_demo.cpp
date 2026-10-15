#include <cmath>
#include <cstdio>
#include <vector>
#include "../include/optibench/Line.h"

int main() {
    using namespace ob;

    // framebuffer instance
    Framebuffer fb(600, 400);

   
    auto bresenham = [&](Vec2 a, Vec2 b, Color c) { drawLine(fb, a, b, c); };

    auto dda = [&](Vec2 a, Vec2 b, Color c) {
        drawLineDDA(fb,
            static_cast<int>(std::lround(a.x)), static_cast<int>(std::lround(a.y)),
            static_cast<int>(std::lround(b.x)), static_cast<int>(std::lround(b.y)), c);
    };

    auto wu = [&](Vec2 a, Vec2 b, Color c) { drawLineWu(fb, a, b, c); };

    // draws one fibre and traces its laser beam using the given line algorithm
    auto drawFibre = [&](double top, Color beam, auto drawBeam) {
        const double bottom = top + 70;
        const double mid = top + 35;

        // mirrors lining the top and bottom of the fibre
        const std::vector<Segment> mirrors = {
            Segment{Vec2{80, top}, Vec2{560, top}},
            Segment{Vec2{80, bottom}, Vec2{560, bottom}},
        };

        // detector screen catching the beam at the end of the fibre
        const Segment detector{Vec2{580, top}, Vec2{580, bottom}};

        // optical axis along the centre of the fibre
        drawDashedLine(fb, Vec2{80, mid}, Vec2{560, mid}, 6, 6, colors::Grey);

        for (const Segment& m : mirrors) drawThickLine(fb, m.a, m.b, 3, colors::White);
        drawThickLine(fb, detector.a, detector.b, 5, colors::Yellow);

        // laser source
        drawThickLine(fb, Vec2{30, mid}, Vec2{55, mid}, 9, beam);

        // laser beam leaving the source at an angle
        Ray ray{Vec2{60, mid}, Vec2{1.0, 0.7}};

        for (int bounce = 0; bounce < 20; ++bounce) {
            // finding the closest mirror in the beam's path
            Hit nearest;
            for (const Segment& m : mirrors) {
                const Hit h = intersectRaySegment(ray, m);
                if (h.hit && (!nearest.hit || h.t < nearest.t)) nearest = h;
            }

            // no mirror left; the beam ends on the detector
            if (!nearest.hit) {
                const Hit end = intersectRaySegment(ray, detector);
                if (end.hit) drawBeam(ray.origin, end.point, beam);
                break;
            }

            drawBeam(ray.origin, nearest.point, beam);

            // bouncing off the mirror: r = d - 2(d.n)n
            const Vec2 d = ray.direction.normalized();
            ray = Ray{nearest.point, d - nearest.normal * (2.0 * d.dot(nearest.normal))};
        }
    };

    drawFibre(30, colors::Red, bresenham);
    drawFibre(150, colors::Green, dda);
    drawFibre(270, colors::Cyan, wu);

    // optical bench rail with a tick mark every 40 pixels
    drawThickLine(fb, Vec2{20, 375}, Vec2{580, 375}, 5, colors::Grey);
    for (int x = 20; x <= 580; x += 40) drawLineBresenham(fb, x, 366, x, 384, colors::Grey);

    if (!fb.saveBMP("output/line_demo.bmp")) {
        std::puts("FAILED to write output/line_demo.bmp");
        return 1;
    }
    std::puts("OK - wrote output/line_demo.bmp (laser beams bouncing through three optical fibres)");
    return 0;
}