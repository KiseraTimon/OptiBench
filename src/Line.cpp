#include "../include/optibench/Line.h"

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <utility>

namespace ob {
    namespace {
        // small number used to treat tiny values as zero
        constexpr double kEpsilon = 1e-9;

        // whole number part of a value (rounded down)
        int ipart(double v) { return static_cast<int>(std::floor(v)); }

        // part of a value after the decimal point
        double fpart(double v) { return v - std::floor(v); }

        // what is left to reach the next whole number
        double rfpart(double v) { return 1.0 - fpart(v); }

        // mixes a color into a pixel; amount 0 keeps the pixel, amount 1 fully replaces it
        void blendPixel(Framebuffer& fb, int x, int y, Color c, double amount) {
            amount = std::clamp(amount, 0.0, 1.0);
            const Color old = fb.getPixel(x, y);

            auto mix = [&](std::uint8_t from, std::uint8_t to) {
                return static_cast<std::uint8_t>(std::lround(from + (to - from) * amount));
            };

            fb.setPixel(x, y, Color{mix(old.r, c.r), mix(old.g, c.g), mix(old.b, c.b)});
        }
    }

    // logic drawing a line with the DDA algorithm
    void drawLineDDA(Framebuffer& fb, int x0, int y0, int x1, int y1, Color c) {
        // ordering the endpoints so A->B and B->A draw the same pixels
        if (x0 > x1 || (x0 == x1 && y0 > y1)) { std::swap(x0, x1); std::swap(y0, y1); }

        const int dx = x1 - x0;
        const int dy = y1 - y0;

        // one step per pixel along the longer side
        const int steps = std::max(std::abs(dx), std::abs(dy));
        if (steps == 0) { fb.setPixel(x0, y0, c); return; }

        // how far x and y move on each step
        const double xInc = static_cast<double>(dx) / steps;
        const double yInc = static_cast<double>(dy) / steps;

        double x = x0, y = y0;
        for (int i = 0; i <= steps; ++i) {
            fb.setPixel(static_cast<int>(std::lround(x)), static_cast<int>(std::lround(y)), c);
            x += xInc;
            y += yInc;
        }
    }

    // logic drawing a line with Bresenham's algorithm
    void drawLineBresenham(Framebuffer& fb, int x0, int y0, int x1, int y1, Color c) {
        // ordering the endpoints so A->B and B->A draw the same pixels
        if (x0 > x1 || (x0 == x1 && y0 > y1)) { std::swap(x0, x1); std::swap(y0, y1); }

        const int dx = std::abs(x1 - x0);
        const int dy = -std::abs(y1 - y0);

        // direction of each step; +1 or -1
        const int sx = x0 < x1 ? 1 : -1;
        const int sy = y0 < y1 ? 1 : -1;

        /**
         * CONTEXT:
         *      err keeps track of how far the drawn pixels are from the real line;
         *      it decides whether the next step moves in x, y or both
         */
        int err = dx + dy;

        while (true) {
            fb.setPixel(x0, y0, c);
            if (x0 == x1 && y0 == y1) break;

            const int e2 = 2 * err;
            if (e2 >= dy) { err += dy; x0 += sx; }
            if (e2 <= dx) { err += dx; y0 += sy; }
        }
    }

    // logic drawing a line between 2D points by rounding them to pixels
    void drawLine(Framebuffer& fb, Vec2 a, Vec2 b, Color c) {
        drawLineBresenham(fb,
            static_cast<int>(std::lround(a.x)), static_cast<int>(std::lround(a.y)),
            static_cast<int>(std::lround(b.x)), static_cast<int>(std::lround(b.y)), c);
    }

    // logic drawing a smooth line with Xiaolin Wu's algorithm
    void drawLineWu(Framebuffer& fb, Vec2 a, Vec2 b, Color c) {
        double x0 = a.x, y0 = a.y, x1 = b.x, y1 = b.y;

        // steep lines are turned on their side so we can always step along x
        const bool steep = std::fabs(y1 - y0) > std::fabs(x1 - x0);
        if (steep) { std::swap(x0, y0); std::swap(x1, y1); }
        if (x0 > x1) { std::swap(x0, x1); std::swap(y0, y1); }

        // how much y changes for every step in x
        const double dx = x1 - x0;
        const double dy = y1 - y0;
        const double gradient = dx < kEpsilon ? 1.0 : dy / dx;

        // colors a pixel, turning steep lines back the right way up
        auto plot = [&](int x, int y, double amount) {
            if (steep) blendPixel(fb, y, x, c, amount);
            else       blendPixel(fb, x, y, c, amount);
        };

        // first endpoint
        double xEnd = std::round(x0);
        double yEnd = y0 + gradient * (xEnd - x0);
        double xGap = rfpart(x0 + 0.5);
        const int xPixel1 = static_cast<int>(xEnd);
        const int yPixel1 = ipart(yEnd);
        plot(xPixel1, yPixel1,     rfpart(yEnd) * xGap);
        plot(xPixel1, yPixel1 + 1, fpart(yEnd) * xGap);

        // y position of the line at the next column
        double interY = yEnd + gradient;

        // second endpoint
        xEnd = std::round(x1);
        yEnd = y1 + gradient * (xEnd - x1);
        xGap = fpart(x1 + 0.5);
        const int xPixel2 = static_cast<int>(xEnd);
        const int yPixel2 = ipart(yEnd);
        plot(xPixel2, yPixel2,     rfpart(yEnd) * xGap);
        plot(xPixel2, yPixel2 + 1, fpart(yEnd) * xGap);

        /**
         * CONTEXT:
         *      a smooth line usually falls between two pixels;
         *      both pixels are colored, the closer one more strongly
         */
        for (int x = xPixel1 + 1; x < xPixel2; ++x) {
            plot(x, ipart(interY),     rfpart(interY));
            plot(x, ipart(interY) + 1, fpart(interY));
            interY += gradient;
        }
    }

    // logic drawing a thick line by coloring every pixel close enough to the segment
    void drawThickLine(Framebuffer& fb, Vec2 a, Vec2 b, int thickness, Color c) {
        if (thickness <= 1) { drawLine(fb, a, b, c); return; }

        const double half = thickness / 2.0;
        const Segment s{a, b};

        // box around the line, kept inside the canvas
        const int minX = std::max(0, static_cast<int>(std::floor(std::min(a.x, b.x) - half)));
        const int maxX = std::min(fb.width() - 1, static_cast<int>(std::ceil(std::max(a.x, b.x) + half)));
        const int minY = std::max(0, static_cast<int>(std::floor(std::min(a.y, b.y) - half)));
        const int maxY = std::min(fb.height() - 1, static_cast<int>(std::ceil(std::max(a.y, b.y) + half)));

        // coloring pixels within half the thickness of the line
        for (int y = minY; y <= maxY; ++y)
            for (int x = minX; x <= maxX; ++x)
                if (distanceToSegment(Vec2{static_cast<double>(x), static_cast<double>(y)}, s) <= half)
                    fb.setPixel(x, y, c);
    }

    // logic drawing a dashed line as a series of short lines
    void drawDashedLine(Framebuffer& fb, Vec2 a, Vec2 b, int dashLen, int gapLen, Color c) {
        if (dashLen <= 0) return;
        if (gapLen <= 0) { drawLine(fb, a, b, c); return; }

        const double total = (b - a).length();
        if (total < kEpsilon) { drawLine(fb, a, b, c); return; }

        // unit vector pointing from a to b
        const Vec2 dir = (b - a) / total;

        // drawing one dash, then skipping one gap, until the end of the line
        for (double t = 0.0; t < total; t += dashLen + gapLen) {
            const double end = std::min(t + dashLen, total);
            drawLine(fb, a + dir * t, a + dir * end, c);
        }
    }

    // logic calculating the length of a segment
    double length(const Segment& s) {
        return (s.b - s.a).length();
    }

    // logic calculating the halfway point of a segment
    Vec2 midpoint(const Segment& s) {
        return (s.a + s.b) * 0.5;
    }

    // logic producing a unit vector at a right angle to a segment
    Vec2 normal(const Segment& s) {
        return (s.b - s.a).perp().normalized();
    }

    // logic calculating the shortest distance from a point to a segment
    double distanceToSegment(Vec2 p, const Segment& s) {
        const Vec2 ab = s.b - s.a;
        const double lenSq = ab.dot(ab);

        // safety check for a segment with no length; it is just a point
        if (lenSq < kEpsilon) return (p - s.a).length();

        // closest point on the segment to p; kept between the two endpoints
        const double t = std::clamp((p - s.a).dot(ab) / lenSq, 0.0, 1.0);
        return (p - (s.a + ab * t)).length();
    }

    // logic finding where two segments cross
    bool intersectSegments(const Segment& s1, const Segment& s2, Vec2& out) {
        const Vec2 r = s1.b - s1.a;
        const Vec2 q = s2.b - s2.a;

        // parallel segments never cross at a single point
        const double denom = r.cross(q);
        if (std::fabs(denom) < kEpsilon) return false;

        /**
         * CONTEXT:
         *      t is how far along s1 the crossing is, u is how far along s2;
         *      both must be between 0 (start) and 1 (end)
         */
        const Vec2 w = s2.a - s1.a;
        const double t = w.cross(q) / denom;
        const double u = w.cross(r) / denom;
        if (t < 0.0 || t > 1.0 || u < 0.0 || u > 1.0) return false;

        out = s1.a + r * t;
        return true;
    }

    // logic finding where a ray first hits a segment
    Hit intersectRaySegment(const Ray& r, const Segment& s) {
        Hit h;

        // unit direction so that t is a real distance
        const Vec2 d = r.direction.normalized();
        if (d.length() < kEpsilon) return h;

        const Vec2 q = s.b - s.a;

        // a ray running parallel to the segment never hits it
        const double denom = d.cross(q);
        if (std::fabs(denom) < kEpsilon) return h;

        /**
         * CONTEXT:
         *      t is how far the ray travels to the hit,
         *      u is how far along the segment the hit is (0 to 1)
         */
        const Vec2 w = s.a - r.origin;
        const double t = w.cross(q) / denom;
        const double u = w.cross(d) / denom;

        // hits behind the ray or past the ends of the segment are misses
        if (t <= kEpsilon || u < 0.0 || u > 1.0) return h;

        h.hit = true;
        h.t = t;
        h.point = r.origin + d * t;

        // flipping the normal so it points back towards the ray
        h.normal = normal(s);
        if (h.normal.dot(d) > 0.0) h.normal = -h.normal;

        return h;
    }
}
