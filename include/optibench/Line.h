#pragma once
#include "Framebuffer.h"
#include "Ray.h"
#include "Vec2.h"

namespace ob {

    // data component of a line segment
    struct Segment {
        // 2D point where the segment starts
        Vec2 a;

        // 2D point where the segment ends
        Vec2 b;
    };


    /*
     * Line algorithms
     */
    // draws a line using the DDA algorithm
    void drawLineDDA(Framebuffer& fb, int x0, int y0, int x1, int y1, Color c);

    // draws a line using Bresenham's algorithm; the default line drawer
    void drawLineBresenham(Framebuffer& fb, int x0, int y0, int x1, int y1, Color c);

    // draws a line between two 2D points by rounding them to the nearest pixels
    void drawLine(Framebuffer& fb, Vec2 a, Vec2 b, Color c);

    // draws a smooth (anti-aliased) line using Xiaolin Wu's algorithm
    void drawLineWu(Framebuffer& fb, Vec2 a, Vec2 b, Color c);


    /*
     * Line styles
     */
    // draws a line that is several pixels wide
    void drawThickLine(Framebuffer& fb, Vec2 a, Vec2 b, int thickness, Color c);

    // draws a dashed line; dashLen pixels drawn, then gapLen pixels skipped
    void drawDashedLine(Framebuffer& fb, Vec2 a, Vec2 b, int dashLen, int gapLen, Color c);


    /*
     * Line operations
     */
    // calculating the length of a segment
    double length(const Segment& s);

    // calculating the point halfway along a segment
    Vec2 midpoint(const Segment& s);

    // producing a unit vector at a right angle to a segment
    Vec2 normal(const Segment& s);

    // calculating the shortest distance from a point to a segment
    double distanceToSegment(Vec2 p, const Segment& s);

    // finds where two segments cross; returns false if they do not
    bool intersectSegments(const Segment& s1, const Segment& s2, Vec2& out);

    //finds where a laser ray hits a segment
    Hit intersectRaySegment(const Ray& r, const Segment& s);
}
