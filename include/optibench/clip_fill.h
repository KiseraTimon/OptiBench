#pragma once
#include <vector>

#include "Color.h"
#include "Framebuffer.h"
#include "Vec2.h"

namespace ob {
    // a polygon is a list of vertices in order; the last vertex joins back to the first
    using Polygon = std::vector<Vec2>;

    // axis-aligned clipping window (e.g. the visible area of the optical bench)
    struct ClipRect {
        double xmin, ymin, xmax, ymax;
    };


    /**
     * Rasterization
     * These function promises are for drawing and filling polygons,
     * pixel-by-pixel onto the screen
     */

    // function promise to draw the outline of a polygon
    void drawPolygon(Framebuffer& fb, const Polygon& poly, Color c);

    // function promise to fill a polygon using the scanline (edge table) algorithm; even-odd rule
    void fillPolygon(Framebuffer& fb, const Polygon& poly, Color c);

    // function promise to repaint the 4-connected region of same-coloured pixels around a seed
    void floodFill(Framebuffer& fb, int x, int y, Color fill);

    // function promise to paint outward from a seed until a boundary colour is reached
    void boundaryFill(Framebuffer& fb, int x, int y, Color fill, Color boundary);


    /**
     * Clipping
     * These functions cut geometry down to the part that lies inside a region
     */

    // method promise trimming a line segment to a rectangle (Cohen-Sutherland); false if fully outside
    bool clipLineCohenSutherland(Vec2& a, Vec2& b, const ClipRect& r);

    // method promise clipping a polygon to a rectangle (Sutherland-Hodgman)
    Polygon clipPolygonToRect(const Polygon& subject, const ClipRect& r);

    // method promise clipping a polygon against any convex polygon (Sutherland-Hodgman)
    Polygon clipPolygonConvex(const Polygon& subject, const Polygon& clip);


    /**
     * Geometry
     */

    // method promise calculating the signed area of a polygon (shoelace formula)
    double polygonArea(const Polygon& poly);
}
