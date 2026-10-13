#pragma once
#include <vector>

#include "Color.h"
#include "Framebuffer.h"
#include "Vec2.h"

namespace ob {
    // a polygon is a list of vertices in order; the last vertex joins back to the first
    using Polygon = std::vector<Vec2>;

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
}
