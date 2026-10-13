#pragma once

#include "Color.h"
#include "Framebuffer.h"
#include "Ray.h"
#include "Vec2.h"

namespace ob {
    /**
     * Rasterization
     * These function promises are for drawing shapes, pixel-by-pixel
     * onto the screen
     */

    // function promise to draw the outline
    void drawCircle(Framebuffer& fb, int cx, int cy, int r, Color c);

    // function promise to draw the solid disc
    void fillCircle(Framebuffer& fb, int cx, int cy, int r, Color c);

    // function promise to draw the outline of an ellipse
    void drawEllipse(Framebuffer& fb, int cx, int cy, int rx, int ry, Color c);


    /**
     * Geometry
     * These functions handle the math for light physics simulation
     */

    // method promise finding where light hits a circle
    Hit rayCircleIntersect(const Ray& ray, const Vec2& center, double radius);

    // method calculating the perpendicular direction from the circle
    Vec2 circleNormal(const Vec2& center, const Vec2& point);
}