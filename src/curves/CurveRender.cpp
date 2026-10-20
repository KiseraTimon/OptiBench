#include "optibench/curves/CurveRender.h"

#include <cmath>

#include "optibench/Line.h"
#include "optibench/circle_ellipse.h"

namespace ob::curves {

    void drawCurve(Framebuffer& fb, const CompositeBezier& curve, Color c, double tolerance, bool smooth) {
        if (smooth) drawCurve(curve, tolerance, [&](Vec2 a, Vec2 b) { drawLineWu(fb, a, b, c); });
        else drawCurve(curve, tolerance, [&](Vec2 a, Vec2 b) { drawLine(fb, a, b, c); });
    }

    void drawCurve(Framebuffer& fb, const Bezier& curve, Color c, double tolerance, bool smooth) {
        if (smooth) drawCurve(curve, tolerance, [&](Vec2 a, Vec2 b) { drawLineWu(fb, a, b, c); });
        else drawCurve(curve, tolerance, [&](Vec2 a, Vec2 b) { drawLine(fb, a, b, c); });
    }

    void drawControlPolygon(Framebuffer& fb, std::span<const Vec2> points, Color lineColor, Color pointColor,
                            int markerRadius) {
        drawPolyline(points, [&](Vec2 a, Vec2 b) { drawDashedLine(fb, a, b, 4, 3, lineColor); });
        for (const Vec2& p : points)
            drawCircle(fb, static_cast<int>(std::lround(p.x)), static_cast<int>(std::lround(p.y)), markerRadius, pointColor);
    }
}
