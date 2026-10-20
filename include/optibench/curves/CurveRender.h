#pragma once

/*
 * Special Notes: Curve Rendering Adapter
 *
 * The curve math never draws. Rendering is a thin step on top of it:
 *
 *      curve --flatten(tolerance)--> polyline --line sink--> pixels
 *
 * A "line sink" is anything callable as sink(Vec2 a, Vec2 b): the line module's ob::drawLine
 * wrapped in a lambda, an anti-aliased drawer, an SVG writer, or a test stub that records the
 * segments. The templates below accept any such callable (checked with C++20 concepts), and the
 * Framebuffer overloads are conveniences that route through the line module (Bresenham by
 * default, Xiaolin Wu when smooth = true) and the circle module for point markers.
 *
 * A flattening tolerance of about a quarter pixel makes the polyline indistinguishable from
 * the curve after rasterization.
 */

#include <concepts>
#include <span>

#include "optibench/Color.h"
#include "optibench/Framebuffer.h"
#include "optibench/curves/Composite.h"

namespace ob::curves {

    // anything that can draw a straight segment from a to b
    template <class F>
    concept LineSink = std::invocable<F&, Vec2, Vec2>;

    // any curve that can be flattened into a polyline
    template <class C>
    concept Flattenable = requires(const C& c, double tol) {
        { c.flatten(tol) } -> std::convertible_to<Polyline>;
    };

    // sends every segment of a polyline to the sink
    template <LineSink F>
    void drawPolyline(std::span<const Vec2> points, F&& line) {
        for (std::size_t i = 0; i + 1 < points.size(); ++i) line(points[i], points[i + 1]);
    }

    // flattens a curve within tolerance and sends the segments to the sink
    template <Flattenable C, LineSink F>
    void drawCurve(const C& curve, double tolerance, F&& line) {
        const Polyline poly = curve.flatten(tolerance);
        drawPolyline(poly, line);
    }

    // draws a curve on the framebuffer with the line module (Wu anti-aliasing if smooth)
    void drawCurve(Framebuffer& fb, const CompositeBezier& curve, Color c, double tolerance = 0.25, bool smooth = false);

    // same for a single Bezier curve
    void drawCurve(Framebuffer& fb, const Bezier& curve, Color c, double tolerance = 0.25, bool smooth = false);

    // dashed control polygon with a small circle at every control point
    void drawControlPolygon(Framebuffer& fb, std::span<const Vec2> points, Color lineColor, Color pointColor,
                            int markerRadius = 3);
}
