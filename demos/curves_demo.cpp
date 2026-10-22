/*
 * Special Notes: Curves Demo
 *
 * Four panels showing the curves module on the optical bench:
 *
 *   A (top left)       Bezier anatomy: control polygon, de Casteljau construction at t = 0.4,
 *                      tangent / normal / osculating circle, tight bounding box and the
 *                      vertices chosen by adaptive flattening (dense where the curve bends)
 *   B (top right)      the same points through four splines: Catmull-Rom uniform (red) and
 *                      centripetal (green), natural cubic (blue), clamped cubic B-spline
 *                      using them as control points (yellow)
 *   C (bottom left)    parallel laser rays refracting through a lens whose faces are Bezier
 *                      curves; short grey lines are the surface normals at each hit
 *   D (bottom right)   top: a parabolic mirror (an exact quadratic Bezier) focusing rays;
 *                      bottom: rays guided along a gradient-index fibre, traced with RK4 and
 *                      drawn as smooth curves (brighter background = higher refractive index)
 *
 * Reflection and Snell's law are computed here, in the demo; the curves module only supplies
 * hit points and normals oriented against the incoming ray.
 *
 * Running this file:
 *      run this command from the root after building all files
 *      .\build\demo_curves.exe
 */

#include <cmath>
#include <cstdint>
#include <cstdio>
#include <numbers>
#include <vector>

#include "optibench/Color.h"
#include "optibench/Framebuffer.h"
#include "optibench/Line.h"
#include "optibench/Ray.h"
#include "optibench/Vec2.h"
#include "optibench/circle_ellipse.h"
#include "optibench/clip_fill.h"
#include "optibench/curves/BSpline.h"
#include "optibench/curves/CatmullRom.h"
#include "optibench/curves/CubicSpline.h"
#include "optibench/curves/CurveRender.h"
#include "optibench/curves/OpticalCurve.h"
#include "optibench/curves/RayPath.h"

using namespace ob;
using namespace ob::curves;

namespace {

    constexpr int kWidth = 1200;
    constexpr int kHeight = 720;

    // the four panels
    const ClipRect kPanelA{10, 10, 594, 354};
    const ClipRect kPanelB{606, 10, 1190, 354};
    const ClipRect kPanelC{10, 366, 594, 710};
    const ClipRect kPanelD{606, 366, 1190, 710};

    constexpr Color kOrange{255, 150, 40};
    constexpr Color kDimCyan{40, 110, 130};
    constexpr Color kGlass{28, 44, 78};


    // --- helpers

    // scales a colour's brightness; f = 0 is black, f = 1 is unchanged
    Color scaled(Color c, double f) {
        auto s = [&](std::uint8_t v) { return static_cast<std::uint8_t>(std::lround(std::clamp(v * f, 0.0, 255.0))); };
        return Color{s(c.r), s(c.g), s(c.b)};
    }

    // panel border
    void drawFrame(Framebuffer& fb, const ClipRect& r) {
        const Vec2 a{r.xmin, r.ymin}, b{r.xmax, r.ymin}, c{r.xmax, r.ymax}, d{r.xmin, r.ymax};
        drawLine(fb, a, b, colors::Grey);
        drawLine(fb, b, c, colors::Grey);
        drawLine(fb, c, d, colors::Grey);
        drawLine(fb, d, a, colors::Grey);
    }

    // a segment trimmed to a panel (Cohen-Sutherland from the clipping module)
    void drawClipped(Framebuffer& fb, Vec2 a, Vec2 b, const ClipRect& r, Color c) {
        if (clipLineCohenSutherland(a, b, r)) drawLineWu(fb, a, b, c);
    }

    // a filled 3x3 dot
    void dot(Framebuffer& fb, Vec2 p, Color c) {
        const int x = static_cast<int>(std::lround(p.x)), y = static_cast<int>(std::lround(p.y));
        for (int dy = -1; dy <= 1; ++dy)
            for (int dx = -1; dx <= 1; ++dx) fb.setPixel(x + dx, y + dy, c);
    }

    // dashed axis-aligned box
    void drawDashedBox(Framebuffer& fb, const Aabb& box, Color c) {
        const Vec2 a = box.min, b{box.max.x, box.min.y}, d = box.max, e{box.min.x, box.max.y};
        drawDashedLine(fb, a, b, 5, 4, c);
        drawDashedLine(fb, b, d, 5, 4, c);
        drawDashedLine(fb, d, e, 5, 4, c);
        drawDashedLine(fb, e, a, 5, 4, c);
    }

    // Snell's law; n must face against d (n . d < 0). False on total internal reflection
    bool refractDir(Vec2 d, Vec2 n, double n1, double n2, Vec2& out) {
        const double cosI = -n.dot(d);
        const double eta = n1 / n2;
        const double k = 1.0 - eta * eta * (1.0 - cosI * cosI);
        if (k < 0.0) return false;
        out = d * eta + n * (eta * cosI - std::sqrt(k));
        return true;
    }

    // mirror reflection r = d - 2 (d.n) n
    Vec2 reflectDir(Vec2 d, Vec2 n) { return d - n * (2.0 * d.dot(n)); }


    // --- panel A: anatomy of a cubic Bezier

    void panelBezierAnatomy(Framebuffer& fb) {
        const Bezier c({{60, 300}, {150, 50}, {430, 40}, {540, 290}});

        drawDashedBox(fb, c.bounds(), scaled(colors::Yellow, 0.6));
        drawControlPolygon(fb, c.controlPoints(), colors::Grey, colors::White, 4);

        // de Casteljau construction at t = 0.4: each level interpolates the previous one
        constexpr double t = 0.4;
        std::vector<Vec2> level(c.controlPoints().begin(), c.controlPoints().end());
        const Color levelColors[] = {kDimCyan, colors::Cyan};
        for (int depth = 0; level.size() > 1; ++depth) {
            std::vector<Vec2> next;
            for (std::size_t i = 0; i + 1 < level.size(); ++i) next.push_back(lerp(level[i], level[i + 1], t));
            if (next.size() > 1) {
                drawPolyline(next, [&](Vec2 a, Vec2 b) { drawLineWu(fb, a, b, levelColors[depth % 2]); });
                for (const Vec2& p : next) drawCircle(fb, static_cast<int>(p.x), static_cast<int>(p.y), 2, levelColors[depth % 2]);
            }
            level = std::move(next);
        }

        drawCurve(fb, c, colors::White, 0.25, true);
        fillCircle(fb, static_cast<int>(level[0].x), static_cast<int>(level[0].y), 4, colors::Cyan);

        // adaptive flattening vertices: sparse on the flat ends, dense in the tight bend
        for (const Vec2& p : c.flatten(1.5)) dot(fb, p, colors::Red);

        // tangent, normal and osculating circle where the curve bends the most
        double s = 0.0;
        for (int i = 0; i <= 200; ++i)
            if (std::fabs(c.curvature(i / 200.0)) > std::fabs(c.curvature(s))) s = i / 200.0;
        const Vec2 p = c.evaluate(s);
        const Vec2 tg = c.tangent(s), nm = c.normal(s);
        const double k = c.curvature(s);
        if (std::fabs(k) > 1e-6) {
            const double radius = std::fabs(1.0 / k);
            const Vec2 center = p + nm / k;
            const bool fits = center.x - radius > kPanelA.xmin && center.x + radius < kPanelA.xmax &&
                              center.y - radius > kPanelA.ymin && center.y + radius < kPanelA.ymax;
            if (fits) {
                drawCircle(fb, static_cast<int>(std::lround(center.x)), static_cast<int>(std::lround(center.y)),
                           static_cast<int>(std::lround(radius)), scaled(kOrange, 0.55));
                fillCircle(fb, static_cast<int>(std::lround(center.x)), static_cast<int>(std::lround(center.y)), 2, kOrange);
            }
        }
        drawThickLine(fb, p - tg * 50.0, p + tg * 50.0, 2, colors::Green);
        drawThickLine(fb, p, p + nm * 40.0, 2, kOrange);
    }


    // --- panel B: four splines through the same points

    void panelSplines(Framebuffer& fb) {
        const std::vector<Vec2> pts{{650, 250}, {720, 110}, {790, 290}, {880, 80}, {950, 210}, {1000, 180}, {1140, 300}};

        const BSpline bs = BSpline::clamped(3, pts);
        drawControlPolygon(fb, pts, scaled(colors::Grey, 1.3), colors::White, 0);
        drawCurve(fb, bs.toBezier(), colors::Yellow, 0.25, true);
        drawCurve(fb, NaturalCubicSpline(pts).toBezier(), colors::Blue, 0.25, true);
        drawCurve(fb, CatmullRom(pts, Parametrization::Uniform).toBezier(), colors::Red, 0.25, true);
        drawCurve(fb, CatmullRom(pts, Parametrization::Centripetal).toBezier(), colors::Green, 0.25, true);
        for (const Vec2& p : pts) drawCircle(fb, static_cast<int>(p.x), static_cast<int>(p.y), 4, colors::White);
    }


    // --- panel C: refraction through a Bezier lens

    // traces a ray through one closed refractive surface, drawing each straight piece
    void traceThroughLens(Framebuffer& fb, const OpticalSurface& lens, Ray ray, const ClipRect& panel, Color c) {
        for (int bounce = 0; bounce < 6; ++bounce) {
            const auto hit = lens.firstHit(ray);
            if (!hit) break;
            drawClipped(fb, ray.origin, hit->point, panel, c);
            drawLine(fb, hit->point, hit->point + hit->surfaceNormal * 14.0, scaled(colors::White, 0.45));

            // frontFace: entering the glass (air -> glass), otherwise leaving it
            const double n1 = hit->frontFace ? 1.0 : lens.refractiveIndex();
            const double n2 = hit->frontFace ? lens.refractiveIndex() : 1.0;
            const Vec2 d = ray.direction.normalized();
            Vec2 out;
            if (!refractDir(d, hit->normal, n1, n2, out)) out = reflectDir(d, hit->normal);
            ray = Ray{hit->point, out};
        }
        drawClipped(fb, ray.origin, ray.origin + ray.direction.normalized() * 2000.0, panel, c);
    }

    void panelLens(Framebuffer& fb) {
        LensSpec spec;
        spec.center = {250, 538};
        spec.aperture = 260;
        spec.edgeThickness = 8;
        spec.frontSag = 34;
        spec.backSag = 34;
        const OpticalSurface lens(lensProfile(spec), SurfaceKind::Refractive, 1.5);

        // glass body: every pixel the surface reports as inside
        const Aabb box = lens.profile().bounds();
        for (int y = static_cast<int>(box.min.y); y <= static_cast<int>(box.max.y); ++y)
            for (int x = static_cast<int>(box.min.x); x <= static_cast<int>(box.max.x); ++x)
                if (lens.contains({x + 0.5, y + 0.5})) fb.setPixel(x, y, kGlass);
        drawCurve(fb, lens.profile(), colors::Cyan, 0.25, true);

        // parallel beam from the left, coloured from the edge (red) to the axis (yellow)
        for (int k = -5; k <= 5; ++k) {
            const double f = std::fabs(k) / 5.0;
            const Color c{255, static_cast<std::uint8_t>(220 - 160 * f), static_cast<std::uint8_t>(60 - 20 * f)};
            traceThroughLens(fb, lens, Ray{{12, spec.center.y + k * 22.0}, {1, 0}}, kPanelC, c);
        }
    }


    // --- panel D: parabolic mirror and gradient-index fibre

    void panelMirrorAndGrin(Framebuffer& fb) {
        // parabolic mirror opening to the left; parallel rays reflect through the focus
        const Vec2 vertex{1170, 455};
        const Vec2 axis{-1, 0};
        const double focal = 120.0;
        const OpticalSurface mirror(CompositeBezier({parabolicProfile(vertex, axis, focal, 75.0)}), SurfaceKind::Reflective);
        drawCurve(fb, mirror.profile(), colors::White, 0.25, true);
        for (int k = -4; k <= 4; ++k) {
            if (k == 0) continue;
            const Ray in{{620, vertex.y + k * 16.0}, {1, 0}};
            const auto hit = mirror.firstHit(in);
            if (!hit) continue;
            drawClipped(fb, in.origin, hit->point, kPanelD, colors::Red);
            const Vec2 r = reflectDir(in.direction, hit->normal);
            drawClipped(fb, hit->point, hit->point + r * 260.0, kPanelD, scaled(colors::Red, 0.7));
        }
        const Vec2 focus = vertex + axis * focal;
        drawCircle(fb, static_cast<int>(focus.x), static_cast<int>(focus.y), 5, colors::Yellow);

        // gradient-index fibre: n(y) falls off quadratically from its axis, which guides rays
        const double axisY = 632.0, halfWidth = 68.0, n0 = 1.55, a = 0.00006;
        auto n = [=](Vec2 p) { return n0 - a * (p.y - axisY) * (p.y - axisY); };
        for (int y = static_cast<int>(axisY - halfWidth); y <= static_cast<int>(axisY + halfWidth); ++y) {
            const double shade = (n({0, static_cast<double>(y)}) - 1.2) / (n0 - 1.2);
            for (int x = 616; x <= 1182; ++x) fb.setPixel(x, y, scaled(Color{40, 60, 120}, 0.4 + 0.6 * shade));
        }

        GrinOptions opts;
        opts.step = 4.0;
        opts.gradient = [=](Vec2 p) { return Vec2(0, -2 * a * (p.y - axisY)); };
        opts.stop = [](Vec2 p) { return p.x > 1182; };
        const Color rayColors[] = {colors::Green, colors::Yellow, kOrange, colors::Red};
        const double angles[] = {6.0, 11.0, 16.0, -14.0};
        for (int i = 0; i < 4; ++i) {
            const double th = angles[i] * std::numbers::pi / 180.0;
            const RayPath path = traceGradientIndex(n, {616, axisY}, {std::cos(th), -std::sin(th)}, opts);
            drawCurve(fb, path.toCurve(), rayColors[i], 0.25, true);
        }
    }
}

int main() {
    Framebuffer fb(kWidth, kHeight, colors::BenchDark);

    panelBezierAnatomy(fb);
    panelSplines(fb);
    panelLens(fb);
    panelMirrorAndGrin(fb);
    for (const ClipRect& r : {kPanelA, kPanelB, kPanelC, kPanelD}) drawFrame(fb, r);

    if (!fb.saveBMP("output/curves_demo.bmp")) {
        std::printf("could not write output/curves_demo.bmp (run from the repository root)\n");
        return 1;
    }
    std::printf("wrote output/curves_demo.bmp (%dx%d)\n", kWidth, kHeight);
    return 0;
}
