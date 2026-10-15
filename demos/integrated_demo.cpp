/*
 * Special Notes: Integrated Demo
 *
 * This is one optical bench scene that uses every finished module:
 *
 *   Framebuffer
 *   Lines
 *   Clipping/filling
 *   Circles/ellipses lens
 *
 * Light path (white laser):
 *   laser -> glass ball lens -> mirror 1 -> mirror 2 -> prism -> six colours -> detector
 *
 * Reflection uses r = d - 2(d.n)n and refraction uses Snell's law. The prism's refractive
 * index is exaggerated per colour so the dispersion is easy to see.
 *
 * Running this file:
 *      run this command from the root after building all files
 *      .\build\demo_integrated.exe
 */

#include <array>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <vector>

#include "../include/optibench/Color.h"
#include "../include/optibench/Framebuffer.h"
#include "../include/optibench/Line.h"
#include "../include/optibench/Ray.h"
#include "../include/optibench/Vec2.h"
#include "../include/optibench/circle_ellipse.h"
#include "../include/optibench/clip_fill.h"

using namespace ob;

namespace {

    constexpr int kWidth = 960;
    constexpr int kHeight = 600;
    constexpr double kPi = 3.14159265358979323846;

    // the visible area of the bench, where everything is clipped
    const ClipRect kView{30, 30, 930, 570};


    // --- helpers

    /**
     * scales a colour's brightness
     * @param c Color
     * @param f double: 0 = black, 1 = unchanged
     * @return Color
     */
    Color scaled(Color c, double f) {
        auto s = [&](std::uint8_t v) { return static_cast<std::uint8_t>(std::lround(v * f)); };
        return Color{s(c.r), s(c.g), s(c.b)};
    }

    /**
     * builds a rotated rectangle (used for the mirror slabs)
     * @param center Vec2
     * @param halfLength double
     * @param halfThickness double
     * @param angle double: radians
     * @return Polygon
     */
    Polygon rotatedRect(Vec2 center, double halfLength, double halfThickness, double angle) {
        const Vec2 u{std::cos(angle), std::sin(angle)};   // along the mirror
        const Vec2 v = u.perp();                          // across the mirror
        return {
            center - u * halfLength - v * halfThickness,
            center + u * halfLength - v * halfThickness,
            center + u * halfLength + v * halfThickness,
            center - u * halfLength + v * halfThickness,
        };
    }

    /**
     * Snell's law. n must face against d (n . d < 0).
     * @return false on total internal reflection
     */
    bool refractDir(Vec2 d, Vec2 n, double n1, double n2, Vec2& out) {
        const double cosI = -n.dot(d);
        const double eta = n1 / n2;
        const double k = 1.0 - eta * eta * (1.0 - cosI * cosI);
        if (k < 0.0) return false;
        out = d * eta + n * (eta * cosI - std::sqrt(k));
        return true;
    }

    /** mirror reflection: r = d - 2(d.n)n */
    Vec2 reflectDir(Vec2 d, Vec2 n) {
        return d - n * (2.0 * d.dot(n));
    }


    // --- the bench

    struct Lens { Vec2 center; double radius; double n; };

    struct Bench {
        std::vector<Segment> mirrors;
        std::vector<Lens> lenses;
        Polygon prism;          // convex, glass
        Segment detector;
    };

    enum class Kind { None, Mirror, Lens, Prism, Detector };

    struct Found {
        Kind kind = Kind::None;
        Hit hit;
    };

    /**
     * finds the closest thing the ray hits
     * @param inside int: -1 in air, index of the glass body the ray is travelling in otherwise
     *               (lenses use 0..L-1, the prism uses L)
     */
    Found nearest(const Bench& bench, const Ray& ray, int inside) {
        Found best;
        auto consider = [&](Kind k, const Hit& h) {
            if (h.hit && (best.kind == Kind::None || h.t < best.hit.t)) { best.kind = k; best.hit = h; }
        };

        if (inside == -1) {
            for (const Segment& m : bench.mirrors) consider(Kind::Mirror, intersectRaySegment(ray, m));
            consider(Kind::Detector, intersectRaySegment(ray, bench.detector));
        }

        for (std::size_t i = 0; i < bench.lenses.size(); ++i) {
            if (inside == -1 || inside == static_cast<int>(i))
                consider(Kind::Lens, rayCircleIntersect(ray, bench.lenses[i].center, bench.lenses[i].radius));
        }

        const int prismId = static_cast<int>(bench.lenses.size());
        if (inside == -1 || inside == prismId) {
            const std::size_t n = bench.prism.size();
            for (std::size_t i = 0; i < n; ++i)
                consider(Kind::Prism, intersectRaySegment(ray, Segment{bench.prism[i], bench.prism[(i + 1) % n]}));
        }
        return best;
    }

    struct Beam { Vec2 a, b; Color c; };

    struct TraceResult {
        bool exitedPrism = false;
        Vec2 exitPoint, exitDir;
        bool hitDetector = false;
        Vec2 detectorPoint;
    };

    /**
     * follows one ray through the bench, recording every straight piece as a Beam
     *
     * @param nPrism double: refractive index of the prism for this colour
     * @param tint Color: colour of the beam once it has entered the prism
     * @param emitWhite bool: only one of the colour rays records the shared white part
     */
    TraceResult trace(const Bench& bench, Ray ray, double nPrism, Color tint, bool emitWhite, std::vector<Beam>& out) {
        TraceResult res;
        const int prismId = static_cast<int>(bench.lenses.size());
        int inside = -1;
        bool dispersed = false;

        for (int step = 0; step < 60; ++step) {
            ray.direction = ray.direction.normalized();
            const Found f = nearest(bench, ray, inside);
            const Color col = dispersed ? tint : Color{255, 255, 255};
            const bool record = dispersed || emitWhite;

            // nothing left to hit: the beam leaves the bench
            if (f.kind == Kind::None) {
                if (record) out.push_back({ray.origin, ray.origin + ray.direction * 3000.0, col});
                return res;
            }
            if (record) out.push_back({ray.origin, f.hit.point, col});

            if (f.kind == Kind::Detector) {
                res.hitDetector = true;
                res.detectorPoint = f.hit.point;
                return res;
            }

            if (f.kind == Kind::Mirror) {
                ray = Ray{f.hit.point, reflectDir(ray.direction, f.hit.normal)};
                continue;
            }

            // glass: work out which body, its index, and whether the ray is entering or leaving
            const int id = (f.kind == Kind::Lens) ? -2 : prismId;
            double glassN = nPrism;
            int bodyId = prismId;
            if (f.kind == Kind::Lens) {
                // identifying the lens that was hit: the one whose circle the point lies on
                bodyId = 0;
                for (std::size_t i = 0; i < bench.lenses.size(); ++i) {
                    const double d = (f.hit.point - bench.lenses[i].center).length();
                    if (std::fabs(d - bench.lenses[i].radius) < 1e-6) { bodyId = static_cast<int>(i); break; }
                }
                glassN = bench.lenses[bodyId].n;
            }
            (void)id;

            // normal facing against the ray
            Vec2 n = f.hit.normal.normalized();
            if (n.dot(ray.direction) > 0.0) n = -n;

            const bool entering = (inside == -1);
            const double n1 = entering ? 1.0 : glassN;
            const double n2 = entering ? glassN : 1.0;

            Vec2 dir;
            if (refractDir(ray.direction, n, n1, n2, dir)) {
                if (entering) {
                    inside = bodyId;
                    if (bodyId == prismId) dispersed = true;
                } else {
                    inside = -1;
                    if (bodyId == prismId) { res.exitedPrism = true; res.exitPoint = f.hit.point; res.exitDir = dir.normalized(); }
                }
                ray = Ray{f.hit.point, dir};
            } else {
                // total internal reflection: stays inside the glass
                ray = Ray{f.hit.point, reflectDir(ray.direction, n)};
            }
        }
        return res;
    }

}


int main() {
    Framebuffer fb(kWidth, kHeight, colors::BenchDark);

    // --- scene description
    const Vec2 laserOut{100, 450};
    const Color glass{25, 55, 110};
    const Color silver{185, 190, 200};

    Bench bench;
    bench.lenses.push_back({{215, 432}, 48.0, 1.50});                      // glass ball lens
    bench.prism = {{640, 70}, {560, 210}, {720, 210}};                     // equilateral-ish prism
    bench.detector = Segment{{905, 60}, {905, 545}};

    const double mirrorAngle = -kPi / 4.0;                                 // "/" slabs
    const Vec2 mirror1{385, 385};                                          // Positioned to intercept the beam refracted by the lens
    const Vec2 mirror2{454, 130};                                          // Positioned to intercept the upward reflection from mirror 1
    const double halfLen = 40.0, halfThick = 5.0;
    for (const Vec2& m : {mirror1, mirror2}) {
        const Vec2 u{std::cos(mirrorAngle), std::sin(mirrorAngle)};
        bench.mirrors.push_back(Segment{m - u * halfLen, m + u * halfLen});
    }

    // six colours with an exaggerated prism index (real glass is ~1.51-1.53)
    struct Band { const char* name; Color color; double n; };
    const std::array<Band, 6> bands{{
        {"red",    Color{255,  70,  70}, 1.52},
        {"orange", Color{255, 160,  50}, 1.54},
        {"yellow", Color{255, 230,  60}, 1.56},
        {"green",  Color{ 80, 230, 110}, 1.58},
        {"blue",   Color{ 80, 150, 255}, 1.60},
        {"violet", Color{170, 100, 255}, 1.62},
    }};

    // --- tracing
    std::vector<Beam> beams;
    std::array<TraceResult, 6> results;
    for (std::size_t i = 0; i < bands.size(); ++i) {
        results[i] = trace(bench, Ray{laserOut, Vec2{1, 0}}, bands[i].n, bands[i].color, i == 0, beams);
    }

    // --- 1. background grid (Lines)
    const Color gridColor{30, 34, 48};
    for (int x = 0; x < kWidth; x += 40) drawLineBresenham(fb, x, 0, x, kHeight - 1, gridColor);
    for (int y = 0; y < kHeight; y += 40) drawLineBresenham(fb, 0, y, kWidth - 1, y, gridColor);

    // --- 2. dispersion wedges (Sutherland-Hodgman + scanline)
    // each wedge is the region between two neighbouring colour rays, extended far off-canvas
    // and clipped to the viewport before it is filled
    for (std::size_t i = 0; i + 1 < bands.size(); ++i) {
        if (!results[i].exitedPrism || !results[i + 1].exitedPrism) continue;
        const Vec2 e0 = results[i].exitPoint, e1 = results[i + 1].exitPoint;
        const Vec2 f0 = e0 + results[i].exitDir * 2500.0, f1 = e1 + results[i + 1].exitDir * 2500.0;
        const Polygon wedge = {e0, f0, f1, e1};
        fillPolygon(fb, clipPolygonToRect(wedge, kView), scaled(bands[i].color, 0.22));
    }

    // --- 3. glass bodies (scanline fill + circle fill)
    fillPolygon(fb, bench.prism, glass);
    for (const Lens& l : bench.lenses)
        fillCircle(fb, static_cast<int>(l.center.x), static_cast<int>(l.center.y), static_cast<int>(l.radius), glass);

    // --- 4. beam glow (thick lines, clipped by Cohen-Sutherland)
    for (const Beam& b : beams) {
        Vec2 a = b.a, e = b.b;
        if (clipLineCohenSutherland(a, e, kView)) drawThickLine(fb, a, e, 5, scaled(b.c, 0.25));
    }

    // --- 5. laser housing (scanline erase + flood fill)
    const Polygon housing = {{40, 430}, {100, 430}, {100, 470}, {40, 470}};
    fillPolygon(fb, housing, colors::BenchDark);            // clearing the grid lines inside first
    drawPolygon(fb, housing, colors::Grey);
    floodFill(fb, 70, 450, Color{120, 40, 45});
    drawThickLine(fb, Vec2{100, 450}, Vec2{108, 450}, 7, Color{255, 255, 255});   // emitter nozzle

    // --- 6. mirrors (boundary fill)
    for (const Vec2& m : {mirror1, mirror2}) {
        drawPolygon(fb, rotatedRect(m, halfLen, halfThick, mirrorAngle), colors::White);
        boundaryFill(fb, static_cast<int>(m.x), static_cast<int>(m.y), silver, colors::White);
    }

    // --- 7. outlines on top of the fills
    drawPolygon(fb, bench.prism, colors::Cyan);
    for (const Lens& l : bench.lenses) {
        drawCircle(fb, static_cast<int>(l.center.x), static_cast<int>(l.center.y), static_cast<int>(l.radius), colors::Cyan);
        drawCircle(fb, static_cast<int>(l.center.x), static_cast<int>(l.center.y), static_cast<int>(l.radius) - 1, colors::Cyan);
    }

    // optical axis of the first leg (dashed) and the detector screen
    drawDashedLine(fb, Vec2{110, 450}, Vec2{370, 450}, 6, 6, colors::Grey);
    drawThickLine(fb, bench.detector.a, bench.detector.b, 4, Color{200, 200, 60});

    // --- 8. crisp anti-aliased beams (Wu)
    for (const Beam& b : beams) {
        Vec2 a = b.a, e = b.b;
        if (clipLineCohenSutherland(a, e, kView)) drawLineWu(fb, a, e, b.c);
    }

    // --- 9. detector hits + aperture rings (circles/ellipses)
    double yMin = 1e9, yMax = -1e9;
    for (std::size_t i = 0; i < bands.size(); ++i) {
        if (!results[i].hitDetector) continue;
        const Vec2 p = results[i].detectorPoint;
        fillCircle(fb, static_cast<int>(std::lround(p.x)) - 10, static_cast<int>(std::lround(p.y)), 5, bands[i].color);
        drawCircle(fb, static_cast<int>(std::lround(p.x)) - 10, static_cast<int>(std::lround(p.y)), 6, colors::White);
        yMin = std::min(yMin, p.y);
        yMax = std::max(yMax, p.y);
    }
    if (yMax >= yMin) {
        const int cy = static_cast<int>(std::lround((yMin + yMax) / 2.0));
        const int ry = static_cast<int>(std::lround((yMax - yMin) / 2.0)) + 20;
        drawEllipse(fb, 895, cy, 12, ry, colors::Grey);
        drawEllipse(fb, 895, cy, 20, ry + 14, Color{70, 74, 90});
    }

    // --- 10. bench rail + legend (Lines / scanline)
    drawThickLine(fb, Vec2{40, 552}, Vec2{890, 552}, 5, colors::Grey);
    for (int x = 40; x <= 880; x += 40) drawLineBresenham(fb, x, 543, x, 561, colors::Grey);

    for (std::size_t i = 0; i < bands.size(); ++i) {
        const double x = 50.0 + 34.0 * static_cast<double>(i);
        fillPolygon(fb, {{x, 505}, {x + 26, 505}, {x + 26, 525}, {x, 525}}, bands[i].color);
    }

    // viewport border
    drawPolygon(fb, {{kView.xmin, kView.ymin}, {kView.xmax, kView.ymin}, {kView.xmax, kView.ymax}, {kView.xmin, kView.ymax}},
                colors::Grey);

    // --- output
    if (!fb.saveBMP("output/integrated_demo.bmp")) {
        std::puts("FAILED to write output/integrated_demo.bmp");
        return 1;
    }

    // a few numbers from the geometry helpers, as a sanity check
    double redPath = 0.0;
    for (const Beam& b : beams) { (void)b; }
    {
        // path length of the red ray = shared white part + its own coloured part
        std::vector<Beam> redOnly;
        trace(bench, Ray{laserOut, Vec2{1, 0}}, bands[0].n, bands[0].color, true, redOnly);
        for (const Beam& b : redOnly) {
            Vec2 a = b.a, e = b.b;
            if (clipLineCohenSutherland(a, e, kView)) redPath += length(Segment{a, e});
        }
    }

    std::puts("OK - wrote output/integrated_demo.bmp");
    std::printf("prism area (shoelace): %.1f px^2\n", std::fabs(polygonArea(bench.prism)));
    std::printf("red beam path length (inside viewport): %.1f px\n", redPath);
    for (std::size_t i = 0; i < bands.size(); ++i) {
        if (results[i].hitDetector)
            std::printf("  %-6s (n=%.2f) hits detector at y = %.1f\n", bands[i].name, bands[i].n, results[i].detectorPoint.y);
        else
            std::printf("  %-6s (n=%.2f) missed the detector\n", bands[i].name, bands[i].n);
    }
    return 0;
}