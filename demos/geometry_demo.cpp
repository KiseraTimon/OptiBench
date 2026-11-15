/*
 * Special Notes: Geometry Demo
 *
 * Six panels showing the geometry module (ob::geom) on the optical bench:
 *
 *   A (top left)       ear-clipping triangulation of a concave polygon: every triangle shaded,
 *                      the outline on top
 *   B (top middle)     convex hull (monotone chain) of a point cloud: hull vertices ringed in
 *                      yellow, every other point inside the shaded hull
 *   C (top right)      point location and signed distance for a concave polygon: blue inside,
 *                      warm outside, a contour every 12 px, the outline in white
 *   D (bottom left)    laser rays refracting through a CONCAVE glass prism (n = 1.5), using
 *                      geom::rayPolygon's outward normals; grey ticks are the normals at each hit
 *   E (bottom middle)  parallel rays through a thick biconvex geom::Lens; the cyan cross marks the
 *                      lensmaker back focus. Edge rays cross sooner: spherical aberration.
 *   F (bottom right)   overlap tests: shapes that overlap something are red, the rest green;
 *                      yellow arrows are SAT minimum translation vectors; dashed boxes are AABBs.
 *                      The block resting in the L's notch has an overlapping AABB but no overlap.
 *
 * Running this file:
 *      run this command from the root after building all files
 *      ./build/demo_geometry          (.\build\demo_geometry.exe on Windows)
 * Writes output/geometry_demo.bmp
 */

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <random>
#include <vector>

#include "optibench/Color.h"
#include "optibench/Framebuffer.h"
#include "optibench/Line.h"
#include "optibench/circle_ellipse.h"
#include "optibench/clip_fill.h"
#include "optibench/geometry/Geometry.h"
#include "optibench/optics.h"
#include "optibench/raster_ops.h"

using namespace ob;
using namespace ob::geom;

namespace {
    using Poly = std::vector<Vec2>;

    constexpr int kWidth = 1200;
    constexpr int kHeight = 720;

    // the six panels, 3 x 2
    const ClipRect kPanels[6] = {
        {10, 10, 396, 355}, {407, 10, 793, 355}, {804, 10, 1190, 355},
        {10, 365, 396, 710}, {407, 365, 793, 710}, {804, 365, 1190, 710},
    };

    constexpr Color kPanelBg{22, 26, 40};
    constexpr Color kBorder{70, 76, 96};
    constexpr Color kNormalGrey{150, 150, 165};
    constexpr Color kOrange{255, 150, 40};
    constexpr Color kOverlapRed{240, 80, 80};
    constexpr Color kClearGreen{90, 220, 130};

    Vec2 origin(const ClipRect& r) { return {r.xmin, r.ymin}; }

    Poly offset(const Poly& p, Vec2 d) {
        Poly out;
        out.reserve(p.size());
        for (const Vec2& v : p) out.push_back(v + d);
        return out;
    }

    // a line trimmed to a panel (Cohen-Sutherland from the clipping module), anti-aliased
    void line(Framebuffer& fb, Vec2 a, Vec2 b, Color c, const ClipRect& panel) {
        if (clipLineCohenSutherland(a, b, panel)) drawLineWu(fb, a, b, c);
    }

    void outline(Framebuffer& fb, const Poly& p, Color c, const ClipRect& panel) {
        for (std::size_t i = 0; i < p.size(); ++i) line(fb, p[i], p[(i + 1) % p.size()], c, panel);
    }

    // a translucent fill, using the geometry module's own point-in-polygon test per pixel
    void fillTranslucent(Framebuffer& fb, const Poly& p, Color c, double alpha, const ClipRect& panel) {
        const Aabb box = boundsOf(p);
        const int x0 = static_cast<int>(std::max(box.min.x, panel.xmin));
        const int x1 = static_cast<int>(std::min(box.max.x, panel.xmax));
        const int y0 = static_cast<int>(std::max(box.min.y, panel.ymin));
        const int y1 = static_cast<int>(std::min(box.max.y, panel.ymax));
        for (int y = y0; y <= y1; ++y) {
            for (int x = x0; x <= x1; ++x) {
                if (contains(p, {x + 0.5, y + 0.5}, 0.0)) blendPixel(fb, x, y, c, alpha);
            }
        }
    }

    void dot(Framebuffer& fb, Vec2 p, int r, Color c) {
        fillCircle(fb, static_cast<int>(std::lround(p.x)), static_cast<int>(std::lround(p.y)), r, c);
    }

    void arrow(Framebuffer& fb, Vec2 from, Vec2 to, Color c, const ClipRect& panel) {
        line(fb, from, to, c, panel);
        const Vec2 d = (to - from).normalized();
        if (d.length() == 0.0) return;
        const Vec2 side = d.perp() * 4.0;
        line(fb, to, to - d * 8.0 + side, c, panel);
        line(fb, to, to - d * 8.0 - side, c, panel);
    }

    void dashedBox(Framebuffer& fb, const Aabb& b, Color c) {
        const Vec2 p[4] = {b.min, {b.max.x, b.min.y}, b.max, {b.min.x, b.max.y}};
        for (int i = 0; i < 4; ++i) drawDashedLine(fb, p[i], p[(i + 1) % 4], 4, 4, c);
    }

    void panelFrame(Framebuffer& fb, const ClipRect& r) {
        const Poly frame = rectangle(boundsOf(Segment{{r.xmin, r.ymin}, {r.xmax, r.ymax}}));
        fillPolygon(fb, frame, kPanelBg);
        drawPolygon(fb, frame, kBorder);
    }

    // a hue wheel for telling neighbouring triangles apart
    Color hue(int i) {
        static const Color wheel[] = {{235, 90, 90}, {240, 170, 70}, {220, 220, 90},
                                      {100, 210, 120}, {80, 180, 230}, {150, 120, 235}};
        return wheel[i % 6];
    }

    // --- A: triangulation of a concave polygon
    void panelTriangulation(Framebuffer& fb, const ClipRect& panel) {
        const Poly shape = offset({{40, 300}, {80, 60}, {150, 210}, {190, 40}, {240, 200}, {300, 70},
                                   {350, 300}, {280, 260}, {250, 320}, {190, 250}, {140, 320}, {110, 240}},
                                  origin(panel));
        const std::vector<Triangle> tris = triangulate(shape);
        for (std::size_t i = 0; i < tris.size(); ++i) {
            const Poly t{shape[tris[i][0]], shape[tris[i][1]], shape[tris[i][2]]};
            fillPolygon(fb, t, blendOver(hue(static_cast<int>(i)), kPanelBg, 0.45));
            outline(fb, t, blendOver(hue(static_cast<int>(i)), kPanelBg, 0.9), panel);
        }
        outline(fb, shape, colors::White, panel);
        for (const Vec2& v : shape) dot(fb, v, 2, colors::White);
        std::printf("A  triangulation: %zu vertices -> %zu triangles, area %.1f = sum %.1f\n",
                    shape.size(), tris.size(), area(shape), [&] {
                        double s = 0.0;
                        for (const Triangle& t : tris) s += area(Poly{shape[t[0]], shape[t[1]], shape[t[2]]});
                        return s;
                    }());
    }

    // --- B: convex hull of a point cloud
    void panelHull(Framebuffer& fb, const ClipRect& panel) {
        std::mt19937 gen(7u);
        std::normal_distribution<double> gx(193.0, 60.0), gy(172.0, 50.0);
        Poly pts;
        while (pts.size() < 70) {
            const Vec2 p{gx(gen), gy(gen)};
            if (p.x > 20 && p.x < 366 && p.y > 20 && p.y < 325) pts.push_back(p + origin(panel));
        }
        const Poly hull = convexHull(pts);
        fillTranslucent(fb, hull, colors::Yellow, 0.12, panel);
        outline(fb, hull, colors::Yellow, panel);
        for (const Vec2& p : pts) dot(fb, p, 2, colors::Cyan);
        for (const Vec2& v : hull) {
            drawCircle(fb, static_cast<int>(std::lround(v.x)), static_cast<int>(std::lround(v.y)), 5, colors::Yellow);
        }
        std::printf("B  convex hull: %zu points -> %zu hull vertices, convex = %s\n",
                    pts.size(), hull.size(), isConvex(hull) ? "yes" : "no");
    }

    // --- C: point location / signed distance field of a concave polygon
    void panelDistance(Framebuffer& fb, const ClipRect& panel) {
        const Poly shape = offset({{60, 60}, {330, 60}, {330, 290}, {250, 290}, {250, 140},
                                   {140, 140}, {140, 230}, {200, 230}, {200, 290}, {60, 290}},
                                  origin(panel));
        for (int y = static_cast<int>(panel.ymin) + 1; y < static_cast<int>(panel.ymax); ++y) {
            for (int x = static_cast<int>(panel.xmin) + 1; x < static_cast<int>(panel.xmax); ++x) {
                const double d = signedDistance(shape, {x + 0.5, y + 0.5});
                const double fade = std::exp(-std::fabs(d) / 70.0);
                Color c = d < 0.0 ? blendOver(Color{70, 130, 255}, kPanelBg, 0.25 + 0.5 * (1.0 - fade))
                                  : blendOver(kOrange, kPanelBg, 0.45 * fade);
                if (std::fmod(std::fabs(d), 12.0) < 1.0) c = blendOver(colors::White, c, 0.18);   // contours
                fb.setPixel(x, y, c);
            }
        }
        outline(fb, shape, colors::White, panel);

        // a few probe points, ringed by what locate() says
        const Vec2 probes[] = {{100, 100}, {195, 185}, {290, 200}, {170, 260}, {30, 320}, {250, 215}};
        for (const Vec2& q : probes) {
            const Vec2 p = q + origin(panel);
            const PointLocation where = locate(shape, p);
            const Color c = where == PointLocation::Inside ? colors::Cyan
                          : where == PointLocation::Boundary ? colors::White : kOrange;
            dot(fb, p, 3, c);
            drawCircle(fb, static_cast<int>(p.x), static_cast<int>(p.y), 7, c);
        }
    }

    // --- D: refraction through a concave glass prism
    void panelConcavePrism(Framebuffer& fb, const ClipRect& panel) {
        const Poly glass = offset({{130, 50}, {270, 50}, {270, 300}, {130, 300}, {210, 175}}, origin(panel));
        fillTranslucent(fb, glass, Color{60, 120, 230}, 0.30, panel);
        outline(fb, glass, colors::Cyan, panel);

        constexpr double n = 1.5;
        for (int k = 0; k < 9; ++k) {
            // +0.5 puts horizontal beams on pixel centres (otherwise Wu splits them over two rows)
            Ray ray{origin(panel) + Vec2{12.0, 70.5 + 26.0 * k}, {1.0, 0.0}};
            bool inside = false;
            const Color beam = blendOver(colors::Red, colors::Yellow, k / 8.0);
            for (int bounce = 0; bounce < 12; ++bounce) {
                const Hit h = rayPolygon(ray, glass);
                if (!h.hit) {
                    line(fb, ray.origin, ray.origin + ray.direction * 600.0, beam, panel);
                    break;
                }
                line(fb, ray.origin, h.point, inside ? blendOver(colors::White, beam, 0.45) : beam, panel);
                line(fb, h.point - h.normal * 10.0, h.point + h.normal * 10.0, kNormalGrey, panel);

                // the outward normal says which side we are on; refract() wants it facing the ray
                const bool entering = ray.direction.dot(h.normal) < 0.0;
                const Vec2 facing = entering ? h.normal : -h.normal;
                Vec2 next;
                if (refract(ray.direction, facing, entering ? 1.0 : n, entering ? n : 1.0, next)) {
                    inside = entering;
                } else {
                    next = reflect(ray.direction, facing);      // total internal reflection
                }
                ray = Ray{h.point + next * 1e-6, next};
            }
        }
    }

    // --- E: a thick biconvex lens and its lensmaker focus
    void panelLens(Framebuffer& fb, const ClipRect& panel) {
        const Lens lens(160.0, 160.0, 36.0);
        constexpr double n = 1.5;
        const Vec2 centre = origin(panel) + Vec2{90.0, 172.0};

        const Poly shape = offset(lens.outline(64), centre);
        fillTranslucent(fb, shape, Color{60, 120, 230}, 0.35, panel);
        outline(fb, shape, colors::Cyan, panel);
        line(fb, {panel.xmin + 4, centre.y}, {panel.xmax - 4, centre.y}, kBorder, panel);     // optical axis

        for (int k = -6; k <= 6; ++k) {
            if (k == 0) continue;
            const double y = k * lens.halfHeight() * 0.9 / 6.0;
            const Color beam = std::abs(k) >= 5 ? kOrange : colors::Red;
            Ray local{{-200.0, y}, {1.0, 0.0}};       // traced in the lens's own frame
            const Hit in = lens.intersect(local);
            if (!in.hit) continue;
            Vec2 bent, out;
            if (!refract(local.direction, in.normal, 1.0, n, bent)) continue;
            const Hit ex = lens.intersect(Ray{in.point, bent});
            if (!ex.hit || !refract(bent, -ex.normal, n, 1.0, out)) continue;

            line(fb, centre + Vec2{-90.0 + 6.0, y}, centre + in.point, beam, panel);
            line(fb, centre + in.point, centre + ex.point, blendOver(colors::White, beam, 0.45), panel);
            line(fb, centre + ex.point, centre + ex.point + out * 400.0, beam, panel);
        }

        // predicted back focus
        const Vec2 focus = centre + Vec2{lens.thickness() / 2.0 + lens.backFocalDistance(n), 0.0};
        line(fb, focus - Vec2{8, 0}, focus + Vec2{8, 0}, colors::Cyan, panel);
        line(fb, focus - Vec2{0, 8}, focus + Vec2{0, 8}, colors::Cyan, panel);
        std::printf("E  lens R1 = R2 = 160, d = 36, n = 1.5: f = %.2f, back focal distance = %.2f, rim = %.2f\n",
                    lens.focalLength(n), lens.backFocalDistance(n), lens.halfHeight());
    }

    // --- F: overlap tests and minimum translation vectors
    void panelOverlap(Framebuffer& fb, const ClipRect& panel) {
        const Vec2 o = origin(panel);
        struct Shape { Poly poly; bool convex; };
        const std::vector<Shape> shapes = {
            {offset(regularPolygon({70, 80}, 45, 6, 0.3), o), true},
            {offset(regularPolygon({140, 95}, 40, 4, 0.2), o), true},
            {offset(regularPolygon({300, 75}, 40, 5), o), true},
            {offset({{30, 180}, {200, 180}, {200, 215}, {65, 215}, {65, 320}, {30, 320}}, o), false},    // an L
            {offset({{66, 216}, {150, 216}, {150, 300}, {66, 300}}, o), true},     // resting in the L's notch
            {offset({{240, 170}, {350, 170}, {350, 260}, {240, 260}}, o), true},
            {offset(regularPolygon({300, 290}, 38, 3, -kPi / 2), o), true},
        };

        std::vector<bool> hit(shapes.size(), false);
        for (std::size_t i = 0; i < shapes.size(); ++i) {
            for (std::size_t j = i + 1; j < shapes.size(); ++j) {
                if (polygonsOverlap(shapes[i].poly, shapes[j].poly)) hit[i] = hit[j] = true;
            }
        }
        for (std::size_t i = 0; i < shapes.size(); ++i) {
            const Color c = hit[i] ? kOverlapRed : kClearGreen;
            dashedBox(fb, boundsOf(shapes[i].poly), kBorder);
            fillTranslucent(fb, shapes[i].poly, c, 0.25, panel);
            outline(fb, shapes[i].poly, c, panel);
        }
        // minimum translation vectors for the overlapping convex pairs, drawn on top of the fills
        for (std::size_t i = 0; i < shapes.size(); ++i) {
            for (std::size_t j = i + 1; j < shapes.size(); ++j) {
                if (!(shapes[i].convex && shapes[j].convex)) continue;
                const Separation s = separateConvex(shapes[i].poly, shapes[j].poly);
                if (!s.overlapping) continue;
                const Vec2 c = centroid(shapes[j].poly);
                arrow(fb, c, c + s.normal * std::max(s.depth, 12.0), colors::Yellow, panel);
            }
        }
        std::printf("F  L-shape vs block in its notch: AABBs overlap = %s, shapes overlap = %s\n",
                    overlaps(boundsOf(shapes[3].poly), boundsOf(shapes[4].poly)) ? "yes" : "no",
                    polygonsOverlap(shapes[3].poly, shapes[4].poly) ? "yes" : "no");
    }
}

int main() {
    Framebuffer fb(kWidth, kHeight, colors::BenchDark);
    for (const ClipRect& r : kPanels) panelFrame(fb, r);

    panelTriangulation(fb, kPanels[0]);
    panelHull(fb, kPanels[1]);
    panelDistance(fb, kPanels[2]);
    panelConcavePrism(fb, kPanels[3]);
    panelLens(fb, kPanels[4]);
    panelOverlap(fb, kPanels[5]);

    // crisp borders on top of anything that ran up to them
    for (const ClipRect& r : kPanels) drawPolygon(fb, rectangle(boundsOf(Segment{{r.xmin, r.ymin}, {r.xmax, r.ymax}})), kBorder);

    const char* path = "output/geometry_demo.bmp";
    if (!fb.saveBMP(path)) {
        std::printf("could not write %s (run from the repository root)\n", path);
        return 1;
    }
    std::printf("wrote %s\n", path);
    return 0;
}
