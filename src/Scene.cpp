#include "optibench/Scene.h"

#include <algorithm>
#include <cmath>

#include "optibench/Tracer.h"
#include "optibench/circle_ellipse.h"
#include "optibench/raster_ops.h"

namespace ob {
    namespace {
        std::uint8_t clampByte(double v) {
            return static_cast<std::uint8_t>(std::lround(std::clamp(v, 0.0, 255.0)));
        }

        // darkens (k < 1) or brightens (k > 1) a colour
        Color shade(Color c, double k) { return {clampByte(c.r * k), clampByte(c.g * k), clampByte(c.b * k)}; }

        // blends a colour towards white
        Color towardsWhite(Color c, double t) {
            return {clampByte(c.r + (255 - c.r) * t), clampByte(c.g + (255 - c.g) * t), clampByte(c.b + (255 - c.b) * t)};
        }

        void glowSegment(Framebuffer& fb, Vec2 a, Vec2 b, Color color, double radius, double strength) {
            const double dx = b.x - a.x, dy = b.y - a.y;
            const double length2 = dx * dx + dy * dy;
            const int minX = std::max(0, static_cast<int>(std::floor(std::min(a.x, b.x) - radius)));
            const int maxX = std::min(fb.width() - 1, static_cast<int>(std::ceil(std::max(a.x, b.x) + radius)));
            const int minY = std::max(0, static_cast<int>(std::floor(std::min(a.y, b.y) - radius)));
            const int maxY = std::min(fb.height() - 1, static_cast<int>(std::ceil(std::max(a.y, b.y) + radius)));
            for (int y = minY; y <= maxY; ++y) for (int x = minX; x <= maxX; ++x) {
                const double t = length2 > 0.0
                    ? std::clamp(((x - a.x) * dx + (y - a.y) * dy) / length2, 0.0, 1.0) : 0.0;
                const double distance = std::hypot(x - (a.x + t * dx), y - (a.y + t * dy));
                if (distance < radius) {
                    const double falloff = 1.0 - distance / radius;
                    blendPixel(fb, x, y, color, strength * falloff * falloff);
                }
            }
        }

        /**
         * draws one beam path:
         *      - the part Cohen-Sutherland removes (outside the viewport) is drawn faintly
         *      - a soft glow band under the beam is clipped with Sutherland-Hodgman and scanline-filled
         *      - the bright core is an anti-aliased (Wu) line
         */
        void drawBeam(Framebuffer& fb, const BeamPath& path, Color beam, const ClipRect& view) {
            const Color faint = shade(beam, 0.30);

            for (const BeamSegment& s : path.segments) {
                const Color core = s.inGlass ? towardsWhite(beam, 0.45) : beam;

                Vec2 a = s.a, b = s.b;
                if (!clipLineCohenSutherland(a, b, view)) {
                    drawLineWu(fb, s.a, s.b, faint);            // completely outside
                    continue;
                }
                if ((a - s.a).length() > 0.5) drawLineWu(fb, s.a, a, faint);
                if ((b - s.b).length() > 0.5) drawLineWu(fb, b, s.b, faint);

                if ((b - a).length() > 1.0) glowSegment(fb, a, b, core, 7.0, 0.30);
                drawLineWu(fb, a, b, core);
            }

            // light spot where the beam lands on a screen: nested ellipses, brightest in the middle
            if (path.hitScreen) {
                const int x = static_cast<int>(std::lround(path.end.x));
                const int y = static_cast<int>(std::lround(path.end.y));
                glowSegment(fb, {static_cast<double>(x), static_cast<double>(y)},
                            {static_cast<double>(x), static_cast<double>(y)}, beam, 18.0, 0.48);
            }
        }
    }


    OpticalElement* Scene::find(const std::string& name) {
        for (auto& e : elements_) if (e->name == name) return e.get();
        return nullptr;
    }

    const OpticalElement* Scene::find(const std::string& name) const {
        for (const auto& e : elements_) if (e->name == name) return e.get();
        return nullptr;
    }

    Scene::Intersection Scene::nearestHit(const Ray& ray) const {
        Intersection best;
        for (const auto& e : elements_) {
            const Hit h = e->intersect(ray);
            if (h.hit && (!best.element || h.t < best.hit.t)) {
                best.hit = h;
                best.element = e.get();
            }
        }
        return best;
    }

    void Scene::render(Framebuffer& fb) const {
        const ClipRect& v = viewport_;

        // 1. margin + bench surface: outline the viewport, then flood-fill its inside
        fb.clear(marginColor);
        const Polygon frame{{v.xmin, v.ymin}, {v.xmax, v.ymin}, {v.xmax, v.ymax}, {v.xmin, v.ymax}};
        drawPolygon(fb, frame, colors::Grey);
        floodFill(fb, static_cast<int>((v.xmin + v.xmax) / 2), static_cast<int>((v.ymin + v.ymax) / 2), panelColor);

        // 2. grid (Bresenham)
        if (gridSpacing > 0) {
            for (double x = v.xmin; x <= v.xmax; x += gridSpacing) drawLine(fb, {x, v.ymin}, {x, v.ymax}, gridColor);
            for (double y = v.ymin; y <= v.ymax; y += gridSpacing) drawLine(fb, {v.xmin, y}, {v.xmax, y}, gridColor);
        }

        // 3. elements
        for (const auto& e : elements_) e->draw(fb);

        // 4. beams, one per laser
        for (const LaserSource* src : sources_) {
            const BeamPath path = traceBeam(*this, src->emissionRay());
            drawBeam(fb, path, src->beamColor(), v);
        }

        // 5. crisp border on top
        drawPolygon(fb, frame, colors::Grey);
    }
}
