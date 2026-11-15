#include "optibench/Scene.h"

#include <algorithm>
#include <cmath>

#include "optibench/Tracer.h"
#include "optibench/circle_ellipse.h"
#include "optibench/geometry/Intersect.h"
#include "optibench/geometry/Overlap.h"
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
            // broad phase: a box the ray misses, or reaches only past the best hit, cannot hold a nearer hit.
            // The box is grown by a hair so a hit exactly on its edge is never lost to rounding.
            const geom::Aabb box = e->bounds();
            if (!box.empty()) {
                const auto span = geom::rayAabb(ray, geom::inflate(box, 1e-6));
                if (!span || (best.element && span->tNear > best.hit.t)) continue;
            }

            const Hit h = e->intersect(ray);
            if (h.hit && (!best.element || h.t < best.hit.t)) {
                best.hit = h;
                best.element = e.get();
            }
        }
        return best;
    }

    const OpticalElement* Scene::solidAt(const Vec2& p) const {
        for (const auto& e : elements_) {
            if (e->isSolid() && e->material.type == SurfaceType::Glass && e->contains(p)) return e.get();
        }
        return nullptr;
    }

    std::vector<std::pair<const OpticalElement*, const OpticalElement*>> Scene::overlappingPairs() const {
        std::vector<std::pair<const OpticalElement*, const OpticalElement*>> pairs;
        std::vector<std::vector<Vec2>> outlines;
        outlines.reserve(elements_.size());
        for (const auto& e : elements_) outlines.push_back(e->outline());

        for (std::size_t i = 0; i < elements_.size(); ++i) {
            for (std::size_t j = i + 1; j < elements_.size(); ++j) {
                if (outlines[i].size() < 3 || outlines[j].size() < 3) continue;
                if (geom::polygonsOverlap(outlines[i], outlines[j])) pairs.emplace_back(elements_[i].get(), elements_[j].get());
            }
        }
        return pairs;
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
