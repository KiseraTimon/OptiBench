#include "Studio.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <span>
#include <string>

#include "optibench/Line.h"
#include "optibench/Mat3.h"
#include "optibench/circle_ellipse.h"
#include "optibench/geometry/Geometry.h"
#include "optibench/raster_ops.h"
#include "optibench/refraction.h"
#include "optibench/render/Parallel.h"
#include "optibench/render/Text.h"

namespace ob::app {
    using render::LinearRgb;
    using render::toLinear;
    using render::toSrgb8;

    namespace {
        constexpr ClipRect kBench = Layout::kBench;
        constexpr int kW = Layout::kWidth, kH = Layout::kHeight;

        // palette (sRGB)
        constexpr Color kInk{226, 232, 245};        // primary text
        constexpr Color kMuted{128, 140, 168};      // secondary text
        constexpr Color kFaint{70, 80, 104};
        constexpr Color kChip{10, 13, 22};          // label background
        constexpr Color kCyan{110, 225, 255};
        constexpr Color kAmber{255, 184, 76};
        constexpr Color kGood{120, 230, 150};
        constexpr Color kBad{255, 96, 96};
        constexpr Color kFocus{255, 226, 110};

        double deg(double rad) { return rad * 180.0 / kPi; }

        std::string fmt(const char* format, double v) {
            char buf[64];
            std::snprintf(buf, sizeof buf, format, v);
            return buf;
        }

        // ------------------------------------------------------------ pixel helpers (linear light)

        void blend(Framebuffer& fb, int x, int y, const LinearRgb& c, float a) {
            if (x < 0 || y < 0 || x >= fb.width() || y >= fb.height() || a <= 0.0f) return;
            a = std::min(a, 1.0f);
            fb.setPixel(x, y, toSrgb8(toLinear(fb.getPixel(x, y)) * (1.0f - a) + c * a));
        }

        void fillRect(Framebuffer& fb, int x0, int y0, int x1, int y1, const LinearRgb& c, float a = 1.0f) {
            for (int y = std::max(0, y0); y < std::min(fb.height(), y1); ++y)
                for (int x = std::max(0, x0); x < std::min(fb.width(), x1); ++x) blend(fb, x, y, c, a);
        }

        /**
         * fills a polygon with a per-pixel shader and anti-aliased edges: coverage comes from the
         * polygon's signed distance (geometry module), so edges are smooth at any angle.
         * shade(p, depth) returns {colour, alpha}; depth is the distance into the body.
         */
        template <class Shade>
        void shadePolygon(Framebuffer& fb, std::span<const Vec2> poly, Shade shade) {
            if (poly.size() < 3) return;
            const geom::Aabb box = geom::boundsOf(poly);
            const int x0 = std::max(0, static_cast<int>(std::floor(box.min.x)) - 1);
            const int x1 = std::min(fb.width() - 1, static_cast<int>(std::ceil(box.max.x)) + 1);
            const int y0 = std::max(0, static_cast<int>(std::floor(box.min.y)) - 1);
            const int y1 = std::min(fb.height() - 1, static_cast<int>(std::ceil(box.max.y)) + 1);
            for (int y = y0; y <= y1; ++y)
                for (int x = x0; x <= x1; ++x) {
                    const Vec2 p{x + 0.5, y + 0.5};
                    const double sd = geom::signedDistance(poly, p);
                    const double coverage = std::clamp(0.5 - sd, 0.0, 1.0);
                    if (coverage <= 0.0) continue;
                    const auto [c, a] = shade(p, std::max(0.0, -sd));
                    blend(fb, x, y, c, static_cast<float>(a * coverage));
                }
        }

        // an anti-aliased ring (event markers)
        void ring(Framebuffer& fb, Vec2 c, double r, Color color, double width = 1.2, double opacity = 1.0) {
            const LinearRgb lc = toLinear(color);
            const int reach = static_cast<int>(std::ceil(r + width + 1));
            for (int y = static_cast<int>(c.y) - reach; y <= static_cast<int>(c.y) + reach; ++y)
                for (int x = static_cast<int>(c.x) - reach; x <= static_cast<int>(c.x) + reach; ++x) {
                    const double d = std::abs((Vec2{x + 0.5, y + 0.5} - c).length() - r);
                    const double a = std::clamp(width / 2.0 + 0.5 - d, 0.0, 1.0) * opacity;
                    if (a > 0.0) blend(fb, x, y, lc, static_cast<float>(a));
                }
        }

        void disc(Framebuffer& fb, Vec2 c, double r, Color color, double opacity = 1.0) {
            const LinearRgb lc = toLinear(color);
            const int reach = static_cast<int>(std::ceil(r + 1));
            for (int y = static_cast<int>(c.y) - reach; y <= static_cast<int>(c.y) + reach; ++y)
                for (int x = static_cast<int>(c.x) - reach; x <= static_cast<int>(c.x) + reach; ++x) {
                    const double a = std::clamp(r + 0.5 - (Vec2{x + 0.5, y + 0.5} - c).length(), 0.0, 1.0) * opacity;
                    if (a > 0.0) blend(fb, x, y, lc, static_cast<float>(a));
                }
        }

        void focusMark(Framebuffer& fb, Vec2 p, Color c) {
            ring(fb, p, 5.0, c, 1.2, 0.9);
            drawLineWu(fb, p - Vec2{9, 0}, p - Vec2{3, 0}, c);
            drawLineWu(fb, p + Vec2{3, 0}, p + Vec2{9, 0}, c);
            drawLineWu(fb, p - Vec2{0, 9}, p - Vec2{0, 3}, c);
            drawLineWu(fb, p + Vec2{0, 3}, p + Vec2{0, 9}, c);
        }

        // CAD-style corner brackets for a bounding box
        void brackets(Framebuffer& fb, const geom::Aabb& b, Color c) {
            if (b.empty()) return;
            const double l = std::min(8.0, std::min(b.max.x - b.min.x, b.max.y - b.min.y) / 3.0);
            const Vec2 corners[4] = {b.min, {b.max.x, b.min.y}, b.max, {b.min.x, b.max.y}};
            const Vec2 sx[4] = {{1, 0}, {-1, 0}, {-1, 0}, {1, 0}};
            const Vec2 sy[4] = {{0, 1}, {0, 1}, {0, -1}, {0, -1}};
            for (int i = 0; i < 4; ++i) {
                drawLine(fb, corners[i], corners[i] + sx[i] * l, c);
                drawLine(fb, corners[i], corners[i] + sy[i] * l, c);
            }
        }

        // a horizontal strip of the visible spectrum (the brand accent under the header)
        void spectrumStrip(Framebuffer& fb, int x0, int x1, int y, int h, float gain) {
            for (int x = x0; x < x1; ++x) {
                const double nm = 400.0 + 300.0 * (x - x0) / std::max(1, x1 - x0 - 1);
                const LinearRgb c = render::laserColor(nm) * gain;
                for (int k = 0; k < h; ++k) fb.setPixel(x, y + k, toSrgb8(c));
            }
        }

        // ------------------------------------------------------------ sprites

        SpriteSheet makePhotonSheet(Color c) {
            constexpr int size = 15, frames = 8;
            Framebuffer sheet(size * frames, size, colors::Black);      // black = transparent (colour key)
            for (int f = 0; f < frames; ++f) {
                const Vec2 centre{f * size + size / 2.0, size / 2.0};
                fillCircle(sheet, static_cast<int>(centre.x), static_cast<int>(centre.y), 1, c);
                for (int k = 0; k < 4; ++k) {
                    const double a = (f / static_cast<double>(frames)) * (kPi / 2.0) + k * kPi / 2.0;
                    const Vec2 dir{std::cos(a), std::sin(a)};
                    drawLine(sheet, centre + dir * 3.0, centre + dir * 6.0, c);
                }
            }
            return SpriteSheet(std::move(sheet), size, size, colors::Black);
        }

        // point at arc length s along a path's segments (clamped to its end)
        Vec2 pointAlong(const BeamPath& path, double s) {
            for (const BeamSegment& seg : path.segments) {
                const double len = (seg.b - seg.a).length();
                if (s <= len) return seg.a + (seg.b - seg.a) * (len > 0 ? s / len : 0.0);
                s -= len;
            }
            return path.end;
        }

        // the part of a path inside the bench (beams that leave are cut at the edge)
        double visibleLength(const BeamPath& path) {
            double total = 0.0;
            for (const BeamSegment& seg : path.segments) {
                Vec2 a = seg.a, b = seg.b;
                if (!clipLineCohenSutherland(a, b, kBench)) break;
                total += (b - a).length();
                if ((b - seg.b).length() > 1e-6) break;     // left the bench
            }
            return total;
        }

        bool isCentral(const TracedRay& r) { return r.index == r.source->rays / 2; }

        // short names for the HUD
        std::string displayName(const std::string& element) {
            if (element == "mirrorLow") return "M1";
            if (element == "mirrorTop") return "M2";
            if (element == "mirrorWhite") return "M3";
            if (element == "notched") return "notch";
            if (element == "floor") return "screen";
            if (element == "screen") return "spectrum";
            return element;
        }

        double lensBfd(const Scene& scene) {
            const auto* lens = scene.findAs<ThickLensElement>("lens");
            return lens ? lens->shape().backFocalDistance(lens->material.refractiveIndex) : 0.0;
        }

        Color sourceColor(const SourceSpec& s) {
            if (s.kind == SourceSpec::Kind::WhiteLight) return Color{235, 238, 245};
            return toSrgb8(render::laserColor(s.wavelengthNm));
        }

        // glass tint from its dispersion: crowns (high Abbe number) cyan, flints violet-blue
        LinearRgb glassTint(const Material& m) {
            const double v = m.abbeNumber > 0 ? m.abbeNumber : 60.0;
            const double u = std::clamp((v - 20.0) / 45.0, 0.0, 1.0);       // 0 flint .. 1 crown
            return LinearRgb{static_cast<float>(0.30 - 0.22 * u), static_cast<float>(0.32 + 0.30 * u), 1.0f};
        }

        std::string glassName(const Material& m) {
            if (std::abs(m.refractiveIndex - 1.5168) < 1e-4) return "BK7";
            if (std::abs(m.refractiveIndex - 1.7847) < 1e-4) return "SF11";
            if (std::abs(m.refractiveIndex - 1.9229) < 1e-4) return "N-SF66";
            if (std::abs(m.refractiveIndex - 1.6200) < 1e-4) return "F2";
            return "n " + fmt("%.3f", m.refractiveIndex);
        }
    }

    // =====================================================================================

    Studio::Studio(StudioOptions options, std::vector<KeyframeTrack> tracks)
        : opt_(options), tracks_(std::move(tracks)), backdrop_(kW, kH), light_(kW, kH) {
        photons_ = {makePhotonSheet(Color{255, 150, 140}), makePhotonSheet(Color{255, 255, 255}),
                    makePhotonSheet(Color{150, 190, 255})};
        buildBackdrop();
    }

    void Studio::buildBackdrop() {
        Framebuffer& fb = backdrop_;
        // canvas: deep navy, slightly lighter at the top
        for (int y = 0; y < kH; ++y) {
            const float t = static_cast<float>(y) / kH;
            const Color c = toSrgb8(LinearRgb{0.0045f, 0.0055f, 0.011f} * (1.15f - 0.3f * t));
            for (int x = 0; x < kW; ++x) fb.setPixel(x, y, c);
        }

        // header bar and its spectral accent line
        fillRect(fb, 0, 0, kW, Layout::kHeaderBottom, LinearRgb{0.008f, 0.010f, 0.019f});
        spectrumStrip(fb, 0, kW, Layout::kHeaderBottom - 2, 2, 0.75f);

        // the optical table: a breadboard with a 25 px hole pattern
        const int bx0 = static_cast<int>(kBench.xmin), bx1 = static_cast<int>(kBench.xmax);
        const int by0 = static_cast<int>(kBench.ymin), by1 = static_cast<int>(kBench.ymax);
        for (int y = by0; y < by1; ++y) {
            const float t = static_cast<float>(y - by0) / static_cast<float>(by1 - by0);
            const LinearRgb top{0.016f, 0.019f, 0.030f}, bottom{0.010f, 0.012f, 0.020f};
            const LinearRgb c = top * (1.0f - t) + bottom * t;
            for (int x = bx0; x < bx1; ++x) fb.setPixel(x, y, toSrgb8(c));
        }
        for (int x = bx0 + 100; x < bx1; x += 100) fillRect(fb, x, by0, x + 1, by1, LinearRgb{0.03f, 0.036f, 0.055f}, 0.5f);
        for (int y = by0 + 100; y < by1; y += 100) fillRect(fb, bx0, y, bx1, y + 1, LinearRgb{0.03f, 0.036f, 0.055f}, 0.5f);
        for (int y = by0 + 12; y < by1 - 6; y += 25)
            for (int x = bx0 + 12; x < bx1 - 6; x += 25) {
                disc(fb, {x + 0.5, y + 1.3}, 2.3, Color{38, 44, 62}, 0.55);    // lit lower lip
                disc(fb, {x + 0.5, y + 0.5}, 2.1, Color{4, 5, 9}, 0.9);        // the hole
            }

        // border and rulers (ticks every 25 px, numbered every 100 px)
        const LinearRgb edge{0.06f, 0.07f, 0.10f};
        fillRect(fb, bx0 - 1, by0 - 1, bx1 + 1, by0, edge);
        fillRect(fb, bx0 - 1, by1, bx1 + 1, by1 + 1, edge);
        fillRect(fb, bx0 - 1, by0, bx0, by1, edge);
        fillRect(fb, bx1, by0, bx1 + 1, by1, edge);
        for (int x = bx0; x <= bx1; x += 25) {
            const int len = ((x - bx0) % 100 == 0) ? 6 : 3;
            fillRect(fb, x, by0, x + 1, by0 + len, LinearRgb{0.10f, 0.12f, 0.17f});
            if ((x - bx0) % 100 == 0 && x > bx0 && x < bx1 - 20)
                render::drawText(fb, x + 3, by0 + 2, std::to_string(x - bx0), kFaint, render::Font::Small);
        }
        for (int y = by0; y <= by1; y += 25) {
            const int len = ((y - by0) % 100 == 0) ? 6 : 3;
            fillRect(fb, bx0, y, bx0 + len, y + 1, LinearRgb{0.10f, 0.12f, 0.17f});
        }

        // HUD panels
        const int top = Layout::kHudTop + 4, bottom = kH - 8;
        const auto panel = [&](int x0, int x1) {
            fillRect(fb, x0, top, x1, bottom, LinearRgb{0.012f, 0.015f, 0.026f});
            fillRect(fb, x0, top, x1, top + 1, LinearRgb{0.05f, 0.06f, 0.09f});
            fillRect(fb, x0, bottom - 1, x1, bottom, LinearRgb{0.03f, 0.035f, 0.05f});
        };
        panel(24, 214);
        panel(222, 742);
        panel(750, 1256);
    }

    // =====================================================================================
    // elements

    void Studio::drawElements(Framebuffer& fb, const Scene& scene) const {
        for (const auto& e : scene.elements()) {
            const std::vector<Vec2> body = e->outline();

            if (const auto* laser = dynamic_cast<const LaserSource*>(e.get())) {
                // a machined housing: cylinder-like shading across its width, a darker front cap
                const Transform2D& t = laser->transform;
                const double halfW = laser->bodyWidth() / 2.0, len = laser->bodyLength();
                const Color bc = laser->beamColor();
                const LinearRgb band = toLinear(bc);
                shadePolygon(fb, body, [&](Vec2 p, double depth) {
                    const Vec2 q = t.toLocal(p);
                    const double v = q.y / halfW;                       // -1 .. 1 across the body
                    float b = static_cast<float>(0.035 + 0.075 * (1.0 - v * v) + 0.05 * std::exp(-(v + 0.45) * (v + 0.45) * 30.0));
                    if (q.x > -7.0) b *= 0.55f;                                     // front cap
                    if (q.x < -len + 4.0) b *= 0.7f;                                // rear plate
                    const bool fins = q.x < -len * 0.45 && q.x > -len + 6.0;        // cooling fins on the rear half
                    if (fins && static_cast<int>(std::floor(-q.x)) % 4 == 0) b *= 0.45f;
                    const float rim = static_cast<float>(std::exp(-depth / 0.8));
                    LinearRgb c = LinearRgb{b * 0.95f, b, b * 1.12f} + LinearRgb{0.05f, 0.055f, 0.07f} * rim;
                    if (q.x > -len * 0.38 && q.x < -len * 0.38 + 3.0) c = band * (0.25f + 0.5f * static_cast<float>(1.0 - v * v));   // colour band
                    return std::pair{c, 1.0};
                });
                continue;
            }

            if (const auto* flat = dynamic_cast<const FlatElement*>(e.get())) {
                const Transform2D& t = flat->transform;
                if (flat->material.type == SurfaceType::Mirror) {
                    // brushed metal; the reflecting face (local y = 0) is a bright silver edge
                    shadePolygon(fb, body, [&](Vec2 p, double) {
                        const Vec2 q = t.toLocal(p);
                        const double brushed = 0.5 + 0.5 * std::sin(q.x * 0.9 + std::sin(q.x * 0.23) * 3.0);
                        const float base = static_cast<float>(0.16 + 0.05 * brushed);
                        const float face = static_cast<float>(std::exp(-q.y / 1.1));
                        const LinearRgb metal = LinearRgb{base, base * 1.03f, base * 1.08f} * (1.0f - face) +
                                                LinearRgb{0.85f, 0.88f, 0.92f} * face;
                        return std::pair{metal, 1.0};
                    });
                } else {
                    // matte screen: near-black with a faint amber face and ruler ticks every 10 px
                    shadePolygon(fb, body, [&](Vec2 p, double) {
                        const Vec2 q = t.toLocal(p);
                        const float face = static_cast<float>(std::exp(-q.y / 1.2));
                        return std::pair{LinearRgb{0.012f, 0.013f, 0.017f} + LinearRgb{0.10f, 0.06f, 0.012f} * face, 1.0};
                    });
                    const double half = flat->halfLength();
                    for (double s = -half + 10.0; s < half; s += 10.0) {
                        const bool major = std::fmod(std::abs(s + half), 50.0) < 1e-6;
                        drawLineWu(fb, t.toWorld({s, flat->thickness()}), t.toWorld({s, flat->thickness() - (major ? 5.0 : 2.5)}),
                                   Color{70, 62, 46});
                    }
                }
                continue;
            }

            if (const auto* dish = dynamic_cast<const CurvedMirrorElement*>(e.get())) {
                // polished metal band: bright along the parabolic face, hatched on the back
                const curves::Polyline face = dish->worldProfile().flatten(0.25);
                shadePolygon(fb, body, [&](Vec2 p, double) {
                    double dFace = 1e9;
                    for (std::size_t i = 0; i + 1 < face.size(); ++i)
                        dFace = std::min(dFace, (geom::closestPointOnSegment(p, Segment{face[i], face[i + 1]}) - p).length());
                    const float f = static_cast<float>(std::exp(-dFace / 1.2));
                    const float g = static_cast<float>(0.15 + 0.05 * std::sin(p.y * 0.35));
                    return std::pair{LinearRgb{g, g * 1.03f, g * 1.08f} * (1.0f - f) + LinearRgb{0.88f, 0.9f, 0.95f} * f, 1.0};
                });
                const Vec2 back = dish->transform.dirToWorld({1, 0}).normalized();
                const Vec2 slant = (back + back.perp()).normalized();
                for (std::size_t i = 4; i < face.size(); i += 8) {
                    const Vec2 p = face[i] + back * 6.5;
                    drawLineWu(fb, p, p + slant * 6.0, Color{90, 96, 112});
                }
                continue;
            }

            // glass (prisms and lenses): tinted body, bright rim, a soft highlight from the top
            if (e->material.type == SurfaceType::Glass) {
                const LinearRgb tint = glassTint(e->material);
                const geom::Aabb box = geom::boundsOf(std::span<const Vec2>(body));
                const double h = std::max(1.0, box.max.y - box.min.y);
                shadePolygon(fb, body, [&](Vec2 p, double depth) {
                    const double topness = 1.0 - (p.y - box.min.y) / h;
                    const float rim = static_cast<float>(std::exp(-depth / 1.3));
                    const float sheen = static_cast<float>(0.05 * topness * topness);
                    const LinearRgb inside = tint * (0.035f + sheen);
                    const LinearRgb edge = tint * 0.75f + LinearRgb{0.15f, 0.15f, 0.15f};
                    return std::pair{inside * (1.0f - rim) + edge * rim, 0.62 + 0.38 * static_cast<double>(rim)};
                });
            }
        }
    }

    // =====================================================================================
    // light

    void Studio::addLight(const Scene& scene, const std::vector<TracedRay>& rays, const FrameInfo& f) {
        light_.clear();
        // the image is cut into horizontal bands, one per thread; each band draws every ray clipped to
        // itself, so a pixel receives exactly the same additions, in the same order, as a serial loop
        render::parallelFor(kH, [&](int yBegin, int yEnd) {
        const ClipRect band{0.0, static_cast<double>(yBegin), kW - 1.0, yEnd - 1.0};
        const ClipRect beamClip{kBench.xmin, std::max(kBench.ymin, band.ymin), kBench.xmax, std::min(kBench.ymax, band.ymax)};
        // (a band outside the bench gets an empty beamClip: addBeam then draws nothing there, spots still do)
        for (const TracedRay& r : rays) {
            const bool white = r.source->kind == SourceSpec::Kind::WhiteLight;
            const bool bundle = r.source->kind == SourceSpec::Kind::Collimated;
            // white light: each ray carries its share of white; a little wider so neighbours blend
            const LinearRgb core = white ? r.color * 4.8f : r.color * (bundle ? 0.75f : 2.2f);
            const double sigma = white ? 1.15 : (bundle ? 0.75 : 0.85);

            for (const BeamSegment& seg : r.path.segments) {
                Vec2 a = seg.a, b = seg.b;
                if (!clipLineCohenSutherland(a, b, kBench)) continue;
                light_.addBeam(a, b, core, sigma, &beamClip);
                // scattered light: a soft halo, stronger inside glass (white rays get theirs from bloom)
                if (seg.inGlass) light_.addBeam(a, b, core * 0.10f, 4.5, &beamClip);
                else if (!white) light_.addBeam(a, b, core * (bundle ? 0.012f : 0.03f), 3.0, &beamClip);
            }
            // where it lands
            if (r.path.hitScreen) light_.addSpot(r.path.end, core * (white ? 0.9f : 0.8f), white ? 3.5 : 4.0, &band);
        }
        });

        // indicator LEDs on the housings, and a photon glow riding each beam
        for (const LaserSource* src : scene.sources()) {
            const Vec2 led = src->transform.toWorld({-src->bodyLength() + 9.0, -src->bodyWidth() / 2.0 + 6.0});
            const TracedRay* first = nullptr;
            for (const TracedRay& r : rays)
                if (r.source->element == src->name && isCentral(r)) first = &r;
            if (!first) continue;
            const LinearRgb c = first->source->kind == SourceSpec::Kind::WhiteLight ? LinearRgb{1, 1, 1} : first->color;
            light_.addSpot(led, c * 1.6f, 1.6);
        }
        for (const TracedRay& r : rays) {
            if (!isCentral(r)) continue;
            const double len = visibleLength(r.path);
            if (len < 1.0) continue;
            const LinearRgb c = r.source->kind == SourceSpec::Kind::WhiteLight ? LinearRgb{1, 1, 1} : r.color;
            for (double s0 = 0.0; s0 < len; s0 += 230.0) {
                const double s = std::fmod(s0 + f.time * 260.0, len);
                light_.addSpot(pointAlong(r.path, s), c * 0.9f, 3.0);
            }
        }
    }

    // =====================================================================================
    // overlays: markers, foci, labels, photons, selection

    void Studio::drawOverlays(Framebuffer& fb, const Scene& scene, const std::vector<TracedRay>& rays,
                              const FrameInfo& f, FrameReport& report) const {
        using render::Align;
        using render::Font;

        // beam events (one marker set per source: its central ray)
        for (const TracedRay& r : rays) {
            if (r.path.hitScreen) ++report.raysOnScreen;
            ++report.rays;
            for (const BeamEvent& e : r.path.events) {
                if (e.kind == BeamEvent::Kind::TotalInternalReflection) {
                    ++report.tir;
                    const OpticalElement* el = scene.find(e.element);
                    const double n = el ? el->material.indexAt(r.wavelengthNm) : 1.5;
                    const auto critical = criticalAngle(n, 1.0);
                    if (critical && e.incidenceDeg * kPi / 180.0 >= *critical - 1e-9) ++report.tirChecked;
                    if (e.element == "fold" && isCentral(r) && critical) {
                        report.tirIncidenceDeg = e.incidenceDeg;
                        report.tirCriticalDeg = deg(*critical);
                    }
                }
                if (!isCentral(r)) continue;
                switch (e.kind) {
                    case BeamEvent::Kind::Refract: ring(fb, e.point, 3.2, kCyan, 1.1, 0.85); break;
                    case BeamEvent::Kind::Reflect: ring(fb, e.point, 3.2, Color{240, 240, 250}, 1.1, 0.85); break;
                    case BeamEvent::Kind::TotalInternalReflection:
                        ring(fb, e.point, 4.5, kAmber, 1.4);
                        ring(fb, e.point, 7.5, kAmber, 1.0, 0.45);
                        break;
                    case BeamEvent::Kind::Absorb: break;
                }
            }
        }

        // the lens's motion path (dashed) with its control points, and the broad-phase boxes
        {
            const curves::Bezier& path = lensPath();
            const curves::Polyline pts = path.flatten(0.2);
            for (std::size_t i = 0; i + 1 < pts.size(); i += 2) drawLineWu(fb, pts[i], pts[i + 1], Color{70, 84, 120});
            ring(fb, pts.front(), 2.5, Color{90, 104, 140});
            ring(fb, pts.back(), 2.5, Color{90, 104, 140});
        }
        if (opt_.showBounds)
            for (const auto& e : scene.elements()) brackets(fb, e->bounds(), Color{60, 70, 98});

        // foci: the lens's lensmaker back focus, the dish's paraboloid focus
        const auto* lens = scene.findAs<ThickLensElement>("lens");
        const auto* dish = scene.findAs<CurvedMirrorElement>("dish");
        const Vec2 lensF = lens->backFocus();
        const Vec2 dishF = dish->transform.toWorld({-160.0, 0.0});
        focusMark(fb, lensF, kFocus);
        focusMark(fb, dishF, kFocus);
        render::drawText(fb, static_cast<int>(lensF.x) + 9, static_cast<int>(lensF.y) - 22, "F'", kFocus, Font::Small);
        render::drawText(fb, static_cast<int>(dishF.x) + 7, static_cast<int>(dishF.y) - 22, "F", kFocus, Font::Small);

        // the moving lens, selected with an XOR box (blitting it again would restore the pixels)
        {
            const geom::Aabb b = lens->bounds();
            const int pad = 7;
            const int x0 = static_cast<int>(std::floor(b.min.x)) - pad, y0 = static_cast<int>(std::floor(b.min.y)) - pad;
            const int w = static_cast<int>(std::ceil(b.max.x)) + pad - x0, h = static_cast<int>(std::ceil(b.max.y)) + pad - y0;
            Framebuffer frame(w, h, colors::Black);
            const Color on{150, 150, 150};
            for (int x = 0; x < w; ++x) if ((x / 3) % 2 == 0) { frame.setPixel(x, 0, on); frame.setPixel(x, h - 1, on); }
            for (int y = 0; y < h; ++y) if ((y / 3) % 2 == 0) { frame.setPixel(0, y, on); frame.setPixel(w - 1, y, on); }
            blit(frame, Rect{0, 0, w, h}, fb, x0, y0, BlitMode::Xor);
        }

        // the spectrum's wavelength scale: where 400, 450 ... 700 nm land, marked behind the screen
        {
            double yLo = 1e9, yHi = -1e9, lastLabel = -1e9;
            for (const TracedRay& r : rays) {
                if (r.source->kind != SourceSpec::Kind::WhiteLight || !r.path.hitScreen) continue;
                yLo = std::min(yLo, r.path.end.y);
                yHi = std::max(yHi, r.path.end.y);
                const double nm = r.wavelengthNm;
                if (std::fmod(nm + 0.01, 50.0) > 0.1) continue;            // every 50 nm
                const Vec2 p = r.path.end + Vec2{10, 0};                    // just behind the 8 px screen
                drawLineWu(fb, p, p + Vec2{5, 0}, kMuted);
                if (std::abs(p.y - lastLabel) < 13.0) continue;             // too close to the previous label
                lastLabel = p.y;
                render::drawText(fb, static_cast<int>(p.x) + 8, static_cast<int>(p.y) - 7,
                                 std::to_string(static_cast<int>(std::lround(nm))), kMuted, Font::Small);
            }
            if (yHi > yLo) report.spectrumSpanPx = yHi - yLo;
        }

        // labels with live readouts
        const auto label = [&](Vec2 at, const std::string& text, Align align = Align::Center, Color c = kInk) {
            render::drawLabel(fb, static_cast<int>(at.x), static_cast<int>(at.y), text, c, kChip, Font::Small, align);
        };
        for (const LaserSource* src : scene.sources()) {
            const Vec2 top = src->transform.toWorld({-src->bodyLength() / 2.0, -src->bodyWidth() / 2.0});
            std::string text = src->name;
            for (const TracedRay& r : rays)
                if (r.source->element == src->name) { text = r.source->label; break; }
            label(top - Vec2{0, 21}, text);
        }
        const double nLens = lens->material.refractiveIndex;
        label(lens->transform.position + Vec2{0, 84}, glassName(lens->material) + " lens  f " + fmt("%.1f", lens->shape().focalLength(nLens)));
        const auto* fold = scene.find("fold");
        label(fold->transform.position + Vec2{0, -64},
              glassName(fold->material) + " fold   TIR " + fmt("%.1f", report.tirIncidenceDeg) + "° ≥ θc " +
                  fmt("%.1f", report.tirCriticalDeg) + "°",
              Align::Center, kAmber);
        const auto* notched = scene.find("notched");
        label(notched->transform.position + Vec2{0, 92}, "BK7 notch (concave)");
        const auto* prism = scene.find("prism");
        label(prism->transform.position + Vec2{58, 38},
              glassName(prism->material) + " prism  n " + fmt("%.3f", prism->material.refractiveIndex) + "  V " +
                  fmt("%.1f", prism->material.abbeNumber),
              Align::Left);
        label(dish->transform.position + Vec2{-96, 96}, "parabolic mirror  f 160");
        label(scene.find("detector")->transform.position + Vec2{-6, -42}, "detector", Align::Center, kMuted);
        label(scene.find("floor")->transform.position + Vec2{-150, -22}, "screen", Align::Center, kMuted);
        const auto* screen = scene.findAs<FlatElement>("screen");
        label(screen->transform.position + Vec2{-14, -screen->halfLength() - 4}, "spectrum  λ nm", Align::Right, kMuted);
        label(scene.find("mirrorTop")->transform.position + Vec2{-30, -34}, "M2", Align::Center, kMuted);
        label(scene.find("mirrorLow")->transform.position + Vec2{40, -8}, "M1", Align::Center, kMuted);
        label(scene.find("mirrorWhite")->transform.position + Vec2{40, 0}, "M3", Align::Center, kMuted);

        // sprite photons riding the beams (sprite sheet frames, colour-keyed onto the frame)
        int sheet = 0;
        for (const TracedRay& r : rays) {
            if (!isCentral(r)) continue;
            const SpriteSheet& ph = photons_[static_cast<std::size_t>(std::min(sheet++, 2))];
            const double len = visibleLength(r.path);
            if (len < 1.0) continue;
            for (double s0 = 0.0; s0 < len; s0 += 230.0) {
                const Vec2 p = pointAlong(r.path, std::fmod(s0 + f.time * 260.0, len));
                ph.drawFrame(static_cast<std::size_t>(f.index + static_cast<int>(s0)) % ph.frameCount(), fb,
                             static_cast<int>(p.x) - ph.frameWidth() / 2, static_cast<int>(p.y) - ph.frameHeight() / 2,
                             BlitMode::KeepDestination);
            }
        }
    }

    // =====================================================================================
    // header + HUD

    void Studio::drawHud(Framebuffer& fb, const Scene& scene, const std::vector<SourceSpec>& sources,
                         const std::vector<TracedRay>& rays, const FrameInfo& f, const FrameReport& report) const {
        using render::Align;
        using render::Font;

        // header
        render::drawText(fb, 24, 15, "OPTIBENCH", kInk, Font::Title);
        render::drawText(fb, 24 + render::textWidth("OPTIBENCH", Font::Title) + 14, 21,
                         "spectral ray tracing on a 2D optical bench", kMuted, Font::Regular);
        const int total = static_cast<int>(std::lround(opt_.duration * opt_.fps));
        char right[96];
        std::snprintf(right, sizeof right, "t %5.2f s   frame %03d / %03d   %g fps", f.time, f.index, total, opt_.fps);
        render::drawText(fb, kW - 24, 21, right, kMuted, Font::Regular, Align::Right);

        const int top = Layout::kHudTop + 4, bottom = kH - 8;

        // --- minimap: the bench drawn through one Mat3 (T * S * T), blitted into the panel
        {
            constexpr int mw = 174, mh = 84;
            Framebuffer mini(mw, mh, Color{9, 11, 19});
            const double s = std::min((mw - 6) / (kBench.xmax - kBench.xmin), (mh - 6) / (kBench.ymax - kBench.ymin));
            const Mat3 toMini = Mat3::translation(3, 3) * Mat3::scaling(s, s) * Mat3::translation(-kBench.xmin, -kBench.ymin);
            for (const auto& e : scene.elements()) {
                const std::vector<Vec2> o = toMini.transformPoints(e->outline());
                const Color c = e->material.type == SurfaceType::Glass ? Color{60, 120, 200}
                              : e->material.type == SurfaceType::Mirror ? Color{170, 175, 190}
                              : e->material.type == SurfaceType::Screen ? Color{120, 96, 50} : Color{80, 86, 100};
                if (o.size() >= 3) fillPolygon(mini, o, c);
            }
            for (const TracedRay& r : rays) {
                if (r.source->kind == SourceSpec::Kind::WhiteLight && r.index % 6 != 0) continue;
                const Color c = toSrgb8(r.source->kind == SourceSpec::Kind::WhiteLight ? render::laserColor(r.wavelengthNm) : r.color);
                for (const BeamSegment& seg : r.path.segments) {
                    Vec2 a = seg.a, b = seg.b;
                    if (!clipLineCohenSutherland(a, b, kBench)) continue;
                    drawLine(mini, toMini.transformPoint(a), toMini.transformPoint(b), c);
                }
            }
            blit(mini, Rect{0, 0, mw, mh}, fb, 32, top + 12, BlitMode::Copy);
            render::drawText(fb, 32, top - 1, "BENCH", kFaint, Font::Small);
        }

        // --- beam readouts
        {
            int y = top + 4;
            render::drawText(fb, 236, y, "BEAMS", kFaint, Font::Small);
            render::drawText(fb, 236 + 150, y, "PATH", kFaint, Font::Small);
            render::drawText(fb, 728, y, "ON SCREEN", kFaint, Font::Small, Align::Right);
            y += 19;
            for (const SourceSpec& spec : sources) {
                int onScreen = 0, count = 0;
                const TracedRay* central = nullptr;
                for (const TracedRay& r : rays) {
                    if (r.source != &spec) continue;
                    ++count;
                    if (r.path.hitScreen) ++onScreen;
                    if (isCentral(r)) central = &r;
                }
                disc(fb, {241.5, y + 7.5}, 3.5, sourceColor(spec));
                render::drawText(fb, 252, y, spec.label, kInk, Font::Small);
                std::string path;
                if (central) {
                    std::string last;
                    for (const BeamEvent& e : central->path.events) {
                        const std::string name = displayName(e.element);
                        if (name == last) {                                 // the same element again (entry / exit)
                            if (e.kind == BeamEvent::Kind::TotalInternalReflection && path.find("TIR") == std::string::npos) path += " (TIR)";
                            continue;
                        }
                        if (!path.empty()) path += " → ";
                        path += name;
                        last = name;
                    }
                }
                if (count > 1) path = std::to_string(count) + (spec.kind == SourceSpec::Kind::WhiteLight ? " λ   " : " rays   ") + path;
                render::drawText(fb, 236 + 150, y, path, kMuted, Font::Small);
                render::drawText(fb, 728, y, std::to_string(onScreen) + "/" + std::to_string(count) + (onScreen == count ? " ✓" : " ✗"),
                                 onScreen == count ? kGood : kBad, Font::Small, Align::Right);
                y += 18;
            }
            render::drawText(fb, 236, y + 2,
                             "spectrum 400–700 nm across " + fmt("%.0f", report.spectrumSpanPx) + " px  ·  lens back focal distance " +
                                 fmt("%.1f", lensBfd(scene)),
                             kFaint, Font::Small);
        }

        // --- checks and the keyframe timeline
        {
            const int x0 = 764;
            int y = top + 4;
            const bool overlapOk = report.overlaps == 0, tirOk = report.tir == report.tirChecked;
            render::drawText(fb, x0, y, "CHECKS", kFaint, Font::Small);
            int x = x0 + 64;
            x = render::drawText(fb, x, y, overlapOk ? "✓ no overlaps" : "✗ overlapping elements", overlapOk ? kGood : kBad, Font::Small);
            x = render::drawText(fb, x + 18, y, tirOk ? "✓ TIR ≥ θc" : "✗ TIR below θc", tirOk ? kGood : kBad, Font::Small);
            render::drawText(fb, x + 18, y, "✓ " + std::to_string(report.raysOnScreen) + "/" + std::to_string(report.rays) + " rays traced",
                             kGood, Font::Small);

            // timeline: one row per animated track, diamonds at its keyframes, a playhead at "now"
            const int tx0 = x0 + 76, tx1 = 1240;
            y += 22;
            const auto timeX = [&](double t) { return tx0 + (tx1 - tx0) * std::clamp(t / kDuration, 0.0, 1.0); };
            for (const KeyframeTrack& track : tracks_) {
                render::drawText(fb, x0, y - 7, track.label, kMuted, Font::Small);
                fillRect(fb, tx0, y, tx1, y + 1, LinearRgb{0.035f, 0.042f, 0.065f});
                for (const double t : track.times) {
                    const double cx = timeX(t);
                    for (int k = -3; k <= 3; ++k) {
                        const int span = 3 - std::abs(k);
                        fillRect(fb, static_cast<int>(cx) - span, y + k, static_cast<int>(cx) + span + 1, y + k + 1,
                                 toLinear(Color{140, 160, 205}));
                    }
                }
                y += 13;
            }
            const double px = timeX(std::fmod(f.time, kDuration + 1e-9));
            fillRect(fb, static_cast<int>(px), top + 24, static_cast<int>(px) + 1, bottom - 4, toLinear(kFocus), 0.9f);
            disc(fb, {px + 0.5, top + 24.0}, 3.0, kFocus);
        }
    }

    // =====================================================================================

    FrameReport Studio::render(Framebuffer& fb, const Scene& scene, const std::vector<SourceSpec>& sources,
                               const FrameInfo& frame) {
        FrameReport report;
        report.overlaps = static_cast<int>(scene.overlappingPairs().size());

        fb = backdrop_;
        drawElements(fb, scene);
        const std::vector<TracedRay> rays = traceSources(scene, sources);
        addLight(scene, rays, frame);
        render::composite(fb, light_, render::Look{});
        drawOverlays(fb, scene, rays, frame, report);
        drawHud(fb, scene, sources, rays, frame, report);
        return report;
    }
}
