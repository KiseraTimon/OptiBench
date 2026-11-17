#include "optibench/render/LightBuffer.h"

#include <algorithm>
#include <cmath>
#include <stdexcept>

#include "optibench/render/Parallel.h"

namespace ob::render {
    LightBuffer::LightBuffer(int width, int height) : w_(width), h_(height) {
        if (width <= 0 || height <= 0) throw std::invalid_argument("LightBuffer: size must be positive");
        px_.assign(static_cast<std::size_t>(w_) * static_cast<std::size_t>(h_), LinearRgb{});
    }

    void LightBuffer::clear() { std::fill(px_.begin(), px_.end(), LinearRgb{}); }

    LinearRgb LightBuffer::at(int x, int y) const {
        if (x < 0 || y < 0 || x >= w_ || y >= h_) return {};
        return px_[static_cast<std::size_t>(y) * static_cast<std::size_t>(w_) + static_cast<std::size_t>(x)];
    }

    void LightBuffer::add(int x, int y, const LinearRgb& c) {
        if (x < 0 || y < 0 || x >= w_ || y >= h_) return;
        px_[static_cast<std::size_t>(y) * static_cast<std::size_t>(w_) + static_cast<std::size_t>(x)] += c;
    }

    void LightBuffer::addBeam(Vec2 a, Vec2 b, const LinearRgb& peak, double sigma, const ClipRect* clip) {
        if (sigma <= 0.0) return;
        const Vec2 d = b - a;
        const double len = d.length();
        if (len < 1e-9) return;
        const Vec2 u = d / len;                     // along the beam
        const double reach = 3.5 * sigma + 1.0;     // beyond this the profile is negligible
        const double inv2s2 = 1.0 / (2.0 * sigma * sigma);

        // pixel range we may touch
        double xmin = std::min(a.x, b.x) - reach, xmax = std::max(a.x, b.x) + reach;
        double ymin = std::min(a.y, b.y) - reach, ymax = std::max(a.y, b.y) + reach;
        xmin = std::max(xmin, 0.0); ymin = std::max(ymin, 0.0);
        xmax = std::min(xmax, w_ - 1.0); ymax = std::min(ymax, h_ - 1.0);
        if (clip) {
            xmin = std::max(xmin, clip->xmin); ymin = std::max(ymin, clip->ymin);
            xmax = std::min(xmax, clip->xmax); ymax = std::min(ymax, clip->ymax);
        }
        if (xmin > xmax || ymin > ymax) return;

        // walk the major axis; on each line only the band around the beam is visited
        const bool steep = std::abs(d.y) > std::abs(d.x);
        const int lo = static_cast<int>(std::floor(steep ? ymin : xmin));
        const int hi = static_cast<int>(std::ceil(steep ? ymax : xmax));
        const double minorLo = steep ? xmin : ymin, minorHi = steep ? xmax : ymax;
        const double major0 = steep ? a.y : a.x, majorD = steep ? d.y : d.x;
        const double minor0 = steep ? a.x : a.y, minorD = steep ? d.x : d.y;
        // the band half-width on the minor axis: reach / cos(angle to the major axis)
        const double halfBand = reach * len / std::abs(majorD);

        for (int m = lo; m <= hi; ++m) {
            // where the centre line crosses this major coordinate (clamped to the segment)
            const double t = std::clamp((m - major0) / majorD, 0.0, 1.0);
            const double c = minor0 + minorD * t;
            const int n0 = static_cast<int>(std::floor(std::max(c - halfBand, minorLo)));
            const int n1 = static_cast<int>(std::ceil(std::min(c + halfBand, minorHi)));
            for (int n = n0; n <= n1; ++n) {
                const int x = steep ? n : m, y = steep ? m : n;
                const Vec2 p{x + 0.5, y + 0.5};
                const Vec2 ap = p - a;
                const double along = ap.dot(u);
                const double across = ap.cross(u);
                // ends: a one-pixel linear ramp centred on each end point. Two segments that share an
                // end point have complementary ramps there, so a chained beam stays uniform at the join.
                const double ends = std::clamp(along + 0.5, 0.0, 1.0) * std::clamp(len - along + 0.5, 0.0, 1.0);
                const double w = ends * std::exp(-across * across * inv2s2);
                if (w < 1e-4) continue;
                add(x, y, peak * static_cast<float>(w));
            }
        }
    }

    void LightBuffer::addSpot(Vec2 centre, const LinearRgb& peak, double sigma, const ClipRect* clip) {
        if (sigma <= 0.0) return;
        const double reach = 3.5 * sigma + 1.0, inv2s2 = 1.0 / (2.0 * sigma * sigma);
        int x0 = std::max(0, static_cast<int>(std::floor(centre.x - reach)));
        int x1 = std::min(w_ - 1, static_cast<int>(std::ceil(centre.x + reach)));
        int y0 = std::max(0, static_cast<int>(std::floor(centre.y - reach)));
        int y1 = std::min(h_ - 1, static_cast<int>(std::ceil(centre.y + reach)));
        if (clip) {
            x0 = std::max(x0, static_cast<int>(std::ceil(clip->xmin))); x1 = std::min(x1, static_cast<int>(std::floor(clip->xmax)));
            y0 = std::max(y0, static_cast<int>(std::ceil(clip->ymin))); y1 = std::min(y1, static_cast<int>(std::floor(clip->ymax)));
        }
        for (int y = y0; y <= y1; ++y)
            for (int x = x0; x <= x1; ++x) {
                const double dx = x + 0.5 - centre.x, dy = y + 0.5 - centre.y;
                const double w = std::exp(-(dx * dx + dy * dy) * inv2s2);
                if (w >= 1e-4) add(x, y, peak * static_cast<float>(w));
            }
    }

    LinearRgb LightBuffer::total() const {
        double r = 0, g = 0, b = 0;
        for (const LinearRgb& c : px_) { r += static_cast<double>(c.r); g += static_cast<double>(c.g); b += static_cast<double>(c.b); }
        return {static_cast<float>(r), static_cast<float>(g), static_cast<float>(b)};
    }

    // one sliding-window box pass per row: O(1) per pixel whatever the radius; edges are clamped
    void LightBuffer::boxBlurRows(int r) {
        const float inv = 1.0f / static_cast<float>(2 * r + 1);
        parallelFor(h_, [&](int yBegin, int yEnd) {
        std::vector<LinearRgb> line(static_cast<std::size_t>(w_));
        for (int y = yBegin; y < yEnd; ++y) {
            LinearRgb* row = &px_[static_cast<std::size_t>(y) * static_cast<std::size_t>(w_)];
            auto px = [&](int x) { return row[std::clamp(x, 0, w_ - 1)]; };
            LinearRgb acc;
            for (int x = -r; x <= r; ++x) acc += px(x);
            for (int x = 0; x < w_; ++x) {
                line[static_cast<std::size_t>(x)] = acc * inv;
                const LinearRgb in = px(x + r + 1), out = px(x - r);
                acc += LinearRgb{in.r - out.r, in.g - out.g, in.b - out.b};
            }
            std::copy(line.begin(), line.end(), row);
        }
        });
    }

    void LightBuffer::boxBlurColumns(int r) {
        const float inv = 1.0f / static_cast<float>(2 * r + 1);
        parallelFor(w_, [&](int xBegin, int xEnd) {
        std::vector<LinearRgb> line(static_cast<std::size_t>(h_));
        for (int x = xBegin; x < xEnd; ++x) {
            auto px = [&](int y) { return px_[static_cast<std::size_t>(std::clamp(y, 0, h_ - 1)) * static_cast<std::size_t>(w_) + static_cast<std::size_t>(x)]; };
            LinearRgb acc;
            for (int y = -r; y <= r; ++y) acc += px(y);
            for (int y = 0; y < h_; ++y) {
                line[static_cast<std::size_t>(y)] = acc * inv;
                const LinearRgb in = px(y + r + 1), out = px(y - r);
                acc += LinearRgb{in.r - out.r, in.g - out.g, in.b - out.b};
            }
            for (int y = 0; y < h_; ++y)
                px_[static_cast<std::size_t>(y) * static_cast<std::size_t>(w_) + static_cast<std::size_t>(x)] = line[static_cast<std::size_t>(y)];
        }
        });
    }

    void LightBuffer::blur(int radius) {
        if (radius <= 0) return;
        // three box passes of radius r ~ a Gaussian with sigma = r (central limit theorem)
        const int r = std::max(1, static_cast<int>(std::lround(radius / 1.73)));
        for (int pass = 0; pass < 3; ++pass) {
            boxBlurRows(r);
            boxBlurColumns(r);
        }
    }

    LightBuffer LightBuffer::downsampled() const {
        LightBuffer half(std::max(1, w_ / 2), std::max(1, h_ / 2));
        for (int y = 0; y < half.h_; ++y)
            for (int x = 0; x < half.w_; ++x) {
                const LinearRgb s = at(2 * x, 2 * y) + at(2 * x + 1, 2 * y) + at(2 * x, 2 * y + 1) + at(2 * x + 1, 2 * y + 1);
                half.px_[static_cast<std::size_t>(y) * static_cast<std::size_t>(half.w_) + static_cast<std::size_t>(x)] = s * 0.25f;
            }
        return half;
    }

    LinearRgb LightBuffer::sampleBilinear(double x, double y) const {
        x = std::clamp(x, 0.0, w_ - 1.0);
        y = std::clamp(y, 0.0, h_ - 1.0);
        const int x0 = static_cast<int>(x), y0 = static_cast<int>(y);
        const int x1 = std::min(x0 + 1, w_ - 1), y1 = std::min(y0 + 1, h_ - 1);
        const float fx = static_cast<float>(x - x0), fy = static_cast<float>(y - y0);
        const LinearRgb top = at(x0, y0) * (1.0f - fx) + at(x1, y0) * fx;
        const LinearRgb bottom = at(x0, y1) * (1.0f - fx) + at(x1, y1) * fx;
        return top * (1.0f - fy) + bottom * fy;
    }

    LinearRgb toneMap(const LinearRgb& c, float whiteCore) {
        // soft shoulder per channel: never clips, and keeps dim light linear (1 - e^-x ~ x)
        auto shoulder = [](float v) { return 1.0f - std::exp(-std::max(v, 0.0f)); };
        LinearRgb t{shoulder(c.r), shoulder(c.g), shoulder(c.b)};
        // light brighter than 1 bleeds into the other channels: saturated laser cores turn white
        const float excess = std::max(0.0f, c.maxComponent() - 1.0f);
        const float bleed = whiteCore * (1.0f - std::exp(-excess));
        t = t * (1.0f - bleed) + LinearRgb{bleed, bleed, bleed};
        return t;
    }

    void composite(Framebuffer& fb, const LightBuffer& light, const Look& look) {
        if (fb.width() != light.width() || fb.height() != light.height())
            throw std::invalid_argument("composite: framebuffer and light buffer differ in size");

        // two-scale bloom at half resolution: a tight glow and a wide halo, merged before upsampling
        LightBuffer glow = light.downsampled();
        LightBuffer far = glow;
        glow.blur(look.bloomNear);
        far.blur(look.bloomFar);
        for (std::size_t i = 0; i < glow.pixels().size(); ++i)
            glow.pixels()[i] = glow.pixels()[i] * 0.6f + far.pixels()[i] * 0.4f;

        const int w = fb.width(), h = fb.height();
        const double cx = w / 2.0, cy = h / 2.0, rmax = std::sqrt(cx * cx + cy * cy);
        // rows are independent, so they are shared out between threads (same pixels as a serial loop)
        parallelFor(h, [&](int yBegin, int yEnd) {
        for (int y = yBegin; y < yEnd; ++y)
            for (int x = 0; x < w; ++x) {
                const double hx = (x + 0.5) / 2.0 - 0.5, hy = (y + 0.5) / 2.0 - 0.5;
                const LinearRgb hdr = (light.at(x, y) + glow.sampleBilinear(hx, hy) * look.bloomStrength) * look.exposure;

                LinearRgb out = toLinear(fb.getPixel(x, y)) + toneMap(hdr, look.whiteCore);
                if (look.vignette > 0.0f) {
                    const double r2 = ((x - cx) * (x - cx) + (y - cy) * (y - cy)) / (rmax * rmax);
                    out = out * static_cast<float>(1.0 - static_cast<double>(look.vignette) * r2 * std::sqrt(std::sqrt(r2)));    // r^2.5
                }
                fb.setPixel(x, y, toSrgb8(out));
            }
        });
    }
}
