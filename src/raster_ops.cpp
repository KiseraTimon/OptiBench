#include "optibench/raster_ops.h"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <vector>

namespace ob {
    namespace {
        bool same(Color a, Color b) {
            return a.r == b.r && a.g == b.g && a.b == b.b;
        }

        std::uint8_t channel(double value) {
            return static_cast<std::uint8_t>(std::lround(std::clamp(value, 0.0, 255.0)));
        }

        Color combine(Color src, Color dst, BlitMode mode) {
            switch (mode) {
                case BlitMode::And: return {static_cast<std::uint8_t>(src.r & dst.r),
                                             static_cast<std::uint8_t>(src.g & dst.g),
                                             static_cast<std::uint8_t>(src.b & dst.b)};
                case BlitMode::Or: return {static_cast<std::uint8_t>(src.r | dst.r),
                                            static_cast<std::uint8_t>(src.g | dst.g),
                                            static_cast<std::uint8_t>(src.b | dst.b)};
                case BlitMode::Xor: return {static_cast<std::uint8_t>(src.r ^ dst.r),
                                             static_cast<std::uint8_t>(src.g ^ dst.g),
                                             static_cast<std::uint8_t>(src.b ^ dst.b)};
                default: return src;
            }
        }
    }

    void blit(const Framebuffer& src, const Rect& srcRect, Framebuffer& dst, int dstX, int dstY,
              BlitMode mode) {
        BlitOptions options;
        options.mode = mode;
        blit(src, srcRect, dst, dstX, dstY, options);
    }

    void blit(const Framebuffer& src, const Rect& srcRect, Framebuffer& dst, int dstX, int dstY,
              const BlitOptions& options) {
        if (srcRect.w <= 0 || srcRect.h <= 0) return;

        // Calculate the corresponding source and destination bounds in 64-bit space
        // so extreme caller coordinates cannot overflow during clipping.
        const std::int64_t left = std::max<std::int64_t>({0, -static_cast<std::int64_t>(srcRect.x),
                                                          -static_cast<std::int64_t>(dstX)});
        const std::int64_t top = std::max<std::int64_t>({0, -static_cast<std::int64_t>(srcRect.y),
                                                         -static_cast<std::int64_t>(dstY)});
        const std::int64_t right = std::min<std::int64_t>({srcRect.w,
            static_cast<std::int64_t>(src.width()) - srcRect.x,
            static_cast<std::int64_t>(dst.width()) - dstX});
        const std::int64_t bottom = std::min<std::int64_t>({srcRect.h,
            static_cast<std::int64_t>(src.height()) - srcRect.y,
            static_cast<std::int64_t>(dst.height()) - dstY});
        if (right <= left || bottom <= top) return;

        if (options.mode == BlitMode::Mask && !options.mask) return;

        const int width = static_cast<int>(right - left);
        const int height = static_cast<int>(bottom - top);
        const int sx0 = static_cast<int>(static_cast<std::int64_t>(srcRect.x) + left);
        const int sy0 = static_cast<int>(static_cast<std::int64_t>(srcRect.y) + top);
        const int dx0 = static_cast<int>(static_cast<std::int64_t>(dstX) + left);
        const int dy0 = static_cast<int>(static_cast<std::int64_t>(dstY) + top);

        // Snapshot the source region for self-blits so overlapping copies behave
        // like memmove and Boolean operations read the original source pixels.
        std::vector<Color> snapshot;
        const bool selfBlit = &src == &dst;
        if (selfBlit) {
            snapshot.reserve(static_cast<std::size_t>(width) * height);
            for (int y = 0; y < height; ++y)
                for (int x = 0; x < width; ++x)
                    snapshot.push_back(src.getPixel(sx0 + x, sy0 + y));
        }

        for (int y = 0; y < height; ++y) {
            for (int x = 0; x < width; ++x) {
                const int sx = sx0 + x;
                const int sy = sy0 + y;
                const int dx = dx0 + x;
                const int dy = dy0 + y;
                const Color source = selfBlit
                    ? snapshot[static_cast<std::size_t>(y) * width + x]
                    : src.getPixel(sx, sy);

                if (options.mode == BlitMode::KeepDestination && same(source, options.colorKey)) continue;
                if (options.mode == BlitMode::Mask &&
                    (sx >= options.mask->width() || sy >= options.mask->height() ||
                     !same(options.mask->getPixel(sx, sy), options.maskColor))) continue;

                const Color destination = dst.getPixel(dx, dy);
                dst.setPixel(dx, dy, combine(source, destination, options.mode));
            }
        }
    }

    Color blendOver(Color src, Color dst, double alpha) {
        if (!std::isfinite(alpha)) alpha = 0.0;
        alpha = std::clamp(alpha, 0.0, 1.0);
        const double inverse = 1.0 - alpha;
        return {channel(src.r * alpha + dst.r * inverse),
                channel(src.g * alpha + dst.g * inverse),
                channel(src.b * alpha + dst.b * inverse)};
    }

    void blendPixel(Framebuffer& fb, int x, int y, Color c, double alpha) {
        if (x < 0 || y < 0 || x >= fb.width() || y >= fb.height()) return;
        fb.setPixel(x, y, blendOver(c, fb.getPixel(x, y), alpha));
    }
}
