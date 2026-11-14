#pragma once
#include <optional>
#include "Color.h"
#include "Framebuffer.h"

namespace ob {
    /**
     * Raster compositing operations. Framebuffer pixels are RGB; transparency is
     * supplied as a color key or a separate mask rather than per-pixel alpha.
     */

    // a rectangular block of pixels: top-left corner plus size
    struct Rect {
        int x = 0, y = 0, w = 0, h = 0;
    };

    // how a source pixel combines with the destination pixel
    enum class BlitMode {
        Copy,       // overwrite: the source replaces the destination
        KeepDestination, // source pixels matching the color key leave destination unchanged
        And,        // bitwise AND of the colour channels
        Or,         // bitwise OR
        Xor,        // bitwise XOR: applying it twice restores the destination
        Mask        // copy source only where the separate mask matches maskColor
    };

    struct BlitOptions {
        BlitMode mode = BlitMode::Copy;
        // Used by KeepDestination. Defaults to black, a convenient transparent key.
        Color colorKey = colors::Black;
        // Used by Mask; coordinates correspond to the source framebuffer.
        const Framebuffer* mask = nullptr;
        Color maskColor = colors::White;
    };

    // copies the block `srcRect` of `src` to (dstX, dstY) in `dst`; parts outside either image are clipped
    void blit(const Framebuffer& src, const Rect& srcRect, Framebuffer& dst, int dstX, int dstY,
              BlitMode mode = BlitMode::Copy);
    void blit(const Framebuffer& src, const Rect& srcRect, Framebuffer& dst, int dstX, int dstY,
              const BlitOptions& options);

    // result = src * alpha + dst * (1 - alpha),   alpha in [0, 1]
    Color blendOver(Color src, Color dst, double alpha);

    // blends `c` into one pixel of the framebuffer with the given alpha
    void blendPixel(Framebuffer& fb, int x, int y, Color c, double alpha);
}
