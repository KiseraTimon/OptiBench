#pragma once
#include "Color.h"
#include "Framebuffer.h"

namespace ob {
    /**
     * CONTRACT: raster operations.                                OWNER: Cyprian
     * (declarations only: implement in src/raster_ops.cpp, signatures can be adjusted via PR)
     */

    // a rectangular block of pixels: top-left corner plus size
    struct Rect {
        int x = 0, y = 0, w = 0, h = 0;
    };

    // how a source pixel combines with the destination pixel
    enum class BlitMode {
        Copy,       // overwrite: the source replaces the destination
        And,        // bitwise AND of the colour channels
        Or,         // bitwise OR
        Xor         // bitwise XOR: applying it twice restores the destination
    };

    // copies the block `srcRect` of `src` to (dstX, dstY) in `dst`; parts outside either image are clipped
    void blit(const Framebuffer& src, const Rect& srcRect, Framebuffer& dst, int dstX, int dstY,
              BlitMode mode = BlitMode::Copy);

    // result = src * alpha + dst * (1 - alpha),   alpha in [0, 1]
    Color blendOver(Color src, Color dst, double alpha);

    // blends `c` into one pixel of the framebuffer with the given alpha
    void blendPixel(Framebuffer& fb, int x, int y, Color c, double alpha);
}
