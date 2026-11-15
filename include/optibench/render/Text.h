#pragma once
#include <string_view>

#include "optibench/Color.h"
#include "optibench/Framebuffer.h"

/**
 * Anti-aliased text for labels and the HUD.
 *
 * Fixed-width bitmap fonts (DejaVu Sans Mono, rasterised by tools/generate_font.py) with 4-bit
 * coverage; each pixel is alpha-blended over the framebuffer in linear light. Strings are UTF-8:
 * printable ASCII plus the symbols ° θ λ · → × µ ≥ ≤ Δ ✓ ✗ ± – ′. Unknown characters draw as '?'.
 *
 *      drawText(fb, 40, 30, "f' = 162.4 px", colors::White);
 *      drawText(fb, 600, 30, "OptiBench", colors::White, Font::Title, Align::Center);
 */
namespace ob::render {
    enum class Font { Small, Regular, Title };
    enum class Align { Left, Center, Right };

    struct FontMetrics {
        int advance;    // width of every character cell
        int height;     // line height
        int ascent;     // baseline, from the top of the cell
    };
    FontMetrics metrics(Font font);

    // number of characters (code points) in a UTF-8 string, and its width in pixels
    int glyphCount(std::string_view utf8);
    int textWidth(std::string_view utf8, Font font = Font::Small);

    /**
     * draws a line of text. (x, y) is the TOP of the text cell; `align` positions x at the left edge,
     * the middle or the right edge. `opacity` scales the coverage. Returns the x just past the text.
     */
    int drawText(Framebuffer& fb, int x, int y, std::string_view utf8, Color color, Font font = Font::Small,
                 Align align = Align::Left, double opacity = 1.0);

    // the same, over a translucent rounded "chip" so it stays readable over beams and grid
    void drawLabel(Framebuffer& fb, int x, int y, std::string_view utf8, Color color, Color background,
                   Font font = Font::Small, Align align = Align::Left, double backgroundOpacity = 0.72);
}
