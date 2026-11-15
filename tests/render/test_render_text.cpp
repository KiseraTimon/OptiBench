// Text: UTF-8 decoding, metrics, alignment, and drawing inside the expected box.
#include <algorithm>

#include "curves/TestHarness.h"
#include "optibench/render/Text.h"

using namespace ob;
using namespace ob::render;

namespace {
    bool sameImage(const Framebuffer& a, const Framebuffer& b) {
        for (int y = 0; y < a.height(); ++y)
            for (int x = 0; x < a.width(); ++x) {
                const Color p = a.getPixel(x, y), q = b.getPixel(x, y);
                if (p.r != q.r || p.g != q.g || p.b != q.b) return false;
            }
        return true;
    }

    // bounding box of the pixels that differ from the background
    struct Box { int x0 = 1 << 20, y0 = 1 << 20, x1 = -1, y1 = -1; };
    Box inkBox(const Framebuffer& fb, Color bg) {
        Box b;
        for (int y = 0; y < fb.height(); ++y)
            for (int x = 0; x < fb.width(); ++x) {
                const Color c = fb.getPixel(x, y);
                if (c.r == bg.r && c.g == bg.g && c.b == bg.b) continue;
                b.x0 = std::min(b.x0, x); b.y0 = std::min(b.y0, y);
                b.x1 = std::max(b.x1, x); b.y1 = std::max(b.y1, y);
            }
        return b;
    }

    void testMetricsAndUtf8(test::Context& ctx) {
        for (const Font f : {Font::Small, Font::Regular, Font::Title}) {
            const FontMetrics m = metrics(f);
            CHECK(m.advance > 0 && m.height > m.ascent && m.ascent > 0);
            CHECK(textWidth("abc", f) == 3 * m.advance);
        }
        CHECK(metrics(Font::Title).height > metrics(Font::Small).height);
        CHECK(glyphCount("") == 0);
        CHECK(glyphCount("θc ≥ 34.1°") == 10);                 // multi-byte characters count once
        CHECK(glyphCount("\xff\xfe") == 2);                     // invalid bytes: one '?' each
        CHECK(glyphCount("ab\xe2\x86") == 3);                   // a truncated sequence at the end
    }

    void testDrawingStaysInItsBox(test::Context& ctx) {
        const Color bg{0, 0, 0};
        Framebuffer fb(200, 40, bg);
        const int end = drawText(fb, 10, 5, "Hello, bench", Color{255, 255, 255});
        const FontMetrics m = metrics(Font::Small);
        CHECK(end == 10 + 12 * m.advance);
        const Box b = inkBox(fb, bg);
        CHECK(b.x1 >= 0);                                        // something was drawn
        CHECK(b.x0 >= 10 && b.x1 < end && b.y0 >= 5 && b.y1 < 5 + m.height);

        // right alignment ends at x; centring straddles it
        Framebuffer r(200, 40, bg);
        drawText(r, 150, 5, "MMMM", Color{255, 255, 255}, Font::Small, Align::Right);
        const Box rb = inkBox(r, bg);
        CHECK(rb.x1 < 150 && rb.x0 >= 150 - 4 * m.advance);
        Framebuffer c(200, 40, bg);
        drawText(c, 100, 5, "MMMM", Color{255, 255, 255}, Font::Small, Align::Center);
        const Box cb = inkBox(c, bg);
        CHECK(cb.x0 < 100 && cb.x1 >= 100);

        // text at the edges is clipped, not written out of bounds
        Framebuffer edge(20, 10, bg);
        drawText(edge, -5, -5, "WWWW", Color{255, 255, 255}, Font::Title);
        drawText(edge, 15, 5, "WWWW", Color{255, 255, 255}, Font::Title);
        CHECK(edge.width() == 20);
    }

    void testUnknownCharactersAndOpacity(test::Context& ctx) {
        const Color bg{10, 20, 30};
        Framebuffer a(60, 30, bg), b(60, 30, bg);
        drawText(a, 5, 5, "中", Color{255, 255, 255});            // not in the font
        drawText(b, 5, 5, "?", Color{255, 255, 255});
        CHECK(sameImage(a, b));

        Framebuffer none(60, 30, bg), blank(60, 30, bg);
        drawText(none, 5, 5, "abc", Color{255, 255, 255}, Font::Small, Align::Left, 0.0);
        CHECK(sameImage(none, blank));                          // opacity 0 draws nothing

        Framebuffer chip(80, 30, bg);
        drawLabel(chip, 40, 8, "label", Color{255, 255, 255}, Color{200, 0, 0}, Font::Small, Align::Center);
        CHECK(chip.getPixel(40 - textWidth("label") / 2 - 3, 10).r > bg.r);      // the chip is painted behind
    }
}

int main() {
    return test::runTests("render: text", {
        {"metrics and UTF-8", testMetricsAndUtf8},
        {"drawing stays in its box", testDrawingStaysInItsBox},
        {"unknown characters, opacity and labels", testUnknownCharactersAndOpacity},
    });
}
