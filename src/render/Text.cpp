#include "optibench/render/Text.h"

#include <algorithm>
#include <cstdint>

#include "optibench/render/Spectrum.h"

#include "FontData.inc"

namespace ob::render {
    namespace {
        struct FontTable {
            FontMetrics m;
            int bytesPerGlyph;
            const unsigned char* data;
        };

        const FontTable& table(Font font) {
            using namespace fontdata;
            static const FontTable small{{kSmallAdvance, kSmallHeight, kSmallAscent}, kSmallBytesPerGlyph, kSmallData};
            static const FontTable regular{{kRegularAdvance, kRegularHeight, kRegularAscent}, kRegularBytesPerGlyph, kRegularData};
            static const FontTable title{{kTitleAdvance, kTitleHeight, kTitleAscent}, kTitleBytesPerGlyph, kTitleData};
            switch (font) {
                case Font::Regular: return regular;
                case Font::Title: return title;
                case Font::Small: break;
            }
            return small;
        }

        // decodes one UTF-8 code point at s[i] and advances i; malformed bytes decode as '?'
        char32_t nextCodepoint(std::string_view s, std::size_t& i) {
            const auto byte = [&](std::size_t k) { return static_cast<unsigned char>(s[k]); };
            const unsigned char b0 = byte(i);
            int extra = 0;
            char32_t cp = 0;
            if (b0 < 0x80) { ++i; return b0; }
            if ((b0 & 0xE0) == 0xC0) { extra = 1; cp = b0 & 0x1Fu; }
            else if ((b0 & 0xF0) == 0xE0) { extra = 2; cp = b0 & 0x0Fu; }
            else if ((b0 & 0xF8) == 0xF0) { extra = 3; cp = b0 & 0x07u; }
            else { ++i; return U'?'; }
            if (i + static_cast<std::size_t>(extra) >= s.size()) {     // truncated sequence at the end
                i = s.size();
                return U'?';
            }
            for (int k = 1; k <= extra; ++k) {
                const unsigned char b = byte(i + static_cast<std::size_t>(k));
                if ((b & 0xC0) != 0x80) { ++i; return U'?'; }
                cp = (cp << 6) | (b & 0x3Fu);
            }
            i += static_cast<std::size_t>(extra) + 1;
            return cp;
        }

        int glyphIndex(char32_t cp) {
            const auto* begin = std::begin(fontdata::kCodepoints);
            const auto* end = std::end(fontdata::kCodepoints);
            if (cp >= 32 && cp < 127) return static_cast<int>(cp - 32);          // ASCII is stored first, in order
            const auto* it = std::find(begin, end, cp);
            return it != end ? static_cast<int>(it - begin) : static_cast<int>(U'?' - 32);
        }

        void blendLinear(Framebuffer& fb, int x, int y, const LinearRgb& c, float a) {
            if (x < 0 || y < 0 || x >= fb.width() || y >= fb.height() || a <= 0.0f) return;
            const LinearRgb under = toLinear(fb.getPixel(x, y));
            fb.setPixel(x, y, toSrgb8(under * (1.0f - a) + c * a));
        }
    }

    FontMetrics metrics(Font font) { return table(font).m; }

    int glyphCount(std::string_view s) {
        int n = 0;
        for (std::size_t i = 0; i < s.size();) { nextCodepoint(s, i); ++n; }
        return n;
    }

    int textWidth(std::string_view s, Font font) { return glyphCount(s) * table(font).m.advance; }

    int drawText(Framebuffer& fb, int x, int y, std::string_view s, Color color, Font font, Align align, double opacity) {
        const FontTable& t = table(font);
        const int width = textWidth(s, font);
        if (align == Align::Center) x -= width / 2;
        else if (align == Align::Right) x -= width;

        const LinearRgb c = toLinear(color);
        const float scale = static_cast<float>(std::clamp(opacity, 0.0, 1.0)) / 15.0f;
        int pen = x;
        for (std::size_t i = 0; i < s.size();) {
            const int g = glyphIndex(nextCodepoint(s, i));
            const unsigned char* data = t.data + static_cast<std::ptrdiff_t>(g) * t.bytesPerGlyph;
            for (int row = 0; row < t.m.height; ++row)
                for (int col = 0; col < t.m.advance; ++col) {
                    const int k = row * t.m.advance + col;
                    const unsigned char byte = data[k / 2];
                    const int nibble = (k % 2 == 0) ? (byte >> 4) : (byte & 0x0F);
                    if (nibble) blendLinear(fb, pen + col, y + row, c, static_cast<float>(nibble) * scale);
                }
            pen += t.m.advance;
        }
        return pen;
    }

    void drawLabel(Framebuffer& fb, int x, int y, std::string_view s, Color color, Color background, Font font,
                   Align align, double backgroundOpacity) {
        const FontMetrics m = metrics(font);
        const int width = textWidth(s, font);
        int left = x;
        if (align == Align::Center) left -= width / 2;
        else if (align == Align::Right) left -= width;

        // chip: padded box with its corners cut, blended over the scene
        constexpr int padX = 5, padY = 1;
        const int x0 = left - padX, x1 = left + width + padX, y0 = y - padY, y1 = y + m.height + padY;
        const LinearRgb bg = toLinear(background);
        const float a = static_cast<float>(std::clamp(backgroundOpacity, 0.0, 1.0));
        for (int py = y0; py < y1; ++py)
            for (int px = x0; px < x1; ++px) {
                const bool corner = (px == x0 || px == x1 - 1) && (py == y0 || py == y1 - 1);
                blendLinear(fb, px, py, bg, corner ? a * 0.35f : a);
            }
        drawText(fb, left, y, s, color, font, Align::Left);
    }
}
