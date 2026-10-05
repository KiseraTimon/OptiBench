#include "../include/optibench/Framebuffer.h"

#include <cstdint>
#include <filesystem>
#include <fstream>

namespace ob {
    // constructor
    Framebuffer::Framebuffer(int width, int height, Color background)
        : w_(width), h_(height), pixels_(static_cast<std::size_t>(width) * (height), background) {}

    // logic clearing the canvas by painting it a single color
    void Framebuffer::clear(Color c) {
        for (auto& p : pixels_) p = c;
    }

    // logic mapping points by changing the color of a specific pixel
    void Framebuffer::setPixel(int x, int y, Color c) {
        // safety check ensuring x/y coordinates are in bounds
        if (x < 0 || y < 0 || x >= w_ || y >= h_) return;

        pixels_[static_cast<std::size_t>(y) * w_ + x] = c;
    }

    // logic to read a point through a specific pixel color
    Color Framebuffer::getPixel(int x, int y) const {
        // safety check treating out-of-bounds coordinates are black
        if (x < 0 || y < 0 || x >= w_ || y >= h_) return colors::Black;

        return pixels_[static_cast<std::size_t>(y) * w_ + x];
    }

    // logic to save the canvas as a .bmp image
    bool Framebuffer::saveBMP(const std::string &path) const {
        // filesystem reference
        namespace fs = std::filesystem;

        // converting path text into an actual file path object
        const fs::path p(path);

        // checks and resolves for nested file paths
        if (p.has_parent_path()) fs::create_directories(p.parent_path());

        // opening a new file at the path; fails with a false if not successful
        std::ofstream f(p, std::ios::binary);
        if (!f) return false;

        /**
         * Row size calculations:
         *      BMP images require each row of pixels to be
         *      a multiple of 4 bytes long
         */
        const int rowBytes = (w_ * 3 + 3) & ~3;

        /**
         * calculating total size of the raw image:
         *      padded row size * canvas height
         */
        const std::uint32_t imageSize = static_cast<std::uint32_t>(rowBytes) * h_;

        // writes for the 32-bit & 16-bit numbers into the file
        auto w32 = [&](std::uint32_t v) {for (int i = 0; i < 4; ++i) f.put(static_cast<char>((v >> (8 * i)) & 0xFF)); };
        auto w16 = [&](std::uint16_t v) { f.put(static_cast<char>(v & 0xFF)); f.put(static_cast<char>((v >> 8) & 0xFF)); };

        // writing the file header; 14 bytes
        f.put('B'); f.put('M');     // tells the program this is a BMP file
        w32(54 + imageSize); w32(0); w32(54);

        // info header (40 bytes)
        w32(40); w32(static_cast<std::uint32_t>(w_)); w32(static_cast<std::uint32_t>(h_));
        w16(1); w16(24); w32(0); w32(imageSize); w32(2835); w32(2835); w32(0); w32(0);

        // pixel data; writing the actual colors
        std::vector<char> row(static_cast<std::size_t>(rowBytes), 0);

        // looping every row of the image
        for (int y = h_ - 1; y >= 0; --y) {
            // looping every pixel; left to right in the current row
            for (int x = 0; x < w_; ++x) {
                // color look-up for the exact pixel
                const Color& c = pixels_[static_cast<std::size_t>(y) * w_ + x];

                // putting blue color in a temporary row
                row[3 * x + 0] = static_cast<char>(c.b);

                // putting green color in a temp row
                row[3 * x + 1] = static_cast<char>(c.g);

                // putting red color in a temp row
                row[3 * x + 2] = static_cast<char>(c.r);

                /**
                 * CONTEXT:
                 * BMP stores colors in a BGR order
                 */
            }

            // writing the entire filled row of colors
            f.write(row.data(), rowBytes);
        }

        // saving the file; returns true if successful
        return static_cast<bool>(f);
    };
}
