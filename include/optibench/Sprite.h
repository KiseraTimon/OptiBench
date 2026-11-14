#pragma once

#include <cstddef>
#include <optional>
#include <utility>

#include "Framebuffer.h"
#include "raster_ops.h"

namespace ob {
    /** A pixel image that can be composited into another framebuffer. */
    class Sprite {
    public:
        explicit Sprite(Framebuffer image, std::optional<Color> colorKey = colors::Black)
            : image_(std::move(image)), colorKey_(colorKey) {}

        int width() const { return image_.width(); }
        int height() const { return image_.height(); }
        const Framebuffer& image() const { return image_; }
        std::optional<Color> colorKey() const { return colorKey_; }

        void draw(Framebuffer& destination, int x, int y,
                  BlitMode mode = BlitMode::KeepDestination) const;

    private:
        Framebuffer image_;
        std::optional<Color> colorKey_;
    };

    /** Row-major fixed-size frames stored in one framebuffer. The sheet owns its pixels. */
    class SpriteSheet {
    public:
        SpriteSheet(Framebuffer image, int frameWidth, int frameHeight,
                    std::optional<Color> colorKey = colors::Black);

        int frameWidth() const { return frameWidth_; }
        int frameHeight() const { return frameHeight_; }
        int columns() const { return columns_; }
        int rows() const { return rows_; }
        std::size_t frameCount() const { return static_cast<std::size_t>(columns_) * rows_; }

        // Invalid indices are ignored. Partial frames at the sheet's right/bottom edge are excluded.
        void drawFrame(std::size_t index, Framebuffer& destination, int x, int y,
                       BlitMode mode = BlitMode::KeepDestination) const;

    private:
        Framebuffer image_;
        int frameWidth_;
        int frameHeight_;
        int columns_;
        int rows_;
        std::optional<Color> colorKey_;
    };
}
