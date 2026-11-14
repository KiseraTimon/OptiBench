#include "optibench/Sprite.h"

#include <stdexcept>

namespace ob {
    namespace {
        void drawRegion(const Framebuffer& image, const Rect& region, int x, int y,
                        Framebuffer& destination, BlitMode mode,
                        const std::optional<Color>& colorKey) {
            if (mode == BlitMode::KeepDestination && colorKey) {
                BlitOptions options;
                options.mode = mode;
                options.colorKey = *colorKey;
                blit(image, region, destination, x, y, options);
            } else {
                blit(image, region, destination, x, y,
                     mode == BlitMode::KeepDestination ? BlitMode::Copy : mode);
            }
        }
    }

    void Sprite::draw(Framebuffer& destination, int x, int y, BlitMode mode) const {
        drawRegion(image_, {0, 0, image_.width(), image_.height()}, x, y,
                   destination, mode, colorKey_);
    }

    SpriteSheet::SpriteSheet(Framebuffer image, int frameWidth, int frameHeight,
                             std::optional<Color> colorKey)
        : image_(std::move(image)), frameWidth_(frameWidth), frameHeight_(frameHeight),
          columns_(frameWidth > 0 ? image_.width() / frameWidth : 0),
          rows_(frameHeight > 0 ? image_.height() / frameHeight : 0), colorKey_(colorKey) {
        if (frameWidth <= 0 || frameHeight <= 0)
            throw std::invalid_argument("SpriteSheet frame dimensions must be positive");
        if (columns_ == 0 || rows_ == 0)
            throw std::invalid_argument("SpriteSheet image must contain at least one complete frame");
    }

    void SpriteSheet::drawFrame(std::size_t index, Framebuffer& destination, int x, int y,
                                BlitMode mode) const {
        if (index >= frameCount()) return;
        const int column = static_cast<int>(index % static_cast<std::size_t>(columns_));
        const int row = static_cast<int>(index / static_cast<std::size_t>(columns_));
        drawRegion(image_, {column * frameWidth_, row * frameHeight_, frameWidth_, frameHeight_},
                   x, y, destination, mode, colorKey_);
    }
}
