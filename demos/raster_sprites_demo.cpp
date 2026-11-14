#include "optibench/Sprite.h"
#include "optibench/Line.h"
#include "optibench/circle_ellipse.h"

#include <cstdio>
#include <utility>

int main() {
    using namespace ob;
    Framebuffer canvas(640, 360, colors::BenchDark);

    // A tiny two-frame sheet: each frame is a colored optical marker on a keyed background.
    Framebuffer sheetPixels(48, 24, colors::Black);
    for (int y = 4; y < 20; ++y) for (int x = 4; x < 20; ++x) {
        if ((x - 12) * (x - 12) + (y - 12) * (y - 12) < 60) sheetPixels.setPixel(x, y, colors::Cyan);
        if ((x - 36) * (x - 36) + (y - 12) * (y - 12) < 60) sheetPixels.setPixel(x, y, colors::Yellow);
    }
    SpriteSheet markers(std::move(sheetPixels), 24, 24);
    markers.drawFrame(0, canvas, 40, 35);
    markers.drawFrame(1, canvas, 80, 35);

    // Demonstrate copy, color-key keep-destination, and channel Boolean operations.
    Framebuffer patch(32, 32, colors::Black);
    for (int y = 0; y < 32; ++y) for (int x = 0; x < 32; ++x)
        if ((x - 16) * (x - 16) + (y - 16) * (y - 16) < 180) patch.setPixel(x, y, {220, 90, 35});
    const BlitMode modes[] = {BlitMode::Copy, BlitMode::KeepDestination,
                              BlitMode::And, BlitMode::Or, BlitMode::Xor};
    for (int i = 0; i < 5; ++i) {
        Framebuffer base(32, 32, {30, 70, 160});
        blit(patch, {0, 0, 32, 32}, base, 0, 0, modes[i]);
        blit(base, {0, 0, 32, 32}, canvas, 190 + i * 48, 35);
    }

    // A soft beam with its bright antialiased core.
    const Color beam{255, 70, 35};
    for (int x = 55; x < 590; ++x) {
        const double wave = 175.0 + 22.0 * (x - 55) / 535.0;
        for (int y = 0; y < canvas.height(); ++y) {
            const double d = y - wave;
            if (d > -12.0 && d < 12.0) {
                const double falloff = 1.0 - (d * d) / 144.0;
                blendPixel(canvas, x, y, beam, 0.20 * falloff * falloff);
            }
        }
    }
    drawLineWu(canvas, {55.0, 175.0}, {590.0, 197.0}, colors::Yellow);
    drawEllipse(canvas, 590, 197, 9, 9, colors::White);

    if (!canvas.saveBMP("output/raster_sprites_demo.bmp")) {
        std::puts("Could not write output/raster_sprites_demo.bmp");
        return 1;
    }
    std::puts("Wrote output/raster_sprites_demo.bmp");
    return 0;
}
