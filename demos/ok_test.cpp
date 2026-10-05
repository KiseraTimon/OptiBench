#include <cstdio>
#include "../include/optibench/Framebuffer.h"

int main() {
    using namespace ob;

    // framebuffer instance
    Framebuffer fb(256, 256);

    // sample .bmp file
    for (int y = 0; y < 256; ++y)
        for (int x = 0; x < 256; ++x)
            fb.setPixel(x, y, Color{static_cast<std::uint8_t>(x), static_cast<std::uint8_t>(y), 128});

    if (!fb.saveBMP("output/ok_test.bmp")) {
        std::puts("FAILED to write output/ok_test.bmp");
        return 1;
    }
    std::puts("OK - wrote output/ok_test.bmp (a colour gradient)");
    return 0;
}
