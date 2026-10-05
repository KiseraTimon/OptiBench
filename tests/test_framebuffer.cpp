#include <cstdio>
#include "../include/optibench/Framebuffer.h"


// test definition
#define CHECK(cond) do { if (!(cond)) { std::printf("FAIL %s:%d  %s\n", __FILE__, __LINE__, #cond); return 1; } } while (0)


int main() {
    using namespace ob;

    // framebuffer instance
    Framebuffer fb(10, 10, colors::Black);

    // changing point(3,4) to color red
    fb.setPixel(3, 4, colors::Red);
    CHECK(fb.getPixel(3, 4).r == colors::Red.r);    // validation

    // testing off-screen writes do not induce a crash
    fb.setPixel(-1, 0, colors::Red);
    fb.setPixel(10, 10, colors::Red);
    CHECK(fb.getPixel(-1, 0).r == 0);

    // testing file saves
    CHECK(fb.saveBMP("output/test_framebuffer.bmp"));

    // successful test
    std::puts("test_framebuffer: PASS");
    return 0;
}