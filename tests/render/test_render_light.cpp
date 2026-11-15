// HDR light: Gaussian beams, spots, blur, tone mapping and compositing.
#include <cmath>
#include <stdexcept>

#include "curves/TestHarness.h"
#include "optibench/render/LightBuffer.h"

using namespace ob;
using namespace ob::render;

namespace {
    constexpr double kSqrt2Pi = 2.5066282746310002;

    void testBeamEnergyAndProfile(test::Context& ctx) {
        // a beam's light per unit length is the Gaussian's integral: peak * sigma * sqrt(2 pi)
        const double sigma = 1.5, len = 200.0;
        LightBuffer h(400, 200);
        h.addBeam({100, 100.5}, {100 + len, 100.5}, LinearRgb{1, 1, 1}, sigma);
        const double expected = len * sigma * kSqrt2Pi;
        CHECK_NEAR(static_cast<double>(h.total().g) / expected, 1.0, 0.02);

        // centre line at the peak, 2 sigma away at e^-2 of it
        CHECK_NEAR(h.at(200, 100).g, 1.0, 0.02);
        CHECK_NEAR(h.at(200, 103).g, std::exp(-2.0), 0.02);

        // the same beam at any angle carries the same light
        for (const double angle : {0.3, 0.785398, 1.2, 2.5}) {
            LightBuffer d(400, 400);
            const Vec2 a{200, 200}, dir{std::cos(angle), std::sin(angle)};
            d.addBeam(a - dir * (len / 2), a + dir * (len / 2), LinearRgb{1, 1, 1}, sigma);
            CHECK_NEAR(static_cast<double>(d.total().g) / expected, 1.0, 0.02);
        }
    }

    void testBeamsJoinWithoutDoubling(test::Context& ctx) {
        // a beam drawn in two pieces holds the same light as one beam (no hot spot at the join)
        LightBuffer one(300, 100), two(300, 100);
        one.addBeam({20, 50}, {280, 50}, LinearRgb{1, 0, 0}, 1.0);
        two.addBeam({20, 50}, {150, 50}, LinearRgb{1, 0, 0}, 1.0);
        two.addBeam({150, 50}, {280, 50}, LinearRgb{1, 0, 0}, 1.0);
        CHECK_NEAR(two.total().r / one.total().r, 1.0, 0.01);
        CHECK(two.at(150, 49).r < 1.15f * one.at(150, 49).r);
    }

    void testClippingAndEdges(test::Context& ctx) {
        LightBuffer b(100, 100);
        const ClipRect clip{20, 20, 60, 60};
        b.addBeam({-50, 40}, {200, 40}, LinearRgb{1, 1, 1}, 1.0, &clip);
        bool outside = false;
        for (int y = 0; y < 100; ++y)
            for (int x = 0; x < 100; ++x)
                if ((x < 20 || x > 60 || y < 20 || y > 60) && b.at(x, y).g != 0.0f) outside = true;
        CHECK(!outside);
        CHECK(b.at(40, 39).g > 0.5f);
        // reading or adding outside the buffer is harmless
        b.add(-1, 5, LinearRgb{1, 1, 1});
        b.add(5, 1000, LinearRgb{1, 1, 1});
        CHECK(b.at(-1, 5).g == 0.0f && b.at(100, 100).g == 0.0f);
        // degenerate input does nothing
        LightBuffer e(10, 10);
        e.addBeam({5, 5}, {5, 5}, LinearRgb{1, 1, 1}, 1.0);
        e.addBeam({1, 1}, {8, 8}, LinearRgb{1, 1, 1}, 0.0);
        e.addSpot({5, 5}, LinearRgb{1, 1, 1}, -1.0);
        CHECK(e.total().g == 0.0f);
        bool threw = false;
        try { LightBuffer bad(0, 10); } catch (const std::invalid_argument&) { threw = true; }
        CHECK(threw);
    }

    void testSpotAndBlur(test::Context& ctx) {
        LightBuffer b(200, 200);
        b.addSpot({100, 100}, LinearRgb{1, 1, 1}, 3.0);
        const double expected = 2.0 * 3.14159265358979 * 9.0;      // 2 pi sigma^2
        CHECK_NEAR(static_cast<double>(b.total().g) / expected, 1.0, 0.01);

        // blurring spreads the light but keeps all of it (away from the edges)
        const float before = b.total().g;
        b.blur(6);
        CHECK_NEAR(b.total().g / before, 1.0, 1e-3);
        CHECK(b.at(100, 100).g < 1.0f && b.at(112, 100).g > 0.0f);

        // half size keeps the average
        const LightBuffer half = b.downsampled();
        CHECK(half.width() == 100 && half.height() == 100);
        CHECK_NEAR(static_cast<double>(half.total().g) * 4.0 / static_cast<double>(b.total().g), 1.0, 1e-4);
        CHECK_VEC_NEAR(Vec2(half.sampleBilinear(50, 50).g, 0), Vec2(half.at(50, 50).g, 0), 1e-6);
    }

    void testToneMap(test::Context& ctx) {
        CHECK(toneMap(LinearRgb{}, 0.6f).maxComponent() == 0.0f);
        // monotonic and below 1
        float prev = -1.0f;
        bool monotonic = true, bounded = true;
        for (int i = 0; i <= 200; ++i) {
            const float v = toneMap(LinearRgb{static_cast<float>(i) * 0.05f, 0, 0}, 0.6f).r;
            monotonic = monotonic && v >= prev;
            bounded = bounded && v <= 1.0f;
            prev = v;
        }
        CHECK(monotonic && bounded);
        // dim light passes almost unchanged
        CHECK_NEAR(toneMap(LinearRgb{0.01f, 0.02f, 0.03f}, 0.6f).g, 0.02, 5e-4);
        // a saturated red core turns towards white; without white core it stays pure red
        const LinearRgb hot = toneMap(LinearRgb{6, 0, 0}, 0.6f);
        CHECK(hot.g > 0.3f && hot.r > hot.g);
        CHECK(toneMap(LinearRgb{6, 0, 0}, 0.0f).g == 0.0f);
    }

    void testComposite(test::Context& ctx) {
        Framebuffer fb(64, 48, Color{30, 60, 90});
        const Framebuffer original = fb;
        LightBuffer none(64, 48);
        Look plain;
        plain.vignette = 0.0f;
        composite(fb, none, plain);
        bool same = true;
        for (int y = 0; y < 48; ++y)
            for (int x = 0; x < 64; ++x) {
                const Color a = fb.getPixel(x, y), b = original.getPixel(x, y);
                same = same && a.r == b.r && a.g == b.g && a.b == b.b;
            }
        CHECK(same);                                    // no light, no vignette: the image is untouched

        LightBuffer light(64, 48);
        light.addSpot({32, 24}, LinearRgb{0, 3, 0}, 2.0);
        composite(fb, light, plain);
        CHECK(fb.getPixel(32, 24).g > 200);             // the spot shows
        CHECK(fb.getPixel(32, 34).g > original.getPixel(32, 34).g);       // and blooms around it

        bool threw = false;
        try { composite(fb, LightBuffer(10, 10)); } catch (const std::invalid_argument&) { threw = true; }
        CHECK(threw);
    }
}

int main() {
    return test::runTests("render: light", {
        {"beam energy and Gaussian profile", testBeamEnergyAndProfile},
        {"beams join without doubling", testBeamsJoinWithoutDoubling},
        {"clipping, edges and degenerate input", testClippingAndEdges},
        {"spots, blur and downsampling keep the light", testSpotAndBlur},
        {"tone map", testToneMap},
        {"composite", testComposite},
    });
}
