// Colour science: sRGB transfer, CIE matching functions, wavelength colours, white-light sampling.
#include <cmath>

#include "curves/TestHarness.h"
#include "optibench/render/Spectrum.h"

using namespace ob;
using namespace ob::render;

namespace {
    void testSrgbRoundTrip(test::Context& ctx) {
        for (int i = 0; i <= 100; ++i) {
            const float x = static_cast<float>(i) / 100.0f;
            CHECK_NEAR(linearToSrgb(srgbToLinear(x)), x, 1e-5);
        }
        CHECK_NEAR(srgbToLinear(0.5f), 0.214041, 1e-5);                // reference value of the sRGB curve
        // every 8-bit value survives decode + encode exactly
        bool exact = true;
        for (int v = 0; v < 256; ++v) {
            const auto b = static_cast<std::uint8_t>(v);
            const Color c = toSrgb8(toLinear(Color{b, b, b}));
            exact = exact && c.r == b && c.g == b && c.b == b;
        }
        CHECK(exact);
        // out-of-range light is clamped, not wrapped
        const Color hot = toSrgb8(LinearRgb{5.0f, -1.0f, 0.5f});
        CHECK(hot.r == 255 && hot.g == 0 && hot.b == 188);
    }

    void testColourMatchingFunctions(test::Context& ctx) {
        // the luminosity function y-bar peaks near 555 nm at about 1
        double peakNm = 0, peak = 0;
        for (double nm = 380; nm <= 780; nm += 1) {
            const double y = cieXyz(nm).g;
            if (y > peak) { peak = y; peakNm = nm; }
        }
        CHECK(peakNm >= 550 && peakNm <= 562);
        CHECK_NEAR(peak, 1.0, 0.05);
        // the eye barely sees the ends of the range
        CHECK(cieXyz(380).g < 0.01f && cieXyz(780).g < 0.01f);
        // equal-energy white (sum over the spectrum) has x = y = z within a few percent
        LinearRgb sum;
        for (double nm = 380; nm <= 780; nm += 1) sum += cieXyz(nm);
        CHECK_NEAR(sum.r / sum.g, 1.0, 0.03);
        CHECK_NEAR(sum.b / sum.g, 1.0, 0.03);
    }

    void testWavelengthColours(test::Context& ctx) {
        const LinearRgb blue = wavelengthToLinear(450), green = wavelengthToLinear(530), red = wavelengthToLinear(650);
        CHECK(blue.b > blue.g && blue.b > blue.r);
        CHECK(green.g > green.r && green.g > green.b);
        CHECK(red.r > red.g && red.r > red.b);
        // never negative (out-of-gamut parts are clipped)
        bool nonNegative = true;
        for (double nm = 380; nm <= 780; nm += 2) {
            const LinearRgb c = wavelengthToLinear(nm);
            nonNegative = nonNegative && c.r >= 0 && c.g >= 0 && c.b >= 0;
        }
        CHECK(nonNegative);
        // the red share rises from green to red light
        auto redShare = [](double nm) { const LinearRgb c = wavelengthToLinear(nm); return c.r / (c.r + c.g + c.b); };
        CHECK(redShare(560) < redShare(600) && redShare(600) < redShare(650));
        // laser colours are normalised to a maximum of 1
        for (double nm : {405.0, 450.0, 532.0, 589.0, 633.0, 650.0}) CHECK_NEAR(laserColor(nm).maxComponent(), 1.0, 1e-6);
        CHECK(laserColor(200).maxComponent() == 0.0f);                    // ultraviolet: invisible
    }

    void testWhiteLightSamples(test::Context& ctx) {
        const auto s = whiteLightSamples(61);
        CHECK(s.size() == 61);
        CHECK_NEAR(s.front().wavelengthNm, 400.0, 1e-12);
        CHECK_NEAR(s.back().wavelengthNm, 700.0, 1e-12);
        CHECK_NEAR(s[1].wavelengthNm - s[0].wavelengthNm, 5.0, 1e-12);
        // white balanced: the rays add up to exactly white
        LinearRgb sum;
        for (const SpectralSample& x : s) sum += x.weight;
        CHECK_NEAR(sum.r, 1.0, 1e-5);
        CHECK_NEAR(sum.g, 1.0, 1e-5);
        CHECK_NEAR(sum.b, 1.0, 1e-5);
        CHECK(whiteLightSamples(0).empty());
        CHECK(whiteLightSamples(1).size() == 1 && whiteLightSamples(1)[0].wavelengthNm == 550.0);
    }
}

int main() {
    return test::runTests("render: spectrum", {
        {"sRGB transfer round trip", testSrgbRoundTrip},
        {"CIE colour matching functions", testColourMatchingFunctions},
        {"wavelength colours", testWavelengthColours},
        {"white-light samples are white balanced", testWhiteLightSamples},
    });
}
