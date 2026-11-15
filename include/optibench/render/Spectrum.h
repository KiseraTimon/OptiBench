#pragma once
#include <vector>

#include "optibench/Color.h"

/**
 * Colour science for the renderer: linear-light RGB, sRGB encoding, and the colour of a wavelength.
 *
 * Light adds up in LINEAR space (two beams on one pixel are twice as bright), but image files store
 * sRGB, which is gamma-encoded. Everything the renderer accumulates is linear; it is encoded once,
 * at the very end.
 */
namespace ob::render {
    // a linear-light colour; components may exceed 1 (high dynamic range)
    struct LinearRgb {
        float r = 0.0f, g = 0.0f, b = 0.0f;

        constexpr LinearRgb operator+(const LinearRgb& o) const { return {r + o.r, g + o.g, b + o.b}; }
        constexpr LinearRgb operator*(float s) const { return {r * s, g * s, b * s}; }
        constexpr LinearRgb operator*(const LinearRgb& o) const { return {r * o.r, g * o.g, b * o.b}; }
        LinearRgb& operator+=(const LinearRgb& o) { r += o.r; g += o.g; b += o.b; return *this; }
        constexpr float maxComponent() const { return r > g ? (r > b ? r : b) : (g > b ? g : b); }
        // relative luminance (Rec. 709 / sRGB primaries)
        constexpr float luminance() const { return 0.2126f * r + 0.7152f * g + 0.0722f * b; }
    };

    // sRGB transfer functions (IEC 61966-2-1), one channel in [0, 1]
    float srgbToLinear(float encoded);
    float linearToSrgb(float linear);

    // 8-bit sRGB <-> linear (fast: table lookup one way, clamped curve the other)
    LinearRgb toLinear(Color c);
    Color toSrgb8(const LinearRgb& c);      // clamps to [0, 1] first

    /**
     * CIE 1931 2-degree colour matching functions at a wavelength (nm), using the multi-lobe Gaussian
     * fit of Wyman, Sloan & Shirley, "Simple Analytic Approximations to the CIE XYZ Color Matching
     * Functions" (JCGT 2013). Returns X, Y, Z in the r, g, b fields.
     */
    LinearRgb cieXyz(double wavelengthNm);

    // CIE XYZ -> linear sRGB (D65). May be negative: spectral colours lie outside the sRGB gamut.
    LinearRgb xyzToLinearSrgb(const LinearRgb& xyz);

    /**
     * the displayable colour of monochromatic light: XYZ -> linear sRGB, negative components clipped.
     * Black outside 360-830 nm (ultraviolet and infrared).
     * Not normalised: brightness follows the eye's sensitivity, so 555 nm is brightest and the ends of
     * the spectrum fade to black, as they do on a real screen.
     */
    LinearRgb wavelengthToLinear(double wavelengthNm);

    // the same colour normalised so its largest component is 1 (the colour to paint a laser of that line)
    LinearRgb laserColor(double wavelengthNm);

    /**
     * a white beam split into `count` monochromatic rays between lo and hi nm, evenly spaced.
     * The weights are white-balanced: summed over all rays they give exactly (1, 1, 1), so the rays
     * look white where they overlap and spread into a spectrum where a prism separates them.
     */
    struct SpectralSample {
        double wavelengthNm;
        LinearRgb weight;
    };
    std::vector<SpectralSample> whiteLightSamples(int count, double lo = 400.0, double hi = 700.0);
}
