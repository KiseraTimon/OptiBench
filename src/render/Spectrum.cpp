#include "optibench/render/Spectrum.h"

#include <algorithm>
#include <array>
#include <cmath>

namespace ob::render {
    namespace {
        // piecewise Gaussian: different widths left and right of the peak
        double lobe(double x, double mu, double sigmaLeft, double sigmaRight) {
            const double s = (x < mu) ? sigmaLeft : sigmaRight;
            const double t = (x - mu) / s;
            return std::exp(-0.5 * t * t);
        }

        const std::array<float, 256>& decodeTable() {
            static const std::array<float, 256> table = [] {
                std::array<float, 256> t{};
                for (int i = 0; i < 256; ++i) t[static_cast<std::size_t>(i)] = srgbToLinear(static_cast<float>(i) / 255.0f);
                return t;
            }();
            return table;
        }

        // linear -> 8-bit sRGB through a 4096-entry table (finer than 8 bits where it matters, near black)
        constexpr int kEncodeSize = 4096;
        const std::array<std::uint8_t, kEncodeSize + 1>& encodeTable() {
            static const std::array<std::uint8_t, kEncodeSize + 1> table = [] {
                std::array<std::uint8_t, kEncodeSize + 1> t{};
                for (int i = 0; i <= kEncodeSize; ++i) {
                    const float v = linearToSrgb(static_cast<float>(i) / static_cast<float>(kEncodeSize));
                    t[static_cast<std::size_t>(i)] = static_cast<std::uint8_t>(std::lround(v * 255.0f));
                }
                return t;
            }();
            return table;
        }

        std::uint8_t encode(float linear) {
            const float c = std::clamp(linear, 0.0f, 1.0f);
            return encodeTable()[static_cast<std::size_t>(std::lround(c * static_cast<float>(kEncodeSize)))];
        }
    }

    float srgbToLinear(float e) {
        return e <= 0.04045f ? e / 12.92f : std::pow((e + 0.055f) / 1.055f, 2.4f);
    }

    float linearToSrgb(float l) {
        return l <= 0.0031308f ? 12.92f * l : 1.055f * std::pow(l, 1.0f / 2.4f) - 0.055f;
    }

    LinearRgb toLinear(Color c) {
        const auto& t = decodeTable();
        return {t[c.r], t[c.g], t[c.b]};
    }

    Color toSrgb8(const LinearRgb& c) { return Color{encode(c.r), encode(c.g), encode(c.b)}; }

    LinearRgb cieXyz(double nm) {
        const double x = 1.056 * lobe(nm, 599.8, 37.9, 31.0) + 0.362 * lobe(nm, 442.0, 16.0, 26.7) -
                         0.065 * lobe(nm, 501.1, 20.4, 26.2);
        const double y = 0.821 * lobe(nm, 568.8, 46.9, 40.5) + 0.286 * lobe(nm, 530.9, 16.3, 31.1);
        const double z = 1.217 * lobe(nm, 437.0, 11.8, 36.0) + 0.681 * lobe(nm, 459.0, 26.0, 13.8);
        return {static_cast<float>(x), static_cast<float>(y), static_cast<float>(z)};
    }

    LinearRgb xyzToLinearSrgb(const LinearRgb& c) {
        return {3.2406f * c.r - 1.5372f * c.g - 0.4986f * c.b,
                -0.9689f * c.r + 1.8758f * c.g + 0.0415f * c.b,
                0.0557f * c.r - 0.2040f * c.g + 1.0570f * c.b};
    }

    LinearRgb wavelengthToLinear(double nm) {
        if (nm < 360.0 || nm > 830.0) return {};      // outside the CIE 1931 tables: invisible
        const LinearRgb rgb = xyzToLinearSrgb(cieXyz(nm));
        return {std::max(rgb.r, 0.0f), std::max(rgb.g, 0.0f), std::max(rgb.b, 0.0f)};
    }

    LinearRgb laserColor(double nm) {
        const LinearRgb c = wavelengthToLinear(nm);
        const float m = c.maxComponent();
        return m > 0.0f ? c * (1.0f / m) : LinearRgb{};
    }

    std::vector<SpectralSample> whiteLightSamples(int count, double lo, double hi) {
        std::vector<SpectralSample> samples;
        if (count <= 0) return samples;
        samples.reserve(static_cast<std::size_t>(count));
        LinearRgb sum;
        for (int i = 0; i < count; ++i) {
            const double nm = count == 1 ? (lo + hi) / 2.0 : lo + (hi - lo) * i / (count - 1);
            const LinearRgb c = wavelengthToLinear(nm);
            samples.push_back({nm, c});
            sum += c;
        }
        // white balance: scale each channel so the rays add up to (1, 1, 1)
        const LinearRgb gain{sum.r > 0 ? 1.0f / sum.r : 0.0f, sum.g > 0 ? 1.0f / sum.g : 0.0f, sum.b > 0 ? 1.0f / sum.b : 0.0f};
        for (SpectralSample& s : samples) s.weight = s.weight * gain;
        return samples;
    }
}
