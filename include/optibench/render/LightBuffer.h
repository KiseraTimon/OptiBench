#pragma once
#include <vector>

#include "optibench/Framebuffer.h"
#include "optibench/Vec2.h"
#include "optibench/clip_fill.h"        // ClipRect
#include "optibench/render/Spectrum.h"

/**
 * High-dynamic-range light: beams are accumulated here additively in linear RGB, then glow (bloom),
 * get tone-mapped and are composited over the ordinary 8-bit scene.
 *
 *      LightBuffer light(w, h);
 *      light.addBeam(a, b, laserColor(650) * 1.5f, 0.9);     // a Gaussian beam profile
 *      composite(fb, light, Look{});                          // bloom + tone map + vignette
 */
namespace ob::render {
    class LightBuffer {
    public:
        LightBuffer(int width, int height);

        int width() const { return w_; }
        int height() const { return h_; }

        void clear();
        LinearRgb at(int x, int y) const;               // black outside
        void add(int x, int y, const LinearRgb& c);     // ignored outside

        /**
         * a beam from a to b with a Gaussian cross-section: `peak` on the centre line, falling off as
         * exp(-d^2 / 2 sigma^2) with the distance d from it. The ends fade over one pixel with ramps that
         * sum to 1 where two segments meet, so beams chained end to end stay uniform across the joins.
         * Only pixels inside `clip` (if given) are touched.
         */
        void addBeam(Vec2 a, Vec2 b, const LinearRgb& peak, double sigma, const ClipRect* clip = nullptr);

        // a round Gaussian spot (a beam landing on a screen, a lamp)
        void addSpot(Vec2 centre, const LinearRgb& peak, double sigma);

        // total light, summed over the image (for tests: energy bookkeeping)
        LinearRgb total() const;

        // in-place blur: three box passes approximate a Gaussian of standard deviation ~ radius
        void blur(int radius);

        // half size (2x2 average) and back (bilinear), used by bloom
        LightBuffer downsampled() const;
        LinearRgb sampleBilinear(double x, double y) const;

        const std::vector<LinearRgb>& pixels() const { return px_; }
        std::vector<LinearRgb>& pixels() { return px_; }        // row-major, width() * height()

    private:
        int w_, h_;
        std::vector<LinearRgb> px_;

        void boxBlurRows(int radius);
        void boxBlurColumns(int radius);
    };

    // how the light is turned into an image
    struct Look {
        float exposure = 1.0f;          // scales the light before tone mapping
        float bloomStrength = 0.55f;    // how much of the blurred light is added back
        int bloomNear = 3;              // blur radius of the tight glow (in half-resolution pixels)
        int bloomFar = 14;              // blur radius of the wide halo
        float whiteCore = 0.6f;         // how strongly very bright light desaturates towards white
        float vignette = 0.32f;         // corner darkening, 0 = none
    };

    /**
     * light -> image: adds a two-scale bloom, tone maps the light with a soft shoulder
     * (1 - e^-x per channel, then over-bright light bleeds into white the way laser cores do on camera),
     * adds it to the scene in linear space, applies the vignette and writes sRGB back into fb.
     * fb and light must be the same size.
     */
    void composite(Framebuffer& fb, const LightBuffer& light, const Look& look = {});

    // the tone curve on its own (for tests and the HUD): HDR linear -> displayable linear in [0, 1)
    LinearRgb toneMap(const LinearRgb& c, float whiteCore);
}
