#pragma once
#include <vector>

#include "Bench.h"
#include "Trace.h"
#include "optibench/Animator.h"
#include "optibench/Framebuffer.h"
#include "optibench/Sprite.h"
#include "optibench/render/LightBuffer.h"

/**
 * Studio: the renderer of the final scene.
 *
 * Per frame:
 *   1. backdrop      optical breadboard, rulers, header and HUD panels (drawn once, then copied)
 *   2. elements      shaded per pixel with anti-aliased edges: tinted glass with a bright rim,
 *                    brushed-metal mirrors, matte screens, laser housings
 *   3. light         every traced ray into an HDR LightBuffer with a Gaussian beam profile; glow inside
 *                    glass, spots where beams land, glowing indicator LEDs and photons
 *   4. composite     two-scale bloom, tone map, vignette (render::composite)
 *   5. overlays      event markers, foci, labels with live optics readouts, the spectrum's wavelength
 *                    scale, sprite photons, the XOR selection box
 *   6. HUD           header, Mat3 minimap, beam readouts, checks and a keyframe timeline
 */
namespace ob::app {
    // what one frame measured (also used for the exit status)
    struct FrameReport {
        int overlaps = 0;               // overlapping element pairs (must be 0)
        int tir = 0, tirChecked = 0;    // TIR events, and how many were at or past the critical angle
        int rays = 0, raysOnScreen = 0; // traced rays, and how many ended on a screen
        double tirIncidenceDeg = 0.0;   // the red beam at the fold prism's hypotenuse
        double tirCriticalDeg = 0.0;
        double spectrumSpanPx = 0.0;    // 400 nm to 700 nm on the spectrum screen
    };

    struct StudioOptions {
        bool showBounds = true;
        double duration = kDuration;
        double fps = 15.0;
    };

    class Studio {
    public:
        Studio(StudioOptions options, std::vector<KeyframeTrack> tracks);

        // draws one complete frame of the bench into fb (Layout::kWidth x Layout::kHeight)
        FrameReport render(Framebuffer& fb, const Scene& scene, const std::vector<SourceSpec>& sources,
                           const FrameInfo& frame);

    private:
        StudioOptions opt_;
        std::vector<KeyframeTrack> tracks_;
        Framebuffer backdrop_;
        render::LightBuffer light_;
        std::vector<SpriteSheet> photons_;      // red, white, blue

        void buildBackdrop();
        void drawElements(Framebuffer& fb, const Scene& scene) const;
        void addLight(const Scene& scene, const std::vector<TracedRay>& rays, const FrameInfo& f);
        void drawOverlays(Framebuffer& fb, const Scene& scene, const std::vector<TracedRay>& rays,
                          const FrameInfo& f, FrameReport& report) const;
        void drawHud(Framebuffer& fb, const Scene& scene, const std::vector<SourceSpec>& sources,
                     const std::vector<TracedRay>& rays, const FrameInfo& f, const FrameReport& report) const;
    };
}
