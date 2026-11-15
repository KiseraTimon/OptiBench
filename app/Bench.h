#pragma once
#include <string>
#include <vector>

#include "optibench/Animator.h"
#include "optibench/Scene.h"
#include "optibench/clip_fill.h"
#include "optibench/curves/Bezier.h"

/**
 * The final bench: what is on it, how the light is emitted, and how it moves.
 * Kept apart from the renderer (Studio) so tests and tools can build the same bench.
 */
namespace ob::app {
    // frame layout (pixels): 720p, a header strip, the bench, a HUD strip
    struct Layout {
        static constexpr int kWidth = 1280;
        static constexpr int kHeight = 720;
        static constexpr int kHeaderBottom = 56;
        static constexpr int kHudTop = 612;
        static constexpr ClipRect kBench{24, 64, 1256, 602};
    };

    // how a source's light is modelled
    struct SourceSpec {
        enum class Kind {
            Laser,          // one monochromatic beam
            WhiteLight,     // `rays` wavelengths from 400 to 700 nm, sharing one beam until glass separates them
            Collimated      // `rays` parallel beams spread across `width` (a wide, expanded beam)
        };
        std::string element;    // the LaserSource it belongs to
        Kind kind = Kind::Laser;
        double wavelengthNm = 0.0;      // Laser / Collimated
        int rays = 1;
        double width = 0.0;             // Collimated: beam diameter (px)
        std::string label;              // shown in the HUD
    };

    // the bench's moving parts, for the HUD timeline (track name and keyframe times)
    struct KeyframeTrack {
        std::string label;
        std::vector<double> times;
    };

    constexpr double kDuration = 4.0;   // seconds; the animation loops seamlessly

    Scene buildBench();
    std::vector<SourceSpec> benchSources();

    // the Bezier arc the lens glides along (curves module)
    const curves::Bezier& lensPath();

    // binds every animation to the Animator; returns the tracks for the timeline display
    std::vector<KeyframeTrack> animateBench(Animator& anim);
}
