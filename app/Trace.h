#pragma once
#include <vector>

#include "Bench.h"
#include "optibench/Tracer.h"
#include "optibench/render/Spectrum.h"

namespace ob::app {
    // one traced ray of one source, with the colour it carries
    struct TracedRay {
        BeamPath path;
        double wavelengthNm = 0.0;
        render::LinearRgb color;        // linear light; for white light, this ray's share of the white
        const SourceSpec* source = nullptr;
        int index = 0;                  // which ray of its source (0 .. rays - 1)
    };

    // traces every ray of every source through the scene, at that ray's wavelength.
    // The rays point into `sources`, so it must outlive them (temporaries are rejected).
    std::vector<TracedRay> traceSources(const Scene& scene, const std::vector<SourceSpec>& sources);
    std::vector<TracedRay> traceSources(const Scene& scene, const std::vector<SourceSpec>&& sources) = delete;
}
