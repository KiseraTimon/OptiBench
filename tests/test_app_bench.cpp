// The OptiBench application: its bench stays valid through the whole animation, and its options parse.
#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <string>
#include <vector>

#include "Bench.h"
#include "Options.h"
#include "Trace.h"
#include "curves/TestHarness.h"
#include "optibench/refraction.h"

using namespace ob;
using namespace ob::app;

namespace {
    // steps the real animation without drawing (the renderer is replaced by a no-op)
    template <class Check>
    void forEveryFrame(Check check, double fps = 15.0) {
        Scene scene = buildBench();
        DoubleBuffer buffers(8, 8);
        Animator anim(scene, buffers);
        animateBench(anim);
        int renders = 0;
        anim.setRenderer([&](Framebuffer&, const FrameInfo&) { ++renders; });
        const int frames = static_cast<int>(std::lround(kDuration * fps));
        for (int i = 0; i <= frames; ++i) {
            const FrameInfo f = anim.step(i == 0 ? 0.0 : 1.0 / fps);
            check(scene, f);
        }
        if (renders != frames + 1) throw std::runtime_error("custom renderer was not called every frame");
    }

    void testLayoutStaysValid(test::Context& ctx) {
        int frames = 0, overlaps = 0;
        forEveryFrame([&](const Scene& scene, const FrameInfo&) {
            ++frames;
            overlaps += static_cast<int>(scene.overlappingPairs().size());
        });
        CHECK(frames == 61);
        CHECK(overlaps == 0);
    }

    void testEveryRayLands(test::Context& ctx) {
        const std::vector<SourceSpec> sources = benchSources();
        int rays = 0, landed = 0, tir = 0, tirValid = 0, redTir = 0;
        forEveryFrame([&](const Scene& scene, const FrameInfo&) {
            for (const TracedRay& r : traceSources(scene, sources)) {
                ++rays;
                if (r.path.hitScreen) ++landed;
                for (const BeamEvent& e : r.path.events) {
                    if (e.kind != BeamEvent::Kind::TotalInternalReflection) continue;
                    ++tir;
                    const double n = scene.find(e.element)->material.indexAt(r.wavelengthNm);
                    const auto critical = criticalAngle(n, 1.0);
                    if (critical && e.incidenceDeg * kPi / 180.0 >= *critical) ++tirValid;
                    if (r.source->element == "laserRed" && e.element == "fold") ++redTir;
                }
            }
        });
        CHECK(rays == 61 * (1 + 61 + 9));
        CHECK(landed == rays);
        CHECK(tir == tirValid);
        CHECK(redTir == 61);                // the fold prism turns the red beam by TIR in every frame
    }

    void testSpectrumIsOrdered(test::Context& ctx) {
        const std::vector<SourceSpec> sources = benchSources();
        double minSpread = 1e9;
        bool ordered = true, onSpectrumScreen = true;
        forEveryFrame([&](const Scene& scene, const FrameInfo&) {
            double previousY = 1e9, lo = 1e9, hi = -1e9;
            for (const TracedRay& r : traceSources(scene, sources)) {
                if (r.source->kind != SourceSpec::Kind::WhiteLight) continue;
                onSpectrumScreen = onSpectrumScreen && r.path.events.back().element == "screen";
                // longer wavelengths are bent less: they land higher (smaller y), in strict order
                ordered = ordered && r.path.end.y < previousY;
                previousY = r.path.end.y;
                lo = std::min(lo, r.path.end.y);
                hi = std::max(hi, r.path.end.y);
            }
            minSpread = std::min(minSpread, hi - lo);
        });
        CHECK(onSpectrumScreen);
        CHECK(ordered);
        CHECK(minSpread > 120.0);
    }

    void testDishFocusesTheBundle(test::Context& ctx) {
        const std::vector<SourceSpec> sources = benchSources();
        double worst = 0.0;
        forEveryFrame([&](const Scene& scene, const FrameInfo&) {
            const Vec2 focus = scene.find("dish")->transform.toWorld({-160.0, 0.0});
            for (const TracedRay& r : traceSources(scene, sources)) {
                if (r.source->kind != SourceSpec::Kind::Collimated) continue;
                const BeamSegment& out = r.path.segments.back();          // dish -> detector
                const Vec2 d = (out.b - out.a).normalized();
                worst = std::max(worst, std::abs((focus - out.a).cross(d)));   // distance from the focus to the ray
            }
        });
        CHECK(worst < 1e-6);
    }

    void testOptions(test::Context& ctx) {
        const Options d = parseOptions({}).options;
        CHECK(d.outDir == "output" && d.fps == 15.0 && d.duration == 4.0 && d.writeFrames && !d.quiet);

        const ParseResult r = parseOptions({"--fps", "30", "--duration=2", "-o", "out dir", "--no-frames", "-q", "--still", "9"});
        CHECK(!r.error);
        CHECK(r.options.fps == 30.0 && r.options.duration == 2.0 && r.options.outDir == "out dir");
        CHECK(!r.options.writeFrames && r.options.quiet);
        CHECK(r.options.stillTime == 2.0);                   // clamped to the duration

        CHECK(parseOptions({"--help"}).options.help);
        CHECK(parseOptions({"--version"}).options.version);
        CHECK(parseOptions({"--bogus"}).error.has_value());
        CHECK(parseOptions({"--fps"}).error.has_value());
        CHECK(parseOptions({"--fps", "abc"}).error.has_value());
        CHECK(parseOptions({"--fps", "0"}).error.has_value());
        CHECK(parseOptions({"--fps", "12x"}).error.has_value());
        CHECK(parseOptions({"--out", ""}).error.has_value());
        CHECK(!usage("OptiBench").empty());
    }
}

int main() {
    return test::runTests("app: bench", {
        {"layout stays valid through the animation", testLayoutStaysValid},
        {"every ray lands; every TIR is past the critical angle", testEveryRayLands},
        {"the spectrum is ordered and wide", testSpectrumIsOrdered},
        {"the dish focuses the collimated beam", testDishFocusesTheBundle},
        {"command-line options", testOptions},
    });
}
