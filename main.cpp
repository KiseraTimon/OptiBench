/*
 * OptiBench: the final combined scene.
 *
 * One optical bench, ray traced and rendered frame by frame, that uses every module of the project:
 *
 *   scene architecture   Scene, elements, Tracer: every source traced through the bench every frame
 *   geometry             a thick BK7 Lens (exact spherical faces, lensmaker focus), a CONCAVE notched
 *                        prism, anti-aliased shading from signed distances, the broad-phase bounding
 *                        boxes and an overlap check of the layout every frame
 *   curves               a parabolic mirror whose face is an exact quadratic Bezier, and the Bezier
 *                        motion path the lens travels along
 *   transform & optics   Transform2D on every element; real glass dispersion (catalogue n_d and Abbe
 *                        numbers), a white beam split into a spectrum, every total internal reflection
 *                        checked against criticalAngle(); a Mat3 (T * S * T) maps the bench to the minimap
 *   animation            Animator + Clock (delta time), keyframe Tracks with easing, a Bezier MotionPath
 *                        timed by a CubicBezierEasing, double buffering
 *   raster ops & sprites photon sprite sheets (colour-keyed), an XOR selection box, alpha-blended labels
 *                        and panels, the minimap blitted into place
 *   rendering            HDR light with Gaussian beam profiles, bloom, tone mapping, CIE spectral colour,
 *                        anti-aliased text (app/Studio.cpp, include/optibench/render/)
 *
 * Run from the repository root:  ./build/OptiBench --help
 */

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <filesystem>
#include <string>
#include <system_error>
#include <vector>

#ifdef _WIN32
#include <io.h>
#define OB_ISATTY(f) _isatty(_fileno(f))
#else
#include <unistd.h>
#define OB_ISATTY(f) isatty(fileno(f))
#endif

#include "Bench.h"
#include "Options.h"
#include "Studio.h"
#include "optibench/Animator.h"
#include "optibench/DoubleBuffer.h"

#ifndef OPTIBENCH_VERSION
#define OPTIBENCH_VERSION "dev"
#endif

using namespace ob;
using namespace ob::app;

namespace {
    // copies src at half size (2x2 box average) to (ox, oy) in dst
    void pasteHalf(const Framebuffer& src, Framebuffer& dst, int ox, int oy) {
        for (int y = 0; y + 1 < src.height(); y += 2)
            for (int x = 0; x + 1 < src.width(); x += 2) {
                const Color a = src.getPixel(x, y), b = src.getPixel(x + 1, y);
                const Color c = src.getPixel(x, y + 1), d = src.getPixel(x + 1, y + 1);
                dst.setPixel(ox + x / 2, oy + y / 2,
                             Color{static_cast<std::uint8_t>((a.r + b.r + c.r + d.r) / 4),
                                   static_cast<std::uint8_t>((a.g + b.g + c.g + d.g) / 4),
                                   static_cast<std::uint8_t>((a.b + b.b + c.b + d.b) / 4)});
            }
    }

    struct Totals {
        int frames = 0, overlaps = 0, tir = 0, tirChecked = 0, rays = 0, raysOnScreen = 0;
        double minSpectrum = 1e9;
    };
}

int main(int argc, char** argv) {
    const std::vector<std::string> args(argv + 1, argv + argc);
    const std::string program = argc > 0 ? std::filesystem::path(argv[0]).filename().string() : "OptiBench";
    const ParseResult parsed = parseOptions(args);
    if (parsed.error) {
        std::fprintf(stderr, "%s: %s\n", program.c_str(), parsed.error->c_str());
        return 2;
    }
    const Options& opt = parsed.options;
    if (opt.help) { std::printf("%s", usage(program).c_str()); return 0; }
    if (opt.version) { std::printf("OptiBench %s\n", OPTIBENCH_VERSION); return 0; }

    const std::filesystem::path out(opt.outDir);
    std::error_code ec;
    std::filesystem::create_directories(opt.writeFrames ? out / "frames" : out, ec);
    if (ec) {
        std::fprintf(stderr, "%s: cannot create %s: %s\n", program.c_str(), out.string().c_str(), ec.message().c_str());
        return 1;
    }

    // --- the bench, its light sources and its animation
    Scene scene = buildBench();
    const std::vector<SourceSpec> sources = benchSources();
    DoubleBuffer buffers(Layout::kWidth, Layout::kHeight);
    Animator anim(scene, buffers);
    const std::vector<KeyframeTrack> tracks = animateBench(anim);

    // --- every frame is drawn by the studio renderer in place of the plain Scene::render
    Studio studio(StudioOptions{opt.showBounds, opt.duration, opt.fps}, tracks);
    Totals totals;
    anim.setRenderer([&](Framebuffer& back, const FrameInfo& f) {
        const FrameReport r = studio.render(back, scene, sources, f);
        ++totals.frames;
        totals.overlaps += r.overlaps;
        totals.tir += r.tir;
        totals.tirChecked += r.tirChecked;
        totals.rays += r.rays;
        totals.raysOnScreen += r.raysOnScreen;
        totals.minSpectrum = std::min(totals.minSpectrum, r.spectrumSpanPx);
    });

    // --- run offline; keep the still and six evenly spaced frames for the contact sheet
    const int lastFrame = static_cast<int>(std::lround(opt.duration * opt.fps));
    const int stillFrame = static_cast<int>(std::lround(opt.stillTime * opt.fps));
    std::vector<int> sheetFrames;
    for (int i = 0; i < 6; ++i) sheetFrames.push_back(static_cast<int>(std::lround(lastFrame * i / 5.0)));
    std::vector<Framebuffer> kept;
    bool wroteStill = false, writeFailed = false;
    const bool progress = !opt.quiet && OB_ISATTY(stdout);      // a live counter only on a terminal

    const std::string stillPath = (out / "final_scene.bmp").string();
    const int frames = anim.run(opt.duration, opt.fps, [&](const Framebuffer& front, const FrameInfo& f) {
        if (opt.writeFrames) {
            char name[32];
            std::snprintf(name, sizeof name, "final_%04d.bmp", f.index);
            const std::string path = (out / "frames" / name).string();
            if (!front.saveBMP(path)) {
                std::fprintf(stderr, "%s: failed to write %s\n", program.c_str(), path.c_str());
                writeFailed = true;
                return false;
            }
        }
        if (f.index == stillFrame) wroteStill = front.saveBMP(stillPath);
        if (opt.writeSheet && std::find(sheetFrames.begin(), sheetFrames.end(), f.index) != sheetFrames.end()) kept.push_back(front);
        if (progress) {
            std::printf("\r  rendering frame %3d / %d", f.index, lastFrame);
            std::fflush(stdout);
        }
        return true;
    });
    if (progress) std::printf("\r%40s\r", "");

    bool wroteSheet = true;
    if (opt.writeSheet && !kept.empty()) {
        Framebuffer sheet(Layout::kWidth, Layout::kHeight / 2 * 3, colors::BenchDark);
        for (std::size_t i = 0; i < kept.size(); ++i)
            pasteHalf(kept[i], sheet, static_cast<int>(i % 2) * (Layout::kWidth / 2), static_cast<int>(i / 2) * (Layout::kHeight / 2));
        wroteSheet = sheet.saveBMP((out / "final_scene_sheet.bmp").string());
    }

    const bool tirOk = totals.tir == totals.tirChecked;
    const bool ok = !writeFailed && wroteStill && wroteSheet && frames == lastFrame + 1 && totals.overlaps == 0 && tirOk;

    if (!opt.quiet || !ok) {
        std::printf("OptiBench %s: %d frames (%g s at %g fps)\n", OPTIBENCH_VERSION, frames, opt.duration, opt.fps);
        std::printf("  overlapping element pairs over all frames: %d\n", totals.overlaps);
        std::printf("  total internal reflections: %d (all at or past the critical angle: %s)\n", totals.tir, tirOk ? "yes" : "NO");
        std::printf("  rays traced: %d, landing on a screen: %d\n", totals.rays, totals.raysOnScreen);
        std::printf("  spectrum (400-700 nm) spread on the screen: at least %.0f px\n", totals.minSpectrum);
        if (!wroteStill) std::printf("  FAILED to write %s\n", stillPath.c_str());
        if (!wroteSheet) std::printf("  FAILED to write the contact sheet\n");
        if (ok)
            std::printf("  wrote %s%s%s\n", stillPath.c_str(), opt.writeSheet ? ", final_scene_sheet.bmp" : "",
                        opt.writeFrames ? " and frames/final_*.bmp" : "");
    }
    return ok ? 0 : 1;
}
