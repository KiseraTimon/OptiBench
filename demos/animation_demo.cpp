// Animation demo: the periscope bench from scene_demo, brought to life by the animation engine.
//
//   mirrorTop   keyframed rotation, quadratic ease-in-out / ease-out segments (Track)
//   laser       keyframed position slide, ease-out then ease-in (Vec2Track)
//   lens        glides along a cubic Bezier MOTION PATH (where it goes), timed by a cubic Bezier
//               EASING curve (when it gets there), at constant speed along the path
//   prism       turns with an overshooting Bezier easing (back-out)
//
// Every frame: delta-time clock -> tracks sampled -> transforms written -> scene rendered into the
// back buffer -> overlays (dim motion path, easing graph inset) -> swap -> front saved.
//
// Writes:
//      output/frames/anim_0000.bmp ...   every frame (4 s at 15 fps = 61 frames)
//      output/animation_demo.bmp         contact sheet: six frames at half size, left-to-right, top-to-bottom
//
// Stitch the frames into a video with e.g.:
//      ffmpeg -framerate 15 -i output/frames/anim_%04d.bmp -pix_fmt yuv420p output/animation.mp4
#include <cstdio>
#include <vector>

#include "optibench/Animator.h"
#include "optibench/DoubleBuffer.h"
#include "optibench/Line.h"
#include "optibench/Scene.h"
#include "optibench/curves/CurveRender.h"

using namespace ob;

namespace {
    constexpr double kDuration = 4.0;   // seconds
    constexpr double kFps = 15.0;

    Scene buildScene() {
        Scene scene(ClipRect{40, 40, 760, 460});
        scene.emplace<LaserSource>("laser", Transform2D{.position = {125, 360}}, Color{255, 70, 70});
        scene.emplace<FlatElement>("mirrorLow", Transform2D{.position = {260, 360}, .rotation = -kPi / 4},
                                   38.0, Material::mirror());
        scene.emplace<FlatElement>("mirrorTop", Transform2D{.position = {260, 150}, .rotation = 3 * kPi / 4},
                                   38.0, Material::mirror());
        scene.emplace<LensElement>("lens", Transform2D{.position = {430, 180}}, 62.0);
        scene.emplace<PrismElement>("prism", Transform2D{.position = {590, 210}}, PrismElement::triangle(70.0, 120.0));
        scene.emplace<FlatElement>("screen", Transform2D{.position = {735, 265}, .rotation = -kPi / 2},
                                   170.0, Material::screen(), 8.0);
        return scene;
    }

    // small (time progress, eased progress) graph of an easing curve, with a dot at the current moment
    void drawEasingInset(Framebuffer& fb, const CubicBezierEasing& ease, double u) {
        const Vec2 origin{612, 440};            // bottom-left corner of the graph
        const double size = 80.0;
        auto toScreen = [&](Vec2 p) { return Vec2{origin.x + p.x * size, origin.y - p.y * size}; };

        const Color frame{90, 96, 120}, diagonal{50, 54, 72};
        drawLine(fb, toScreen({0, 0}), toScreen({1, 0}), frame);
        drawLine(fb, toScreen({0, 0}), toScreen({0, 1}), frame);
        drawLine(fb, toScreen({1, 0}), toScreen({1, 1}), diagonal);
        drawLine(fb, toScreen({0, 1}), toScreen({1, 1}), diagonal);
        drawLine(fb, toScreen({0, 0}), toScreen({1, 1}), diagonal);                // linear, for comparison

        // the easing curve itself: Bezier curves are affine invariant, so map the control points and draw
        std::vector<Vec2> mapped;
        for (const Vec2& p : ease.curve().controlPoints()) mapped.push_back(toScreen(p));
        const curves::Bezier onScreen(mapped);
        curves::drawCurve(fb, onScreen, colors::Cyan, 0.25, true);

        // control handles P0-P1 and P2-P3
        drawLine(fb, mapped[0], mapped[1], frame);
        drawLine(fb, mapped[3], mapped[2], frame);

        // the current moment
        const Vec2 dot = toScreen({u, ease(u)});
        for (int dy = -2; dy <= 2; ++dy)
            for (int dx = -2; dx <= 2; ++dx)
                fb.setPixel(static_cast<int>(dot.x) + dx, static_cast<int>(dot.y) + dy, colors::Yellow);
    }

    // copies `src` at half size (2x2 box average) to (ox, oy) in `dst`
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
}

int main() {
    Scene scene = buildScene();
    DoubleBuffer buffers(800, 500);
    Animator anim(scene, buffers);

    // 1. top mirror: keyframes with quadratic easing (each easing shapes the segment it starts)
    const double base = 3 * kPi / 4;
    anim.animateRotation("mirrorTop", Track{}
                                          .add(0.0, base, easeInOut)
                                          .add(1.2, base + 0.05, easeInOut)
                                          .add(2.6, base - 0.04, easeOut)
                                          .add(4.0, base));

    // 2. laser: slides right, then back (position track)
    anim.animatePosition("laser", Vec2Track{}
                                      .add(0.0, {125, 360}, easeOut)
                                      .add(2.0, {165, 360}, easeIn)
                                      .add(4.0, {125, 360}));

    // 3. lens: Bezier motion path (where) + Bezier easing (when), constant speed along the arc
    const curves::Bezier lensPath({{395, 210}, {405, 110}, {475, 110}, {490, 205}});
    const CubicBezierEasing lensEase = CubicBezierEasing::easeInOut();
    anim.animatePath("lens", MotionPath(lensPath, 0.0, kDuration, lensEase, true));

    // 4. prism: overshooting Bezier easing, then holds
    anim.animateRotation("prism", Track{}
                                      .add(0.0, 0.0)
                                      .add(0.8, 0.0, CubicBezierEasing::backOut())
                                      .add(2.8, 0.22));

    // overlays: the lens's path (dim) and the lens's easing graph with the current moment marked
    anim.addOverlay([&](Framebuffer& back, const FrameInfo&) {
        curves::drawCurve(back, lensPath, Color{70, 80, 110});
    });
    anim.addOverlay([&](Framebuffer& back, const FrameInfo& f) { drawEasingInset(back, lensEase, f.time / kDuration); });

    // run offline: save every front buffer, and keep six for the contact sheet
    const std::vector<int> sheetFrames{0, 12, 24, 36, 48, 60};
    std::vector<Framebuffer> kept;
    const int frames = anim.run(kDuration, kFps, [&](const Framebuffer& front, const FrameInfo& f) {
        char name[64];
        std::snprintf(name, sizeof name, "output/frames/anim_%04d.bmp", f.index);
        if (!front.saveBMP(name)) { std::printf("FAILED to write %s\n", name); return false; }
        for (int k : sheetFrames)
            if (k == f.index) kept.push_back(front);
        return true;
    });

    // contact sheet, 2 columns x 3 rows of half-size frames
    Framebuffer sheet(800, 750, colors::BenchDark);
    for (std::size_t i = 0; i < kept.size(); ++i)
        pasteHalf(kept[i], sheet, static_cast<int>(i % 2) * 400, static_cast<int>(i / 2) * 250);
    if (!sheet.saveBMP("output/animation_demo.bmp")) { std::puts("FAILED to write output/animation_demo.bmp"); return 1; }

    std::printf("wrote %d frames to output/frames/ (%.0f s at %.0f fps) and output/animation_demo.bmp\n",
                frames, kDuration, kFps);
    return frames == static_cast<int>(kDuration * kFps) + 1 ? 0 : 1;
}
