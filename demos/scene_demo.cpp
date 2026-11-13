// Integrated scene demo: a laser, a two-mirror periscope, a lens, a prism and a screen,
// all described as OpticalElements and rendered by Scene::render().
//
// Writes:
//      output/scene_demo.bmp           one still frame
//      output/frames/scene_0000.bmp... a short swing of the top mirror (stand-in for the
//                                      animation system: it just edits a transform per frame)
#include <cmath>
#include <cstdio>

#include "optibench/Scene.h"
#include "optibench/Tracer.h"

using namespace ob;

namespace {
    const char* kindName(BeamEvent::Kind k) {
        switch (k) {
            case BeamEvent::Kind::Reflect: return "reflect";
            case BeamEvent::Kind::Refract: return "refract";
            case BeamEvent::Kind::TotalInternalReflection: return "TIR";
            case BeamEvent::Kind::Absorb: return "absorb";
        }
        return "?";
    }

    Scene buildScene() {
        Scene scene(ClipRect{40, 40, 760, 460});

        // laser at the bottom-left, firing right
        scene.emplace<LaserSource>("laser", Transform2D{.position = {125, 360}}, Color{255, 70, 70});

        // periscope: both mirrors are "/" shaped, turned 45 degrees.
        // Mirror 1 turns the rightward beam upward; mirror 2 turns it back to the right.
        scene.emplace<FlatElement>("mirrorLow", Transform2D{.position = {260, 360}, .rotation = -kPi / 4},
                                   38.0, Material::mirror());
        scene.emplace<FlatElement>("mirrorTop", Transform2D{.position = {260, 150}, .rotation = 3 * kPi / 4},
                                   38.0, Material::mirror());

        // lens a little below the beam, so the beam is refracted off-axis
        scene.emplace<LensElement>("lens", Transform2D{.position = {430, 180}}, 62.0);

        // prism: a narrow apex lets the beam out; a wide one (e.g. triangle(120, 110)) totally internally reflects it
        scene.emplace<PrismElement>("prism", Transform2D{.position = {590, 210}},
                                    PrismElement::triangle(70.0, 120.0));

        // screen on the right: body on its right-hand side
        scene.emplace<FlatElement>("screen", Transform2D{.position = {735, 265}, .rotation = -kPi / 2},
                                   170.0, Material::screen(), 8.0);
        return scene;
    }
}

int main() {
    Scene scene = buildScene();
    Framebuffer fb(800, 500);

    // ---- still frame + a printed report of what the beam did ----
    scene.render(fb);
    if (!fb.saveBMP("output/scene_demo.bmp")) { std::puts("FAILED to write output/scene_demo.bmp"); return 1; }

    const BeamPath path = traceBeam(scene, scene.sources().front()->emissionRay());
    std::puts("beam events:");
    for (const BeamEvent& e : path.events) {
        std::printf("  %-8s at %-10s (%6.1f, %6.1f)  incidence %5.1f deg", kindName(e.kind), e.element.c_str(),
                    e.point.x, e.point.y, e.incidenceDeg);
        if (e.kind == BeamEvent::Kind::Refract)
            std::printf("  -> %5.1f deg   [n1 sin = %.4f, n2 sin = %.4f]", e.refractionDeg,
                        e.n1 * std::sin(e.incidenceDeg * kPi / 180.0),
                        e.n2 * std::sin(e.refractionDeg * kPi / 180.0));
        std::puts("");
    }
    std::printf("beam %s\n", path.hitScreen ? "landed on the screen" : "missed the screen");

    // ---- tiny animation: swing the top mirror and write numbered frames ----
    constexpr int frames = 12;
    for (int i = 0; i < frames; ++i) {
        const double t = static_cast<double>(i) / frames;
        scene.findAs<FlatElement>("mirrorTop")->transform.rotation = 3 * kPi / 4 + 0.05 * std::sin(2 * kPi * t);
        scene.render(fb);

        char name[64];
        std::snprintf(name, sizeof name, "output/frames/scene_%04d.bmp", i);
        if (!fb.saveBMP(name)) { std::printf("FAILED to write %s\n", name); return 1; }
    }
    std::printf("wrote output/scene_demo.bmp and %d frames in output/frames/\n", frames);
    return 0;
}
