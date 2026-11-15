#include "Bench.h"

#include <cmath>

#include "optibench/Timeline.h"
#include "optibench/geometry/Lens.h"

namespace ob::app {
    namespace {
        constexpr double kMirrorTop = 3 * kPi / 4;
        constexpr double kBlueY0 = 500.0, kBlueY1 = 522.0;

        // the white lane's turning mirror and dispersing prism
        // (found by sweeping mirror angle, prism position and prism angle for the widest spectrum that
        //  stays on the screen, free of total internal reflection, over the notch's whole rotation range
        //  and with +-0.015 rad of slack in the prism angle)
        const Vec2 kWhiteMirror{620.0, 294.2};
        constexpr double kWhiteMirrorAngle = -0.8215;
        const Vec2 kPrism{667.5, 225.5};
        constexpr double kPrismSide = 90.0;
        constexpr double kPrismAngle = 1.338;

        // an equilateral triangle centred on its centroid, one side facing down
        Polygon equilateral(double side) {
            const double h = side * std::sqrt(3.0) / 2.0;
            return {{-side / 2, h / 3}, {side / 2, h / 3}, {0.0, -2 * h / 3}};
        }
    }

    const curves::Bezier& lensPath() {
        static const curves::Bezier path({{380, 130}, {420, 117}, {480, 117}, {530, 130}});
        return path;
    }

    Scene buildBench() {
        Scene scene(Layout::kBench);

        // red (top lane): a periscope lifts the beam over the bench, a thick BK7 lens gliding on a Bezier
        // arc focuses it, and a 45-45-90 SF11 fold prism turns it down by total internal reflection
        scene.emplace<LaserSource>("laserRed", Transform2D{.position = {110, 575}}, colors::Red);
        scene.emplace<FlatElement>("mirrorLow", Transform2D{.position = {250, 575}, .rotation = -kPi / 4}, 30.0,
                                   Material::mirror());
        scene.emplace<FlatElement>("mirrorTop", Transform2D{.position = {250, 130}, .rotation = kMirrorTop}, 30.0,
                                   Material::mirror());
        scene.emplace<ThickLensElement>("lens", Transform2D{.position = lensPath().evaluate(0.0)},
                                        geom::Lens(170.0, 170.0, 34.0, 52.0), Material::crownBK7());
        // SF11: n = 1.785, critical angle 34.1 degrees; the hypotenuse is met at about 45 degrees
        scene.emplace<PrismElement>("fold", Transform2D{.position = {780, 133}},
                                    Polygon{{-35.0, 35.0}, {-35.0, -35.0}, {35.0, 35.0}}, Material::denseFlintSF11());
        scene.emplace<FlatElement>("floor", Transform2D{.position = {772, 588}}, 110.0, Material::screen(), 8.0);

        // white (middle lane): a concave notched BK7 block, a mirror turns the beam up into an equilateral
        // SF11 prism near minimum deviation, which fans it into a spectrum across the right-hand screen
        scene.emplace<LaserSource>("lampWhite", Transform2D{.position = {110, 365}}, colors::White);
        scene.emplace<PrismElement>("notched", Transform2D{.position = {430, 380}},
                                    Polygon{{-55, -75}, {55, -75}, {55, 75}, {-55, 75}, {5, 0}}, Material::crownBK7());
        scene.emplace<FlatElement>("mirrorWhite", Transform2D{.position = kWhiteMirror, .rotation = kWhiteMirrorAngle}, 30.0,
                                   Material::mirror());
        scene.emplace<PrismElement>("prism", Transform2D{.position = kPrism, .rotation = kPrismAngle},
                                    equilateral(kPrismSide), Material::denseFlintSF11());
        scene.emplace<FlatElement>("screen", Transform2D{.position = {1206, 205}, .rotation = -kPi / 2}, 125.0,
                                   Material::screen(), 8.0);

        // blue (bottom lane): an expanded, collimated beam into a parabolic dish, converging on its focus
        scene.emplace<LaserSource>("laserBlue", Transform2D{.position = {110, kBlueY0}}, colors::Blue, 64.0, 50.0);
        scene.emplace<CurvedMirrorElement>("dish", Transform2D{.position = {1170, 470}},
                                           CurvedMirrorElement::parabola(160.0, 120.0));
        scene.emplace<FlatElement>("detector", Transform2D{.position = {975, 458}, .rotation = kPi / 2}, 16.0,
                                   Material::screen(), 8.0);
        return scene;
    }

    std::vector<SourceSpec> benchSources() {
        return {
            {"laserRed", SourceSpec::Kind::Laser, 650.0, 1, 0.0, "RED 650 nm"},
            {"lampWhite", SourceSpec::Kind::WhiteLight, 0.0, 61, 0.0, "WHITE 400-700 nm"},
            {"laserBlue", SourceSpec::Kind::Collimated, 450.0, 9, 36.0, "BLUE 450 nm"},
        };
    }

    std::vector<KeyframeTrack> animateBench(Animator& anim) {
        anim.animateRotation("mirrorTop", Track{}
                                              .add(0.0, kMirrorTop, easeInOut)
                                              .add(1.5, kMirrorTop + 0.02, easeInOut)
                                              .add(3.0, kMirrorTop - 0.02, easeOut)
                                              .add(4.0, kMirrorTop));
        anim.animatePath("lens", MotionPath(lensPath(), 0.0, kDuration, CubicBezierEasing::easeInOut(), true));
        anim.animateRotation("notched", Track{}
                                            .add(0.0, 0.0)
                                            .add(0.6, 0.0, CubicBezierEasing::backOut())
                                            .add(2.6, 0.16, easeInOut)
                                            .add(4.0, 0.0));
        anim.animatePosition("laserBlue", Vec2Track{}
                                              .add(0.0, {110, kBlueY0}, easeInOut)
                                              .add(2.0, {110, kBlueY1}, easeInOut)
                                              .add(4.0, {110, kBlueY0}));
        // the fold prism rocks by +-0.03 rad, sweeping the red spot along the floor; the beam stays past
        // the critical angle, so it is always totally internally reflected (every frame checks this)
        anim.animateRotation("fold", Track{}.add(0.0, -0.03, easeInOut).add(2.0, 0.03, easeInOut).add(4.0, -0.03));

        return {
            {"mirror", {0.0, 1.5, 3.0, 4.0}},
            {"lens path", {0.0, 4.0}},
            {"notch", {0.0, 0.6, 2.6, 4.0}},
            {"blue", {0.0, 2.0, 4.0}},
            {"fold", {0.0, 2.0, 4.0}},
        };
    }
}
