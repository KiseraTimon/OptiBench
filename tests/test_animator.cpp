// Tests for double buffering and the animation loop (Animator driving a Scene).
#include <cmath>
#include <cstdio>
#include <stdexcept>
#include <vector>

#include "optibench/Animator.h"
#include "optibench/DoubleBuffer.h"
#include "optibench/Scene.h"

#define CHECK(cond) do { if (!(cond)) { std::printf("FAIL %s:%d  %s\n", __FILE__, __LINE__, #cond); return 1; } } while (0)

using namespace ob;

static bool near(double a, double b, double eps = 1e-6) { return std::fabs(a - b) < eps; }
static bool near(Vec2 a, Vec2 b, double eps = 1e-6) { return near(a.x, b.x, eps) && near(a.y, b.y, eps); }

static bool sameColor(Color a, Color b) { return a.r == b.r && a.g == b.g && a.b == b.b; }

static Scene smallScene() {
    Scene scene(ClipRect{10, 10, 190, 110});
    scene.emplace<LaserSource>("laser", Transform2D{.position = {30, 60}}, Color{255, 0, 0});
    scene.emplace<FlatElement>("mirror", Transform2D{.position = {120, 60}, .rotation = 0.0}, 30.0, Material::mirror());
    return scene;
}

int main() {
    // ---------- double buffering ----------
    {
        DoubleBuffer db(4, 3, Color{0, 0, 0});
        CHECK(db.width() == 4 && db.height() == 3);
        CHECK(&db.back() != &db.front());

        // drawing goes to the back buffer only: the front is untouched until the swap
        db.back().setPixel(1, 1, Color{255, 255, 255});
        CHECK(sameColor(db.front().getPixel(1, 1), Color{0, 0, 0}));

        // swap flips roles without copying: the buffer drawn into is now the front one
        const Framebuffer* drawn = &db.back();
        const Framebuffer* shown = &db.front();
        db.swap();
        CHECK(&db.front() == drawn && &db.back() == shown);
        CHECK(sameColor(db.front().getPixel(1, 1), Color{255, 255, 255}));
        CHECK(db.swapCount() == 1);

        // swapping twice restores the original roles
        db.swap();
        CHECK(&db.front() == shown);
    }

    // ---------- animator: bindings write transforms, frames are presented from the front ----------
    {
        Scene scene = smallScene();
        DoubleBuffer db(200, 120);
        Animator anim(scene, db);

        anim.animateRotation("mirror", Track{}.add(0.0, 0.0).add(1.0, 0.5, easeInOut));
        anim.animatePosition("laser", Vec2Track{}.add(0.0, {30, 60}).add(1.0, {30, 40}));

        std::vector<FrameInfo> seen;
        bool frontOnly = true;
        const int n = anim.run(1.0, 10.0, [&](const Framebuffer& f, const FrameInfo& info) {
            frontOnly = frontOnly && (&f == &db.front());
            seen.push_back(info);
            return true;
        });

        CHECK(n == 11 && seen.size() == 11);                 // t = 0, 0.1, ..., 1.0
        CHECK(frontOnly);
        CHECK(seen.front().index == 0 && seen.front().time == 0.0 && seen.front().dt == 0.0);
        CHECK(seen.back().index == 10 && near(seen.back().time, 1.0, 1e-9));
        CHECK(near(seen[5].time, 0.5, 1e-9) && near(seen[5].dt, 0.1, 1e-12));
        CHECK(db.swapCount() == 11);

        CHECK(near(scene.find("mirror")->transform.rotation, 0.5));
        CHECK(near(scene.find("laser")->transform.position, {30, 40}));

        // the presented frame is a fully rendered scene (bench panel colour inside the viewport)
        CHECK(sameColor(db.front().getPixel(15, 105), scene.panelColor));
    }

    // ---------- the result depends on time, not on frame rate ----------
    {
        auto finalRotation = [](double fps) {
            Scene scene = smallScene();
            DoubleBuffer db(200, 120);
            Animator anim(scene, db);
            // the easing on the first keyframe shapes the 0 s -> 2 s segment
            anim.animateRotation("mirror", Track{}.add(0.0, 0.0, CubicBezierEasing::ease()).add(2.0, 1.0).add(3.0, 0.2));
            double atOneSecond = NAN;
            anim.run(1.5, fps, [&](const Framebuffer&, const FrameInfo& info) {
                if (near(info.time, 1.0, 1e-9)) atOneSecond = scene.find("mirror")->transform.rotation;
                return true;
            });
            return atOneSecond;
        };
        const double at10 = finalRotation(10.0), at30 = finalRotation(30.0);
        CHECK(!std::isnan(at10) && !std::isnan(at30));
        CHECK(near(at10, at30, 1e-9));
        CHECK(near(at10, CubicBezierEasing::ease()(0.5), 1e-9));
    }

    // ---------- motion path binding, orientation, overlays, stopping early ----------
    {
        Scene scene = smallScene();
        DoubleBuffer db(200, 120);
        Animator anim(scene, db);

        const curves::Bezier arc({{40, 90}, {40, 20}, {160, 20}, {160, 90}});
        anim.animatePath("mirror", MotionPath(arc, 0.0, 2.0, CubicBezierEasing::easeInOut()), true);

        int overlays = 0;
        bool onBack = true;
        anim.addOverlay([&](Framebuffer& back, const FrameInfo&) {
            onBack = onBack && (&back == &db.back());     // overlays draw before the swap
            ++overlays;
            back.setPixel(0, 0, Color{1, 2, 3});
        });

        const int n = anim.run(2.0, 4.0, [&](const Framebuffer&, const FrameInfo& info) { return info.index < 3; });
        CHECK(n == 4 && overlays == 4 && onBack);             // stops when present() returns false
        CHECK(sameColor(db.front().getPixel(0, 0), Color{1, 2, 3}));

        const MotionPath check(arc, 0.0, 2.0, CubicBezierEasing::easeInOut());
        CHECK(near(scene.find("mirror")->transform.position, check.sample(0.75), 1e-9));
        CHECK(near(scene.find("mirror")->transform.rotation, check.heading(0.75), 1e-9));

        // step() keeps going from where run() stopped
        const FrameInfo next = anim.step(0.25);
        CHECK(next.index == 4 && near(next.time, 1.0, 1e-9));
        CHECK(near(scene.find("mirror")->transform.position, check.sample(1.0), 1e-9));

        // custom updates run every frame with the current time
        double lastTime = -1.0;
        anim.addUpdate([&](double t) { lastTime = t; });
        anim.step(0.5);
        CHECK(near(lastTime, 1.5, 1e-9));

        // a paused clock freezes the animation but keeps drawing frames
        anim.clock().pause();
        const FrameInfo frozen = anim.step(0.5);
        CHECK(frozen.dt == 0.0 && near(frozen.time, 1.5, 1e-9));
    }

    // ---------- bad input ----------
    {
        Scene scene = smallScene();
        DoubleBuffer db(200, 120);
        Animator anim(scene, db);
        bool missing = false, empty = false, badFps = false;
        try { anim.animateRotation("nope", Track{}.add(0, 0)); } catch (const std::invalid_argument&) { missing = true; }
        try { anim.animateRotation("mirror", Track{}); } catch (const std::invalid_argument&) { empty = true; }
        try { anim.run(1.0, 0.0); } catch (const std::invalid_argument&) { badFps = true; }
        CHECK(missing && empty && badFps);
    }

    std::puts("test_animator: all checks passed");
    return 0;
}
