// Tests for the timeline: lerp, easing (quadratic and cubic Bezier), keyframe tracks,
// motion paths along curves, and the delta-time clock.
#include <cmath>
#include <cstdio>
#include <stdexcept>

#include "optibench/Timeline.h"
#include "optibench/curves/CatmullRom.h"

#define CHECK(cond) do { if (!(cond)) { std::printf("FAIL %s:%d  %s\n", __FILE__, __LINE__, #cond); return 1; } } while (0)

using namespace ob;

static bool near(double a, double b, double eps = 1e-6) { return std::fabs(a - b) < eps; }
static bool near(Vec2 a, Vec2 b, double eps = 1e-6) { return near(a.x, b.x, eps) && near(a.y, b.y, eps); }

template <class F>
static bool throws(F f) {
    try { f(); } catch (const std::invalid_argument&) { return true; }
    return false;
}

// f(0) = 0, f(1) = 1 exactly and f never decreases on [0, 1]
template <class F>
static bool isEasing(F f) {
    if (f(0.0) != 0.0 || f(1.0) != 1.0) return false;
    double prev = 0.0;
    for (int i = 1; i <= 200; ++i) {
        const double v = f(i / 200.0);
        if (v < prev - 1e-12) return false;
        prev = v;
    }
    return true;
}

int main() {
    // ---------- lerp ----------
    {
        CHECK(lerp(2.0, 10.0, 0.0) == 2.0);
        CHECK(lerp(2.0, 10.0, 1.0) == 10.0);
        CHECK(near(lerp(2.0, 10.0, 0.25), 4.0));
        CHECK(near(lerp(Vec2{0, 0}, Vec2{10, -20}, 0.5), {5, -10}));
        CHECK(near(lerp(0.0, 10.0, 1.5), 15.0));      // extrapolates outside [0, 1]
    }

    // ---------- quadratic easing ----------
    {
        CHECK(isEasing(linear));
        CHECK(isEasing(easeIn));
        CHECK(isEasing(easeOut));
        CHECK(isEasing(easeInOut));

        CHECK(near(easeIn(0.5), 0.25));         // slow start: behind linear
        CHECK(near(easeOut(0.5), 0.75));        // fast start: ahead of linear
        CHECK(near(easeInOut(0.5), 0.5));       // symmetric
        CHECK(near(easeInOut(0.25), 0.125));
        CHECK(near(easeInOut(0.25) + easeInOut(0.75), 1.0));

        // inputs outside [0, 1] are clamped
        CHECK(easeIn(-1.0) == 0.0 && easeOut(2.0) == 1.0);
    }

    // ---------- cubic Bezier easing ----------
    {
        // control points on the diagonal give a straight line: exactly linear
        const CubicBezierEasing straight(1.0 / 3.0, 1.0 / 3.0, 2.0 / 3.0, 2.0 / 3.0);
        for (int i = 0; i <= 20; ++i) CHECK(near(straight(i / 20.0), i / 20.0, 1e-7));

        // every preset is a valid easing, and solving x(s) = u really inverts x
        for (const CubicBezierEasing& e : {CubicBezierEasing::ease(), CubicBezierEasing::easeIn(),
                                           CubicBezierEasing::easeOut(), CubicBezierEasing::easeInOut()}) {
            CHECK(isEasing(e));
            for (int i = 0; i <= 50; ++i) {
                const double u = i / 50.0;
                CHECK(near(e.curve().evaluate(e.solveParam(u)).x, u, 1e-7));
            }
        }

        // ease-in lags linear, ease-out leads it, ease-in-out is symmetric about the centre
        CHECK(CubicBezierEasing::easeIn()(0.3) < 0.3);
        CHECK(CubicBezierEasing::easeOut()(0.3) > 0.3);
        const CubicBezierEasing io = CubicBezierEasing::easeInOut();
        CHECK(near(io(0.5), 0.5, 1e-7));
        CHECK(near(io(0.2) + io(0.8), 1.0, 1e-7));

        // known value: CSS "ease" at 50% time is about 80.2% progress
        CHECK(near(CubicBezierEasing::ease()(0.5), 0.8024, 1e-3));

        // overshoot: y1 > 1 goes past the target before settling, still ending exactly at 1
        const CubicBezierEasing back = CubicBezierEasing::backOut();
        double peak = 0.0;
        for (int i = 0; i <= 100; ++i) peak = std::max(peak, back(i / 100.0));
        CHECK(peak > 1.05);
        CHECK(back(1.0) == 1.0);

        // a flat x'(s) at the ends (x1 = 0, x2 = 1) still solves through the bisection fallback
        const CubicBezierEasing flat(0.0, 0.0, 1.0, 1.0);
        CHECK(isEasing(flat));
        CHECK(near(flat.curve().evaluate(flat.solveParam(0.01)).x, 0.01, 1e-7));

        // x outside [0, 1] would make x(s) go backwards: rejected
        CHECK(throws([] { CubicBezierEasing(-0.1, 0, 0.5, 1); }));
        CHECK(throws([] { CubicBezierEasing(0.5, 0, 1.2, 1); }));
        CHECK(throws([] { CubicBezierEasing(0.5, NAN, 0.5, 1); }));

        // usable anywhere an Easing is
        const Easing asFunction = CubicBezierEasing::easeOut();
        CHECK(near(asFunction(0.3), CubicBezierEasing::easeOut()(0.3)));
    }

    // ---------- tracks ----------
    {
        Track t;
        CHECK(t.empty() && t.sample(1.0) == 0.0);

        // keyframes added out of order are sorted
        t.add(2.0, 10.0).add(0.0, 0.0).add(4.0, 0.0, easeIn);
        CHECK(t.size() == 3);
        CHECK(t.keys()[0].time == 0.0 && t.keys()[1].time == 2.0 && t.keys()[2].time == 4.0);
        CHECK(t.startTime() == 0.0 && t.endTime() == 4.0 && t.duration() == 4.0);

        // exact at keyframes, clamped outside
        CHECK(t.sample(0.0) == 0.0 && t.sample(2.0) == 10.0 && t.sample(4.0) == 0.0);
        CHECK(t.sample(-5.0) == 0.0 && t.sample(99.0) == 0.0);

        // first segment linear, second eased: the easing belongs to the keyframe that starts the segment
        CHECK(near(t.sample(1.0), 5.0));
        t.add(2.0, 10.0, easeIn);   // does nothing to sample(1): segment 0..2 starts at keyframe 0
        CHECK(near(t.sample(1.0), 5.0));

        Track e;
        e.add(0.0, 0.0, easeIn).add(1.0, 100.0);
        CHECK(near(e.sample(0.5), 25.0));

        // a Bezier easing on a keyframe
        Track b;
        b.add(0.0, 0.0, CubicBezierEasing::easeInOut()).add(2.0, 8.0);
        CHECK(near(b.sample(1.0), 4.0, 1e-6));
        CHECK(b.sample(0.4) < lerp(0.0, 8.0, 0.2));

        // two keyframes at the same time make an instant jump
        Track j;
        j.add(0.0, 0.0).add(1.0, 1.0).add(1.0, 5.0).add(2.0, 5.0);
        CHECK(near(j.sample(0.999), 0.999, 1e-9));
        CHECK(j.sample(1.0) == 5.0 && j.sample(1.5) == 5.0);

        // Vec2 positions
        Vec2Track p;
        p.add(0.0, {0, 0}).add(1.0, {10, 20}, easeOut).add(3.0, {10, 20});
        CHECK(near(p.sample(0.5), {5, 10}));
        CHECK(near(p.sample(2.0), {10, 20}));

        CHECK(throws([] { Track().add(NAN, 1.0); }));
        CHECK(throws([] { Track().add(0.0, 1.0, Easing{}); }));
    }

    // ---------- motion paths ----------
    {
        // a straight Bezier: constant speed covers equal distances in equal times
        const curves::Bezier straight({{0, 0}, {10, 0}, {90, 0}, {100, 0}});   // bunched-up parameter
        const MotionPath even(straight, 0.0, 10.0, linear, true);
        const MotionPath raw(straight, 0.0, 10.0, linear, false);
        CHECK(near(even.length(), 100.0, 1e-6));
        CHECK(near(even.sample(0.0), {0, 0}) && near(even.sample(10.0), {100, 0}));
        for (int i = 0; i <= 10; ++i) CHECK(near(even.sample(i), {10.0 * i, 0}, 1e-3));
        CHECK(!near(raw.sample(2.0).x, 20.0, 1.0));     // parametric speed is uneven for these control points
        CHECK(near(raw.sample(5.0), straight.evaluate(0.5)));

        // a curved path: points really are on the curve, ends match, clamped outside the time range
        const curves::Bezier arc({{0, 0}, {0, 100}, {100, 100}, {100, 0}});
        const MotionPath path(arc, 1.0, 3.0, easeInOut);
        CHECK(near(path.sample(0.0), {0, 0}) && near(path.sample(1.0), {0, 0}));
        CHECK(near(path.sample(3.0), {100, 0}) && near(path.sample(9.0), {100, 0}));
        CHECK(near(path.sample(2.0), arc.evaluate(0.5), 1e-3));        // symmetric arc + symmetric easing
        const Vec2 mid = path.sample(1.7);
        CHECK(near(mid, arc.evaluate(arc.closestParam(mid)), 1e-6));
        CHECK(path.sample(2.0).y > 50.0);                               // off the straight line between the ends

        // constant speed: distance travelled grows linearly with time
        const MotionPath steady(arc, 0.0, 1.0);
        const double quarter = arc.arcLength(0.0, steady.paramAt(0.25));
        CHECK(near(quarter, 0.25 * steady.length(), 1e-2));

        // heading follows the tangent: at the top of the arc, travel is along +x
        CHECK(near(path.heading(2.0), 0.0, 1e-3));
        CHECK(near(path.heading(1.0), std::atan2(1.0, 0.0), 1e-6));     // leaves (0,0) heading towards +y

        // a Catmull-Rom spline passes through every point; a path along it reaches each in turn
        const curves::CatmullRom spline({{0, 0}, {50, 40}, {100, 0}, {150, 40}});
        const MotionPath along(spline.toBezier(), 0.0, 3.0, linear, false);
        CHECK(near(along.sample(1.0), {50, 40}, 1e-6));
        CHECK(near(along.sample(2.0), {100, 0}, 1e-6));
        CHECK(near(along.sample(3.0), {150, 40}, 1e-6));

        // overshooting easings stay on the path
        const MotionPath bouncy(arc, 0.0, 1.0, CubicBezierEasing::backOut());
        for (int i = 0; i <= 20; ++i) {
            const double pr = bouncy.progress(i / 20.0);
            CHECK(pr >= 0.0 && pr <= 1.0);
        }

        CHECK(throws([] { MotionPath(curves::CompositeBezier{}, 0.0, 1.0); }));
        CHECK(throws([&] { MotionPath(arc, 2.0, 2.0); }));
    }

    // ---------- clock: delta time ----------
    {
        Clock c;
        CHECK(c.time() == 0.0);
        c.advance(0.5);
        c.advance(0.25);
        CHECK(near(c.time(), 0.75) && near(c.dt(), 0.25) && c.frames() == 2);

        // same total time at different frame rates -> the same animation value
        Track rot;
        rot.add(0.0, 0.0).add(2.0, 1.0, easeOut).add(3.0, 3.0);
        Clock slow, fast;
        for (int i = 0; i < 20; ++i) slow.advance(1.0 / 10.0);        // 10 fps for 2 s
        for (int i = 0; i < 120; ++i) fast.advance(1.0 / 60.0);       // 60 fps for 2 s
        CHECK(near(slow.time(), 2.0, 1e-9) && near(fast.time(), 2.0, 1e-9));
        CHECK(near(rot.sample(slow.time()), rot.sample(fast.time()), 1e-9));

        // pause and speed
        c.pause();
        CHECK(c.advance(1.0) == 0.0 && near(c.time(), 0.75));
        c.resume();
        c.setSpeed(0.5);
        CHECK(near(c.advance(1.0), 0.5) && near(c.time(), 1.25));
        c.reset();
        CHECK(c.time() == 0.0 && c.frames() == 0);

        CHECK(throws([&] { c.advance(-0.1); }));
        CHECK(throws([&] { c.setSpeed(-1.0); }));

        RealTimeSource rt;
        CHECK(rt.lap() >= 0.0);

        // looping helpers
        CHECK(near(loopTime(5.5, 2.0), 1.5) && near(loopTime(-0.5, 2.0), 1.5));
        CHECK(near(pingPongTime(1.5, 2.0), 1.5) && near(pingPongTime(2.5, 2.0), 1.5));
        CHECK(near(pingPongTime(4.0, 2.0), 0.0));
    }

    std::puts("test_timeline: all checks passed");
    return 0;
}
