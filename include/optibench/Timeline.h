#pragma once

/*
 * Special Notes: Timeline (tweening, easing, keyframes, motion paths, time)
 *
 * The animation engine is built from small pieces, each a standard animation technique:
 *
 *   lerp()               linear interpolation:  a + (b - a) * u,  u in [0, 1]
 *   easeIn/Out/InOut()   easing: remap u so motion speeds up / slows down instead of moving
 *                        at a constant rate (quadratic: u^2, 1 - (1 - u)^2, and both halves)
 *   CubicBezierEasing    an easing curve shaped by a cubic Bezier through (0,0) and (1,1) with
 *                        two free control points, the same model as CSS cubic-bezier()
 *   BasicTrack<T>        keyframes (time, value, easing) sampled with tweening; Track animates
 *                        doubles (rotation), Vec2Track animates points (position, scale)
 *   MotionPath           moves a point ALONG a curve in scene space (Bezier, Catmull-Rom, ...),
 *                        optionally at constant speed by arc-length reparameterization
 *   Clock                delta-time: time advances by real seconds (t += dt), never by frame
 *                        count, so motion runs at the same speed at any frame rate
 *
 * Easing vs. motion path: an easing curve lives in a (time, progress) graph and only decides
 * WHEN the object is how far along; a motion path lives in the scene and decides WHERE it goes.
 * They combine: position = path(ease(u)).
 *
 * How animation plugs into the scene: each frame, look elements up by name and assign to
 * their `transform`, then call scene.render() (Animator in Animator.h does this for you):
 *
 *      scene.findAs<FlatElement>("mirrorTop")->transform.rotation = rotationTrack.sample(t);
 *
 * OWNER: Sharon
 */

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <functional>
#include <stdexcept>
#include <vector>

#include "Vec2.h"
#include "curves/Bezier.h"
#include "curves/Composite.h"

namespace ob {

    /*
     * Linear interpolation
     */
    // a + (b - a) * t; t = 0 gives a, t = 1 gives b (t outside [0, 1] extrapolates)
    double lerp(double a, double b, double t);
    Vec2 lerp(const Vec2& a, const Vec2& b, double t);

    /*
     * Easing functions: map progress u in [0, 1] to eased progress, with f(0) = 0 and f(1) = 1.
     * Inputs outside [0, 1] are clamped.
     */
    double linear(double t);        // u              constant speed
    double easeIn(double t);        // u^2            starts slow, speeds up
    double easeOut(double t);       // 1 - (1 - u)^2  starts fast, settles
    double easeInOut(double t);     // ease in for the first half, ease out for the second

    // any callable double(double) works as an easing curve
    using Easing = std::function<double(double)>;

    /**
     * Cubic Bezier easing curve (as covered in lectures, and as CSS cubic-bezier(x1, y1, x2, y2)).
     *
     * The curve has control points P0 = (0, 0), P1 = (x1, y1), P2 = (x2, y2), P3 = (1, 1),
     * where x is time progress and y is eased progress:
     *
     *      B(s) = (1-s)^3 P0 + 3(1-s)^2 s P1 + 3(1-s) s^2 P2 + s^3 P3,      s in [0, 1]
     *
     * To ease a time progress u we solve x(s) = u for the curve parameter s (Newton's method on
     * x(s) - u, with bisection as a fallback where x'(s) is near zero), then return y(s).
     * x1 and x2 must be in [0, 1] so that x(s) never decreases and the solution is unique.
     * y1 and y2 may leave [0, 1], which makes the motion overshoot (back-out) or anticipate.
     */
    class CubicBezierEasing {
    public:
        /**
         * @throws std::invalid_argument if a value is not finite or x1 / x2 is outside [0, 1]
         */
        CubicBezierEasing(double x1, double y1, double x2, double y2);

        // eased progress for time progress u (u clamped to [0, 1]; exactly 0 at 0 and 1 at 1)
        double operator()(double u) const;

        // the curve parameter s with x(s) = u (u clamped to [0, 1])
        double solveParam(double u) const;

        // the two free control points
        Vec2 p1() const { return curve_.controlPoint(1); }
        Vec2 p2() const { return curve_.controlPoint(2); }

        // the full cubic, P0..P3, in (time, progress) coordinates (e.g. for drawing the graph)
        const curves::Bezier& curve() const { return curve_; }

        // common presets (CSS values), plus an overshooting back-out
        static CubicBezierEasing ease()       { return {0.25, 0.10, 0.25, 1.00}; }
        static CubicBezierEasing easeIn()     { return {0.42, 0.00, 1.00, 1.00}; }
        static CubicBezierEasing easeOut()    { return {0.00, 0.00, 0.58, 1.00}; }
        static CubicBezierEasing easeInOut()  { return {0.42, 0.00, 0.58, 1.00}; }
        static CubicBezierEasing backOut()    { return {0.34, 1.56, 0.64, 1.00}; }

    private:
        curves::Bezier curve_;
    };

    /*
     * Keyframes and tracks
     */
    // a value at a moment in time; `ease` shapes the segment that STARTS at this keyframe
    template <class T>
    struct BasicKeyframe {
        double time = 0.0;      // seconds
        T value{};
        Easing ease = linear;
    };

    /**
     * A sequence of keyframes sampled with tweening:
     *
     *      for prev.time <= t < next.time:   u = (t - prev.time) / (next.time - prev.time)
     *                                        value = lerp(prev.value, next.value, prev.ease(u))
     *
     * Before the first keyframe / after the last one the value is clamped to that keyframe.
     * Keyframes are kept sorted by time. Two keyframes at the same time make an instant jump.
     */
    template <class T>
    class BasicTrack {
    public:
        using Key = BasicKeyframe<T>;

        /**
         * adds a keyframe; returns *this so calls can be chained
         * @throws std::invalid_argument if time is not finite or ease is empty
         */
        BasicTrack& add(double time, T value, Easing ease = linear) {
            if (!std::isfinite(time)) throw std::invalid_argument("BasicTrack::add: time must be finite");
            if (!ease) throw std::invalid_argument("BasicTrack::add: easing must not be empty");
            const auto at = std::upper_bound(keys_.begin(), keys_.end(), time,
                                             [](double t, const Key& k) { return t < k.time; });
            keys_.insert(at, Key{time, std::move(value), std::move(ease)});
            return *this;
        }

        // the tweened value at time t (T{} if the track has no keyframes)
        T sample(double t) const {
            if (keys_.empty()) return T{};
            if (t <= keys_.front().time) return keys_.front().value;
            if (t >= keys_.back().time) return keys_.back().value;

            // first keyframe strictly after t; the one before it starts the segment containing t
            const auto next = std::upper_bound(keys_.begin(), keys_.end(), t,
                                               [](double v, const Key& k) { return v < k.time; });
            const auto prev = next - 1;
            const double u = (t - prev->time) / (next->time - prev->time);   // span > 0 here
            return lerp(prev->value, next->value, prev->ease(u));
        }

        bool empty() const { return keys_.empty(); }
        std::size_t size() const { return keys_.size(); }
        const std::vector<Key>& keys() const { return keys_; }

        // time of the first / last keyframe (0 when empty)
        double startTime() const { return keys_.empty() ? 0.0 : keys_.front().time; }
        double endTime() const { return keys_.empty() ? 0.0 : keys_.back().time; }

        // time from the first to the last keyframe
        double duration() const { return endTime() - startTime(); }

    private:
        std::vector<Key> keys_;
    };

    using Keyframe = BasicKeyframe<double>;
    using Track = BasicTrack<double>;           // rotation, single numbers
    using Vec2Track = BasicTrack<Vec2>;         // position, scale

    /**
     * Motion along a curve in scene space.
     *
     * Between startTime and endTime, time progress u = (t - startTime) / (endTime - startTime) is
     * eased, then mapped onto the curve:
     *
     *      parametric speed:  curve parameter = ease(u) * segmentCount
     *                         (speed varies with control-point spacing)
     *      constant speed:    arc length travelled = ease(u) * length()
     *                         (arc-length reparameterization: equal times cover equal distances,
     *                          so only the easing changes the speed)
     *
     * Any curve from the curves module works: a single Bezier, a CompositeBezier, or a spline's
     * toBezier() (e.g. CatmullRom, which passes through every point you give it).
     * Eased progress outside [0, 1] (overshooting easings) is clamped to the path's ends.
     */
    class MotionPath {
    public:
        /**
         * @throws std::invalid_argument if the curve is empty, the times are not finite, endTime <= startTime,
         *                               or ease is empty
         */
        MotionPath(curves::CompositeBezier curve, double startTime, double endTime, Easing ease = linear,
                   bool constantSpeed = true);
        MotionPath(const curves::Bezier& curve, double startTime, double endTime, Easing ease = linear,
                   bool constantSpeed = true);

        // position on the curve at time t (clamped to the path's start / end outside the time range)
        Vec2 sample(double t) const;

        // direction of travel at time t, as an angle in radians (atan2 of the unit tangent)
        double heading(double t) const;

        // the curve's global parameter (0..segmentCount) reached at time t
        double paramAt(double t) const;

        // eased progress in [0, 1] at time t
        double progress(double t) const;

        const curves::CompositeBezier& curve() const { return curve_; }
        double length() const { return lengths_.empty() ? 0.0 : lengths_.back(); }
        double startTime() const { return t0_; }
        double endTime() const { return t1_; }
        bool constantSpeed() const { return constantSpeed_; }

    private:
        curves::CompositeBezier curve_;
        double t0_, t1_;
        Easing ease_;
        bool constantSpeed_;

        // cumulative arc length table: lengths_[i] is the length from 0 to params_[i]
        static constexpr std::size_t kSamplesPerSegment = 64;
        std::vector<double> params_;
        std::vector<double> lengths_;

        double paramAtLength(double s) const;
    };

    /*
     * Time
     */
    /**
     * Delta-time clock. Each frame, advance() by the time that passed (a fixed 1/fps when
     * rendering frames offline, or a measured RealTimeSource::lap() in a live window); animation
     * is then sampled at time(), never at a frame number.
     */
    class Clock {
    public:
        explicit Clock(double startTime = 0.0) : time_(startTime) {}

        /**
         * moves time forward by dt * speed (0 while paused); returns the step actually applied
         * @throws std::invalid_argument if dt is negative or not finite
         */
        double advance(double dt);

        double time() const { return time_; }       // seconds since the start
        double dt() const { return dt_; }           // the last step applied
        long frames() const { return frames_; }     // how many times advance() was called

        void reset(double time = 0.0) { time_ = time; dt_ = 0.0; frames_ = 0; }
        void pause() { paused_ = true; }
        void resume() { paused_ = false; }
        bool paused() const { return paused_; }

        // playback speed multiplier (1 = real time, 0.5 = slow motion); must be finite and >= 0
        void setSpeed(double speed);
        double speed() const { return speed_; }

    private:
        double time_;
        double dt_ = 0.0;
        double speed_ = 1.0;
        long frames_ = 0;
        bool paused_ = false;
    };

    // measures real elapsed time, for a live (windowed) loop: lap() returns seconds since the last lap()
    class RealTimeSource {
    public:
        RealTimeSource() : last_(std::chrono::steady_clock::now()) {}
        double lap();

    private:
        std::chrono::steady_clock::time_point last_;
    };

    // repeats time over [0, period): 0, ..., period, 0, ...   (t unchanged if period <= 0)
    double loopTime(double t, double period);

    // plays forward then backward: 0 -> period -> 0 -> ...    (t unchanged if period <= 0)
    double pingPongTime(double t, double period);
}
