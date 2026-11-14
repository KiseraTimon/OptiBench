#include "optibench/Timeline.h"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <utility>

namespace ob {
    namespace {
        double clamp01(double t) { return std::clamp(t, 0.0, 1.0); }
    }

    /*
     * Linear interpolation
     */
    double lerp(double a, double b, double t) { return a + (b - a) * t; }

    Vec2 lerp(const Vec2& a, const Vec2& b, double t) { return a + (b - a) * t; }

    /*
     * Easing functions
     */
    double linear(double t) { return clamp01(t); }

    double easeIn(double t) {
        t = clamp01(t);
        return t * t;
    }

    double easeOut(double t) {
        t = clamp01(t);
        const double r = 1.0 - t;
        return 1.0 - r * r;
    }

    double easeInOut(double t) {
        // two quadratic halves meeting at (0.5, 0.5) with matching slope (2), so speed is continuous
        t = clamp01(t);
        if (t < 0.5) return 2.0 * t * t;
        const double r = 1.0 - t;
        return 1.0 - 2.0 * r * r;
    }

    /*
     * Cubic Bezier easing
     */
    CubicBezierEasing::CubicBezierEasing(double x1, double y1, double x2, double y2)
        : curve_({Vec2{0.0, 0.0}, Vec2{x1, y1}, Vec2{x2, y2}, Vec2{1.0, 1.0}}) {
        if (!std::isfinite(x1) || !std::isfinite(y1) || !std::isfinite(x2) || !std::isfinite(y2))
            throw std::invalid_argument("CubicBezierEasing: control points must be finite");
        if (x1 < 0.0 || x1 > 1.0 || x2 < 0.0 || x2 > 1.0)
            throw std::invalid_argument("CubicBezierEasing: x1 and x2 must be in [0, 1]");
    }

    double CubicBezierEasing::solveParam(double u) const {
        u = clamp01(u);
        if (u <= 0.0) return 0.0;
        if (u >= 1.0) return 1.0;

        constexpr double tol = 1e-9;

        // Newton's method on f(s) = x(s) - u, starting from s = u (exact for a linear x(s))
        double s = u;
        for (int i = 0; i < 8; ++i) {
            const double f = curve_.evaluate(s).x - u;
            if (std::fabs(f) < tol) return s;
            const double df = curve_.derivative(s).x;
            if (std::fabs(df) < 1e-6) break;            // flat spot: Newton would jump away
            const double next = s - f / df;
            if (next < 0.0 || next > 1.0) break;        // left the curve: fall back
            s = next;
        }

        // bisection: x(s) is non-decreasing on [0, 1] because x1, x2 are in [0, 1]
        double lo = 0.0, hi = 1.0;
        s = u;
        for (int i = 0; i < 60; ++i) {
            const double x = curve_.evaluate(s).x;
            if (std::fabs(x - u) < tol) break;
            (x < u ? lo : hi) = s;
            s = 0.5 * (lo + hi);
        }
        return s;
    }

    double CubicBezierEasing::operator()(double u) const {
        u = clamp01(u);
        if (u <= 0.0) return 0.0;
        if (u >= 1.0) return 1.0;
        return curve_.evaluate(solveParam(u)).y;
    }

    /*
     * Motion paths
     */
    MotionPath::MotionPath(curves::CompositeBezier curve, double startTime, double endTime, Easing ease,
                           bool constantSpeed)
        : curve_(std::move(curve)), t0_(startTime), t1_(endTime), ease_(std::move(ease)),
          constantSpeed_(constantSpeed) {
        if (curve_.empty()) throw std::invalid_argument("MotionPath: curve has no segments");
        if (!std::isfinite(t0_) || !std::isfinite(t1_) || t1_ <= t0_)
            throw std::invalid_argument("MotionPath: need finite times with endTime > startTime");
        if (!ease_) throw std::invalid_argument("MotionPath: easing must not be empty");

        // cumulative arc length, sampled finely per segment with the curve's own quadrature
        params_.push_back(0.0);
        lengths_.push_back(0.0);
        double before = 0.0;    // length of all earlier segments
        for (std::size_t i = 0; i < curve_.segmentCount(); ++i) {
            const curves::Bezier& seg = curve_.segment(i);
            for (std::size_t k = 1; k <= kSamplesPerSegment; ++k) {
                const double local = static_cast<double>(k) / kSamplesPerSegment;
                params_.push_back(static_cast<double>(i) + local);
                lengths_.push_back(before + seg.arcLength(0.0, local));
            }
            before = lengths_.back();
        }
    }

    MotionPath::MotionPath(const curves::Bezier& curve, double startTime, double endTime, Easing ease,
                           bool constantSpeed)
        : MotionPath(
              [&] {
                  curves::CompositeBezier c;
                  c.append(curve);
                  return c;
              }(),
              startTime, endTime, std::move(ease), constantSpeed) {}

    double MotionPath::progress(double t) const {
        const double u = clamp01((t - t0_) / (t1_ - t0_));
        return clamp01(ease_(u));
    }

    double MotionPath::paramAtLength(double s) const {
        const double total = length();
        if (total <= 0.0) return 0.0;
        s = std::clamp(s, 0.0, total);

        // binary search the table, then interpolate linearly inside the bracketing interval
        const auto it = std::lower_bound(lengths_.begin(), lengths_.end(), s);
        if (it == lengths_.begin()) return params_.front();
        if (it == lengths_.end()) return params_.back();
        const std::size_t i = static_cast<std::size_t>(it - lengths_.begin());
        const double span = lengths_[i] - lengths_[i - 1];
        const double f = span > 0.0 ? (s - lengths_[i - 1]) / span : 0.0;
        const double guess = lerp(params_[i - 1], params_[i], f);

        // refine with Newton's method on L(u) - s inside the bracket, where L'(u) = |C'(u)| (the speed)
        const double lo = params_[i - 1], hi = params_[i];
        const std::size_t seg = std::min(static_cast<std::size_t>(lo), curve_.segmentCount() - 1);
        const curves::Bezier& b = curve_.segment(seg);
        const double before = lengths_[seg * kSamplesPerSegment];   // length of all earlier segments
        double u = guess;
        for (int k = 0; k < 3; ++k) {
            const double local = u - static_cast<double>(seg);
            const double err = before + b.arcLength(0.0, local) - s;
            const double speed = b.derivative(local).length();
            if (std::fabs(err) < 1e-10 || speed <= 1e-12) break;
            u = std::clamp(u - err / speed, lo, hi);
        }
        return u;
    }

    double MotionPath::paramAt(double t) const {
        const double e = progress(t);
        if (constantSpeed_) return paramAtLength(e * length());
        return e * curve_.paramMax();
    }

    Vec2 MotionPath::sample(double t) const { return curve_.evaluate(paramAt(t)); }

    double MotionPath::heading(double t) const {
        const Vec2 d = curve_.tangent(paramAt(t));
        return (d.x == 0.0 && d.y == 0.0) ? 0.0 : std::atan2(d.y, d.x);
    }

    /*
     * Time
     */
    double Clock::advance(double dt) {
        if (!std::isfinite(dt) || dt < 0.0) throw std::invalid_argument("Clock::advance: dt must be finite and >= 0");
        dt_ = paused_ ? 0.0 : dt * speed_;
        time_ += dt_;
        ++frames_;
        return dt_;
    }

    void Clock::setSpeed(double speed) {
        if (!std::isfinite(speed) || speed < 0.0) throw std::invalid_argument("Clock::setSpeed: speed must be finite and >= 0");
        speed_ = speed;
    }

    double RealTimeSource::lap() {
        const auto now = std::chrono::steady_clock::now();
        const double dt = std::chrono::duration<double>(now - last_).count();
        last_ = now;
        return dt;
    }

    double loopTime(double t, double period) {
        if (period <= 0.0) return t;
        const double r = std::fmod(t, period);
        return r < 0.0 ? r + period : r;
    }

    double pingPongTime(double t, double period) {
        if (period <= 0.0) return t;
        const double r = loopTime(t, 2.0 * period);
        return r <= period ? r : 2.0 * period - r;
    }
}
