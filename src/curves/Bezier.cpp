#include "optibench/curves/Bezier.h"

#include <stdexcept>

#include "optibench/Line.h"

namespace ob::curves {
    namespace {
        // deepest recursion used by adaptive flattening (at most 2^18 segments per curve)
        constexpr int kMaxFlattenDepth = 18;

        // smallest flattening tolerance accepted, relative to the curve size
        constexpr double kMinRelativeTolerance = 1e-9;

        // true if every interior control point is within tol of the chord P0-Pn
        bool isFlat(std::span<const Vec2> p, double tol) {
            const Segment chord{p.front(), p.back()};
            for (std::size_t i = 1; i + 1 < p.size(); ++i)
                if (distanceToSegment(p[i], chord) > tol) return false;
            return true;
        }

        void flattenRecursive(const Bezier& c, double tol, int depth, Polyline& out) {
            if (depth >= kMaxFlattenDepth || isFlat(c.controlPoints(), tol)) {
                out.push_back(c.back());
                return;
            }
            const auto [left, right] = c.split(0.5);
            flattenRecursive(left, tol, depth + 1, out);
            flattenRecursive(right, tol, depth + 1, out);
        }
    }

    Bezier::Bezier(std::vector<Vec2> controlPoints) : points_(std::move(controlPoints)) {
        if (points_.empty()) throw std::invalid_argument("Bezier: at least one control point is required");
    }

    double Bezier::scale() const {
        const Vec2 s = controlBounds().size();
        return std::max({1.0, s.x, s.y});
    }

    Vec2 Bezier::evaluate(double t) const {
        const double u = 1.0 - t;
        switch (points_.size()) {
            case 1: return points_[0];
            case 2: return points_[0] * u + points_[1] * t;
            case 3: return points_[0] * (u * u) + points_[1] * (2.0 * u * t) + points_[2] * (t * t);
            case 4:
                return points_[0] * (u * u * u) + points_[1] * (3.0 * u * u * t) + points_[2] * (3.0 * u * t * t) +
                       points_[3] * (t * t * t);
            default: return evaluateDeCasteljau(t);
        }
    }

    Vec2 Bezier::evaluateDeCasteljau(double t) const {
        std::vector<Vec2> work = points_;
        for (std::size_t r = work.size() - 1; r > 0; --r)
            for (std::size_t i = 0; i < r; ++i) work[i] = lerp(work[i], work[i + 1], t);
        return work[0];
    }

    Vec2 Bezier::evaluateBernstein(double t) const {
        const int n = degree();
        Vec2 sum;
        for (int i = 0; i <= n; ++i) sum = sum + points_[static_cast<std::size_t>(i)] * bernstein(n, i, t);
        return sum;
    }

    Bezier Bezier::hodograph() const {
        if (points_.size() < 2) return Bezier({Vec2{}});
        const double n = static_cast<double>(degree());
        std::vector<Vec2> d(points_.size() - 1);
        for (std::size_t i = 0; i < d.size(); ++i) d[i] = (points_[i + 1] - points_[i]) * n;
        return Bezier(std::move(d));
    }

    Vec2 Bezier::derivative(double t, int order) const {
        if (order <= 0) return evaluate(t);
        if (order > degree()) return {};
        Bezier d = hodograph();
        for (int k = 1; k < order; ++k) d = d.hodograph();
        return d.evaluate(t);
    }

    Vec2 Bezier::tangent(double t) const {
        const double eps = 1e-12 * scale();
        Bezier d = hodograph();
        for (int k = 1; k <= degree(); ++k) {
            const Vec2 v = d.evaluate(t);
            if (v.length() > eps) {
                // B'(t + h) ~ h^(k-1) / (k-1)! * B^(k)(t): approaching t = 1 from the left flips odd powers
                const bool flip = t >= 1.0 && (k - 1) % 2 == 1;
                return flip ? -v.normalized() : v.normalized();
            }
            d = d.hodograph();
        }
        return {};
    }

    Vec2 Bezier::normal(double t) const { return tangent(t).perp(); }

    double Bezier::curvature(double t) const {
        const Vec2 d1 = derivative(t, 1);
        const double speed = d1.length();
        if (speed <= 1e-12 * scale()) return 0.0;
        const Vec2 d2 = derivative(t, 2);
        return d1.cross(d2) / (speed * speed * speed);
    }

    std::pair<Bezier, Bezier> Bezier::split(double t) const {
        const std::size_t n = points_.size();
        std::vector<Vec2> work = points_;
        std::vector<Vec2> left(n), right(n);
        for (std::size_t r = 0; r < n; ++r) {
            left[r] = work[0];
            right[n - 1 - r] = work[n - 1 - r];
            for (std::size_t i = 0; i + 1 < n - r; ++i) work[i] = lerp(work[i], work[i + 1], t);
        }
        return {Bezier(std::move(left)), Bezier(std::move(right))};
    }

    Bezier Bezier::subCurve(double t0, double t1) const {
        if (t0 > t1) std::swap(t0, t1);
        if (t1 <= 0.0 || t1 - t0 <= 0.0) return Bezier(std::vector<Vec2>(points_.size(), evaluate(t0)));
        const Bezier head = split(t1).first;
        return head.split(t0 / t1).second;
    }

    Bezier Bezier::elevated() const {
        const std::size_t n = points_.size() - 1; // current degree
        std::vector<Vec2> q(n + 2);
        q.front() = points_.front();
        q.back() = points_.back();
        for (std::size_t i = 1; i <= n; ++i) {
            const double a = static_cast<double>(i) / static_cast<double>(n + 1);
            q[i] = points_[i - 1] * a + points_[i] * (1.0 - a);
        }
        return Bezier(std::move(q));
    }

    Polyline Bezier::flatten(double tolerance) const {
        const double minTol = kMinRelativeTolerance * scale();
        if (!(tolerance > minTol)) tolerance = minTol; // also catches NaN
        Polyline out{points_.front()};
        if (points_.size() == 1) return out;
        flattenRecursive(*this, tolerance, 0, out);
        return out;
    }

    Polyline Bezier::flattenUniform(int segments) const {
        segments = std::max(segments, 1);
        Polyline out;
        out.reserve(static_cast<std::size_t>(segments) + 1);
        for (int i = 0; i <= segments; ++i) out.push_back(evaluate(static_cast<double>(i) / segments));
        return out;
    }

    Aabb Bezier::bounds() const {
        Aabb box;
        box.expand(points_.front());
        box.expand(points_.back());
        if (points_.size() < 3) return box;

        const Bezier d = hodograph();
        const double zeroTol = 1e-12 * scale();
        std::vector<double> xs, ys;
        for (const Vec2& p : d.controlPoints()) {
            xs.push_back(p.x);
            ys.push_back(p.y);
        }
        for (const auto* coeffs : {&xs, &ys})
            for (double t : bernsteinRoots(*coeffs, zeroTol)) box.expand(evaluate(t));
        return box;
    }

    double Bezier::arcLength(double t0, double t1) const {
        if (t0 > t1) std::swap(t0, t1);
        if (points_.size() == 2) return (points_[1] - points_[0]).length() * (t1 - t0);
        const Bezier d = hodograph();
        return integrate([&d](double t) { return d.evaluate(t).length(); }, t0, t1, 1e-12 * scale());
    }

    ArcLengthTable Bezier::arcLengthTable(int intervals) const {
        return ArcLengthTable([d = hodograph()](double t) { return d.evaluate(t).length(); }, 0.0, 1.0, intervals);
    }

    double Bezier::paramAtArcLength(double s) const { return arcLengthTable(8).paramAt(s); }

    Polyline Bezier::sampleByArcLength(int segments) const {
        segments = std::max(segments, 1);
        const ArcLengthTable table = arcLengthTable(std::max(8, segments));
        const double total = table.totalLength();
        Polyline out;
        out.reserve(static_cast<std::size_t>(segments) + 1);
        for (int i = 0; i <= segments; ++i) out.push_back(evaluate(table.paramAt(total * i / segments)));
        return out;
    }

    double Bezier::closestParam(Vec2 p) const {
        // coarse search over uniform samples, then Newton on f(t) = (B(t) - p) . B'(t)
        constexpr int kSamples = 64;
        double best = 0.0;
        double bestDist = (evaluate(0.0) - p).length();
        for (int i = 1; i <= kSamples; ++i) {
            const double t = static_cast<double>(i) / kSamples;
            const double dist = (evaluate(t) - p).length();
            if (dist < bestDist) { best = t; bestDist = dist; }
        }
        const Bezier d1 = hodograph();
        const Bezier d2 = d1.hodograph();
        double t = best;
        for (int iter = 0; iter < 16; ++iter) {
            const Vec2 r = evaluate(t) - p;
            const Vec2 v = d1.evaluate(t);
            const double f = r.dot(v);
            const double df = v.dot(v) + r.dot(d2.evaluate(t));
            if (std::fabs(df) < 1e-300) break;
            const double next = std::clamp(t - f / df, 0.0, 1.0);
            if ((evaluate(next) - p).length() > (evaluate(t) - p).length()) break;
            if (std::fabs(next - t) < 1e-15) { t = next; break; }
            t = next;
        }
        return (evaluate(t) - p).length() <= bestDist ? t : best;
    }
}
