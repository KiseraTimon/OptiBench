#include "optibench/curves/CatmullRom.h"

#include <stdexcept>

namespace ob::curves {
    namespace {
        // knot gap between two points for exponent alpha
        double gap(Vec2 a, Vec2 b, double alpha) { return alpha == 0.0 ? 1.0 : std::pow((b - a).length(), alpha); }

        // Bezier segment P1 -> P2 given the neighbours P0 and P3
        Bezier segmentBezier(Vec2 p0, Vec2 p1, Vec2 p2, Vec2 p3, double alpha, double tension) {
            const double d01 = gap(p0, p1, alpha);
            const double d12 = gap(p1, p2, alpha);
            const double d23 = gap(p2, p3, alpha);
            const Vec2 v1 = (p1 - p0) / d01 - (p2 - p0) / (d01 + d12) + (p2 - p1) / d12;
            const Vec2 v2 = (p2 - p1) / d12 - (p3 - p1) / (d12 + d23) + (p3 - p2) / d23;
            const double scale = (1.0 - tension) * d12;
            const Vec2 m1 = v1 * scale;
            const Vec2 m2 = v2 * scale;
            return Bezier({p1, p1 + m1 / 3.0, p2 - m2 / 3.0, p2});
        }
    }

    CatmullRom::CatmullRom(std::vector<Vec2> points, double alpha, double tension, bool closed)
        : points_(removeConsecutiveDuplicates(std::move(points))), alpha_(alpha), tension_(tension), closed_(closed) {
        if (!std::isfinite(alpha_) || alpha_ < 0.0) throw std::invalid_argument("CatmullRom: alpha must be finite and >= 0");
        if (!std::isfinite(tension_)) throw std::invalid_argument("CatmullRom: tension must be finite");
        if (closed_ && points_.size() > 1 && (points_.back() - points_.front()).length() <= 1e-12) points_.pop_back();
        if (points_.size() < (closed_ ? 3u : 2u))
            throw std::invalid_argument("CatmullRom: too few distinct points");

        const std::size_t n = points_.size();
        // neighbour lookup: wraps for closed curves, reflected phantoms for open ones
        auto at = [&](std::ptrdiff_t i) -> Vec2 {
            const auto count = static_cast<std::ptrdiff_t>(n);
            if (closed_) return points_[static_cast<std::size_t>(((i % count) + count) % count)];
            if (i < 0) return points_[0] * 2.0 - points_[1];
            if (i >= count) return points_[n - 1] * 2.0 - points_[n - 2];
            return points_[static_cast<std::size_t>(i)];
        };

        const std::size_t segments = closed_ ? n : n - 1;
        for (std::size_t s = 0; s < segments; ++s) {
            const auto i = static_cast<std::ptrdiff_t>(s);
            bezier_.append(segmentBezier(at(i - 1), at(i), at(i + 1), at(i + 2), alpha_, tension_));
        }
    }
}
