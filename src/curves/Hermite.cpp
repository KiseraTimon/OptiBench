#include "optibench/curves/Hermite.h"

#include <stdexcept>

namespace ob::curves {

    std::array<double, 4> hermiteBasis(double t) {
        const double t2 = t * t;
        const double t3 = t2 * t;
        return {2 * t3 - 3 * t2 + 1, t3 - 2 * t2 + t, -2 * t3 + 3 * t2, t3 - t2};
    }

    Vec2 HermiteSegment::evaluate(double t) const {
        const auto h = hermiteBasis(t);
        return p0 * h[0] + m0 * h[1] + p1 * h[2] + m1 * h[3];
    }

    Vec2 HermiteSegment::derivative(double t) const {
        const double t2 = t * t;
        return p0 * (6 * t2 - 6 * t) + m0 * (3 * t2 - 4 * t + 1) + p1 * (-6 * t2 + 6 * t) + m1 * (3 * t2 - 2 * t);
    }

    HermiteSpline::HermiteSpline(std::vector<Vec2> points, std::vector<Vec2> tangents)
        : points_(std::move(points)), tangents_(std::move(tangents)) {
        if (points_.size() < 2) throw std::invalid_argument("HermiteSpline: at least 2 points are required");
        if (points_.size() != tangents_.size())
            throw std::invalid_argument("HermiteSpline: points and tangents must have the same size");
    }

    HermiteSegment HermiteSpline::segment(std::size_t i) const {
        return {points_.at(i), tangents_.at(i), points_.at(i + 1), tangents_.at(i + 1)};
    }

    Vec2 HermiteSpline::evaluate(double u) const {
        u = std::clamp(u, 0.0, static_cast<double>(segmentCount()));
        const std::size_t i = std::min(static_cast<std::size_t>(u), segmentCount() - 1);
        return segment(i).evaluate(u - static_cast<double>(i));
    }

    CompositeBezier HermiteSpline::toBezier() const {
        CompositeBezier out;
        for (std::size_t i = 0; i < segmentCount(); ++i) out.append(segment(i).toBezier());
        return out;
    }
}
