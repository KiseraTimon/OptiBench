#include "optibench/curves/CubicSpline.h"

#include <stdexcept>

namespace ob::curves {

    NaturalCubicSpline::NaturalCubicSpline(std::vector<Vec2> points, Parametrization parametrization)
        : points_(removeConsecutiveDuplicates(std::move(points))) {
        if (points_.size() < 2) throw std::invalid_argument("NaturalCubicSpline: at least 2 distinct points are required");
        knots_ = knotValues(points_, alphaOf(parametrization));

        const std::size_t n = points_.size() - 1; // number of intervals
        m_.assign(points_.size(), Vec2{});
        if (n < 2) return; // a single interval is a straight line

        // tridiagonal system for the interior second derivatives M1..M(n-1)
        const std::size_t unknowns = n - 1;
        std::vector<double> sub(unknowns), diag(unknowns), sup(unknowns);
        std::vector<Vec2> rhs(unknowns);
        for (std::size_t r = 0; r < unknowns; ++r) {
            const std::size_t i = r + 1;
            const double h0 = knots_[i] - knots_[i - 1];
            const double h1 = knots_[i + 1] - knots_[i];
            sub[r] = r == 0 ? 0.0 : h0;
            diag[r] = 2.0 * (h0 + h1);
            sup[r] = r + 1 == unknowns ? 0.0 : h1;
            rhs[r] = ((points_[i + 1] - points_[i]) / h1 - (points_[i] - points_[i - 1]) / h0) * 6.0;
        }
        const std::vector<Vec2> interior = solveTridiagonal<Vec2>(sub, diag, sup, rhs);
        for (std::size_t r = 0; r < interior.size(); ++r) m_[r + 1] = interior[r];
    }

    std::size_t NaturalCubicSpline::interval(double t) const {
        const auto it = std::upper_bound(knots_.begin(), knots_.end(), t);
        const std::size_t idx = it == knots_.begin() ? 0 : static_cast<std::size_t>(it - knots_.begin()) - 1;
        return std::min(idx, knots_.size() - 2);
    }

    Vec2 NaturalCubicSpline::evaluate(double t) const {
        t = std::clamp(t, paramMin(), paramMax());
        const std::size_t i = interval(t);
        const double h = knots_[i + 1] - knots_[i];
        const double a = (knots_[i + 1] - t) / h;
        const double b = 1.0 - a;
        return points_[i] * a + points_[i + 1] * b + (m_[i] * (a * a * a - a) + m_[i + 1] * (b * b * b - b)) * (h * h / 6.0);
    }

    Vec2 NaturalCubicSpline::derivative(double t, int order) const {
        if (order <= 0) return evaluate(t);
        if (order > 3) return {};
        t = std::clamp(t, paramMin(), paramMax());
        const std::size_t i = interval(t);
        const double h = knots_[i + 1] - knots_[i];
        const double a = (knots_[i + 1] - t) / h;
        const double b = 1.0 - a;
        switch (order) {
            case 1:
                return (points_[i + 1] - points_[i]) / h + (m_[i + 1] * (3 * b * b - 1) - m_[i] * (3 * a * a - 1)) * (h / 6.0);
            case 2: return m_[i] * a + m_[i + 1] * b;
            default: return (m_[i + 1] - m_[i]) / h;
        }
    }

    CompositeBezier NaturalCubicSpline::toBezier() const {
        // Hermite form per interval: end derivatives scaled by h give the Bezier handles
        CompositeBezier out;
        for (std::size_t i = 0; i + 1 < points_.size(); ++i) {
            const double h = knots_[i + 1] - knots_[i];
            const Vec2 chord = (points_[i + 1] - points_[i]) / h;
            const Vec2 d0 = chord - (m_[i] * 2.0 + m_[i + 1]) * (h / 6.0);
            const Vec2 d1 = chord + (m_[i] + m_[i + 1] * 2.0) * (h / 6.0);
            out.append(Bezier({points_[i], points_[i] + d0 * (h / 3.0), points_[i + 1] - d1 * (h / 3.0), points_[i + 1]}));
        }
        return out;
    }
}
