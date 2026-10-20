#include "optibench/curves/BSpline.h"

#include <stdexcept>

namespace ob::curves {

    BSpline::BSpline(int degree, std::vector<Vec2> controlPoints, std::vector<double> knots)
        : degree_(degree), points_(std::move(controlPoints)), knots_(std::move(knots)) {
        if (degree_ < 0) throw std::invalid_argument("BSpline: degree must be >= 0");
        const auto p = static_cast<std::size_t>(degree_);
        if (points_.size() < p + 1) throw std::invalid_argument("BSpline: need at least degree + 1 control points");
        if (knots_.size() != points_.size() + p + 1)
            throw std::invalid_argument("BSpline: knot count must be control points + degree + 1");
        for (std::size_t i = 0; i < knots_.size(); ++i) {
            if (!std::isfinite(knots_[i])) throw std::invalid_argument("BSpline: knots must be finite");
            if (i > 0 && knots_[i] < knots_[i - 1]) throw std::invalid_argument("BSpline: knots must be non-decreasing");
        }
        if (!(knots_[p] < knots_[points_.size()])) throw std::invalid_argument("BSpline: empty parameter domain");
        for (std::size_t i = p + 1; i < points_.size(); ++i)
            if (multiplicity(knots_[i]) > degree_ + 1)
                throw std::invalid_argument("BSpline: interior knot multiplicity exceeds degree + 1");
    }

    BSpline BSpline::clamped(int degree, std::vector<Vec2> controlPoints) {
        if (degree < 0 || controlPoints.size() < static_cast<std::size_t>(degree) + 1)
            throw std::invalid_argument("BSpline::clamped: need at least degree + 1 control points");
        const std::size_t n = controlPoints.size() - 1;
        const auto p = static_cast<std::size_t>(degree);
        std::vector<double> knots(p + 1, 0.0);
        const std::size_t interior = n - p;
        for (std::size_t j = 1; j <= interior; ++j) knots.push_back(static_cast<double>(j) / static_cast<double>(interior + 1));
        knots.insert(knots.end(), p + 1, 1.0);
        return BSpline(degree, std::move(controlPoints), std::move(knots));
    }

    BSpline BSpline::uniform(int degree, std::vector<Vec2> controlPoints) {
        if (degree < 0 || controlPoints.size() < static_cast<std::size_t>(degree) + 1)
            throw std::invalid_argument("BSpline::uniform: need at least degree + 1 control points");
        std::vector<double> knots(controlPoints.size() + static_cast<std::size_t>(degree) + 1);
        for (std::size_t i = 0; i < knots.size(); ++i) knots[i] = static_cast<double>(i);
        return BSpline(degree, std::move(controlPoints), std::move(knots));
    }

    double BSpline::paramMin() const { return knots_[static_cast<std::size_t>(degree_)]; }
    double BSpline::paramMax() const { return knots_[points_.size()]; }

    bool BSpline::isClamped() const {
        const auto p = static_cast<std::size_t>(degree_);
        for (std::size_t i = 1; i <= p; ++i)
            if (knots_[i] != knots_[0] || knots_[knots_.size() - 1 - i] != knots_.back()) return false;
        return true;
    }

    int BSpline::multiplicity(double u) const {
        return static_cast<int>(std::count(knots_.begin(), knots_.end(), u));
    }

    std::size_t BSpline::findSpan(double u) const {
        const auto p = static_cast<std::size_t>(degree_);
        const std::size_t n = points_.size() - 1;
        if (u >= paramMax()) {
            // last non-empty span that ends at the domain end
            std::size_t k = n;
            while (k > p && knots_[k] >= knots_[k + 1]) --k;
            return k;
        }
        const auto it = std::upper_bound(knots_.begin() + static_cast<std::ptrdiff_t>(p),
                                         knots_.begin() + static_cast<std::ptrdiff_t>(n + 1), u);
        return static_cast<std::size_t>(it - knots_.begin()) - 1;
    }

    Vec2 BSpline::evaluate(double u) const {
        u = std::clamp(u, paramMin(), paramMax());
        const auto p = static_cast<std::size_t>(degree_);
        const std::size_t k = findSpan(u);
        std::vector<Vec2> d(points_.begin() + static_cast<std::ptrdiff_t>(k - p),
                            points_.begin() + static_cast<std::ptrdiff_t>(k + 1));
        for (std::size_t r = 1; r <= p; ++r) {
            for (std::size_t j = p; j >= r; --j) {
                const std::size_t i = j + k - p;
                const double denom = knots_[i + p + 1 - r] - knots_[i];
                const double a = denom > 0.0 ? (u - knots_[i]) / denom : 0.0;
                d[j] = lerp(d[j - 1], d[j], a);
            }
        }
        return d[p];
    }

    std::vector<double> BSpline::basis(double u, std::size_t& span) const {
        // "The NURBS Book" algorithm A2.2: the p + 1 non-zero basis functions at u
        u = std::clamp(u, paramMin(), paramMax());
        const auto p = static_cast<std::size_t>(degree_);
        span = findSpan(u);
        std::vector<double> n(p + 1, 0.0), left(p + 1, 0.0), right(p + 1, 0.0);
        n[0] = 1.0;
        for (std::size_t j = 1; j <= p; ++j) {
            left[j] = u - knots_[span + 1 - j];
            right[j] = knots_[span + j] - u;
            double saved = 0.0;
            for (std::size_t r = 0; r < j; ++r) {
                const double denom = right[r + 1] + left[j - r];
                const double temp = denom > 0.0 ? n[r] / denom : 0.0;
                n[r] = saved + right[r + 1] * temp;
                saved = left[j - r] * temp;
            }
            n[j] = saved;
        }
        return n;
    }

    BSpline BSpline::derivativeSpline() const {
        if (degree_ == 0) return BSpline(0, std::vector<Vec2>(points_.size(), Vec2{}), knots_);
        const auto p = static_cast<std::size_t>(degree_);
        std::vector<Vec2> q(points_.size() - 1);
        for (std::size_t i = 0; i < q.size(); ++i) {
            const double denom = knots_[i + p + 1] - knots_[i + 1];
            q[i] = denom > 0.0 ? (points_[i + 1] - points_[i]) * (static_cast<double>(p) / denom) : Vec2{};
        }
        std::vector<double> knots(knots_.begin() + 1, knots_.end() - 1);
        return BSpline(degree_ - 1, std::move(q), std::move(knots));
    }

    Vec2 BSpline::derivative(double u, int order) const {
        if (order <= 0) return evaluate(u);
        if (order > degree_) return {};
        BSpline d = derivativeSpline();
        for (int k = 1; k < order; ++k) d = d.derivativeSpline();
        return d.evaluate(u);
    }

    int BSpline::insertKnot(double u, int times) {
        if (!(u >= paramMin() && u <= paramMax())) return 0;
        const auto p = static_cast<std::size_t>(degree_);
        int inserted = 0;
        for (; inserted < times; ++inserted) {
            const int s = multiplicity(u);
            if (s >= degree_) break;
            // k: last index with u(k) <= u
            const auto it = std::upper_bound(knots_.begin(), knots_.end(), u);
            const std::size_t k = static_cast<std::size_t>(it - knots_.begin()) - 1;
            const auto su = static_cast<std::size_t>(s);

            std::vector<Vec2> q(points_.size() + 1);
            for (std::size_t i = 0; i < q.size(); ++i) {
                if (i + p <= k) {
                    q[i] = points_[i];
                } else if (i + su <= k) {
                    const double a = (u - knots_[i]) / (knots_[i + p] - knots_[i]);
                    q[i] = lerp(points_[i - 1], points_[i], a);
                } else {
                    q[i] = points_[i - 1];
                }
            }
            points_ = std::move(q);
            knots_.insert(knots_.begin() + static_cast<std::ptrdiff_t>(k + 1), u);
        }
        return inserted;
    }

    CompositeBezier BSpline::toBezier() const {
        BSpline work = *this;
        const auto p = static_cast<std::size_t>(degree_);

        // every distinct knot value in the domain gets multiplicity p
        std::vector<double> values;
        for (std::size_t i = p; i <= points_.size(); ++i)
            if (values.empty() || knots_[i] != values.back()) values.push_back(knots_[i]);
        for (double v : values) work.insertKnot(v, degree_);

        CompositeBezier out;
        const std::size_t n = work.points_.size() - 1;
        for (std::size_t k = p; k <= n; ++k) {
            if (!(work.knots_[k] < work.knots_[k + 1])) continue;
            out.append(Bezier(std::vector<Vec2>(work.points_.begin() + static_cast<std::ptrdiff_t>(k - p),
                                                work.points_.begin() + static_cast<std::ptrdiff_t>(k + 1))));
        }
        return out;
    }
}
