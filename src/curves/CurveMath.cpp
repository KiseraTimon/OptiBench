#include "optibench/curves/CurveMath.h"

#include <utility>

namespace ob::curves {
    namespace {
        // intervals narrower than this stop subdividing and become root candidates
        constexpr double kLeafWidth = 1e-7;

        // candidates closer than this (in t) are treated as one root
        constexpr double kMergeGap = 1e-6;

        // hard cap on subdivision work, guarding against nearly-zero polynomials
        constexpr int kMaxNodes = 20000;

        // splits Bernstein coefficients at s into the coefficients of the two halves
        void split1D(std::span<const double> c, double s, std::vector<double>& left, std::vector<double>& right) {
            const std::size_t n = c.size();
            std::vector<double> work(c.begin(), c.end());
            left.resize(n);
            right.resize(n);
            for (std::size_t r = 0; r < n; ++r) {
                left[r] = work[0];
                right[n - 1 - r] = work[n - 1 - r];
                for (std::size_t i = 0; i + 1 < n - r; ++i) work[i] = work[i] + (work[i + 1] - work[i]) * s;
            }
        }

        // number of strict sign changes in the coefficients; -1 if any is within zeroTol of 0
        int signChanges(std::span<const double> c, double zeroTol) {
            int changes = 0;
            for (std::size_t i = 0; i < c.size(); ++i) {
                if (std::fabs(c[i]) <= zeroTol) return -1;
                if (i > 0 && ((c[i] > 0) != (c[i - 1] > 0))) ++changes;
            }
            return changes;
        }

        // bisection on [0, 1] (local coordinates) for a bracketed simple root
        double bisectLocal(std::span<const double> c) {
            double lo = 0.0, hi = 1.0;
            const bool loPositive = c.front() > 0.0;
            for (int i = 0; i < 64 && hi - lo > 1e-16; ++i) {
                const double mid = 0.5 * (lo + hi);
                if ((evalBernstein1D(c, mid) > 0.0) == loPositive) lo = mid; else hi = mid;
            }
            return 0.5 * (lo + hi);
        }

        struct RootSearch {
            double zeroTol = 0.0;
            int nodes = 0;
            std::vector<double> exact;      // roots isolated by the fast path
            std::vector<double> candidates; // leaf midpoints needing polishing

            void run(std::span<const double> c, double a, double b) {
                if (++nodes > kMaxNodes) return;

                const auto [lo, hi] = std::minmax_element(c.begin(), c.end());
                if (*lo > zeroTol || *hi < -zeroTol) return; // convex hull misses zero

                if (signChanges(c, zeroTol) == 1) {
                    exact.push_back(a + (b - a) * bisectLocal(c));
                    return;
                }
                if (b - a < kLeafWidth) {
                    candidates.push_back(0.5 * (a + b));
                    return;
                }

                std::vector<double> left, right;
                split1D(c, 0.5, left, right);
                const double m = 0.5 * (a + b);
                run(left, a, m);
                run(right, m, b);
            }
        };

        // derivative of a Bernstein polynomial (degree n-1 coefficients)
        std::vector<double> derivative1D(std::span<const double> c) {
            if (c.size() < 2) return {0.0};
            const double n = static_cast<double>(c.size() - 1);
            std::vector<double> d(c.size() - 1);
            for (std::size_t i = 0; i + 1 < c.size(); ++i) d[i] = n * (c[i + 1] - c[i]);
            return d;
        }
    }

    Aabb boundsOf(std::span<const Vec2> points) {
        Aabb box;
        for (const Vec2& p : points) box.expand(p);
        return box;
    }

    std::vector<double> knotValues(std::span<const Vec2> points, double alpha) {
        std::vector<double> t;
        t.reserve(points.size());
        if (points.empty()) return t;
        t.push_back(0.0);
        for (std::size_t i = 1; i < points.size(); ++i) {
            const double chord = (points[i] - points[i - 1]).length();
            t.push_back(t.back() + (alpha == 0.0 ? 1.0 : std::pow(chord, alpha)));
        }
        return t;
    }

    std::vector<Vec2> removeConsecutiveDuplicates(std::vector<Vec2> points, double tol) {
        std::vector<Vec2> out;
        out.reserve(points.size());
        for (const Vec2& p : points)
            if (out.empty() || (p - out.back()).length() > tol) out.push_back(p);
        return out;
    }

    double binomial(int n, int k) {
        if (k < 0 || k > n) return 0.0;
        k = std::min(k, n - k);
        double r = 1.0;
        for (int i = 1; i <= k; ++i) r = r * (n - k + i) / i;
        return r;
    }

    double bernstein(int n, int i, double t) {
        if (n < 0 || i < 0 || i > n) return 0.0;
        return binomial(n, i) * std::pow(t, i) * std::pow(1.0 - t, n - i);
    }

    double evalBernstein1D(std::span<const double> coeffs, double t) {
        if (coeffs.empty()) return 0.0;
        std::vector<double> work(coeffs.begin(), coeffs.end());
        for (std::size_t r = work.size() - 1; r > 0; --r)
            for (std::size_t i = 0; i < r; ++i) work[i] = work[i] + (work[i + 1] - work[i]) * t;
        return work[0];
    }

    bool isZeroPolynomial(std::span<const double> coeffs, double zeroTol) {
        return std::all_of(coeffs.begin(), coeffs.end(), [&](double c) { return std::fabs(c) <= zeroTol; });
    }

    std::vector<double> bernsteinRoots(std::span<const double> coeffs, double zeroTol) {
        if (coeffs.empty() || isZeroPolynomial(coeffs, zeroTol)) return {};
        zeroTol = std::max(zeroTol, 0.0);

        RootSearch search;
        search.zeroTol = zeroTol;
        search.run(coeffs, 0.0, 1.0);

        // leaf candidates come in runs of neighbouring intervals (a tangency touches zero over a
        // small range): group each run first, then polish only its best member with Newton
        const std::vector<double> deriv = derivative1D(coeffs);
        std::vector<double> all = search.exact;
        std::sort(search.candidates.begin(), search.candidates.end());
        for (std::size_t i = 0; i < search.candidates.size();) {
            std::size_t j = i;
            double best = search.candidates[i];
            double bestVal = std::fabs(evalBernstein1D(coeffs, best));
            while (j + 1 < search.candidates.size() && search.candidates[j + 1] - search.candidates[j] <= kMergeGap) {
                ++j;
                const double v = std::fabs(evalBernstein1D(coeffs, search.candidates[j]));
                if (v < bestVal) { best = search.candidates[j]; bestVal = v; }
            }
            const double lo = search.candidates[i] - kLeafWidth, hi = search.candidates[j] + kLeafWidth;
            for (int iter = 0; iter < 8; ++iter) {
                const double d = evalBernstein1D(deriv, best);
                if (std::fabs(d) < 1e-300) break;
                const double next = std::clamp(best - evalBernstein1D(coeffs, best) / d, std::max(lo, 0.0), std::min(hi, 1.0));
                const double nextVal = std::fabs(evalBernstein1D(coeffs, next));
                if (!(nextVal < bestVal)) break;
                best = next;
                bestVal = nextVal;
            }
            if (bestVal <= 2.0 * zeroTol) all.push_back(best);
            i = j + 1;
        }

        // merge roots found by both paths (an isolated root next to a candidate run)
        std::sort(all.begin(), all.end());
        std::vector<double> roots;
        for (std::size_t i = 0; i < all.size();) {
            std::size_t j = i;
            double best = all[i];
            double bestVal = std::fabs(evalBernstein1D(coeffs, best));
            while (j + 1 < all.size() && all[j + 1] - all[j] <= kMergeGap) {
                ++j;
                const double v = std::fabs(evalBernstein1D(coeffs, all[j]));
                if (v < bestVal) { best = all[j]; bestVal = v; }
            }
            roots.push_back(best);
            i = j + 1;
        }
        return roots;
    }

    ArcLengthTable::ArcLengthTable(std::function<double(double)> speed, double t0, double t1, int intervals)
        : speed_(std::move(speed)) {
        intervals = std::max(intervals, 1);
        if (!(t1 > t0)) t1 = t0;
        params_.reserve(static_cast<std::size_t>(intervals) + 1);
        lengths_.reserve(static_cast<std::size_t>(intervals) + 1);
        params_.push_back(t0);
        lengths_.push_back(0.0);
        for (int i = 1; i <= intervals; ++i) {
            const double t = t0 + (t1 - t0) * i / intervals;
            lengths_.push_back(lengths_.back() + integrate(speed_, params_.back(), t, 1e-12));
            params_.push_back(t);
        }
    }

    double ArcLengthTable::lengthAt(double t) const {
        t = std::clamp(t, params_.front(), params_.back());
        const auto it = std::upper_bound(params_.begin(), params_.end(), t);
        const std::size_t i = it == params_.begin() ? 0 : static_cast<std::size_t>(it - params_.begin()) - 1;
        if (i + 1 >= params_.size()) return lengths_.back();
        return lengths_[i] + integrate(speed_, params_[i], t, 1e-12);
    }

    double ArcLengthTable::paramAt(double s) const {
        s = std::clamp(s, 0.0, lengths_.back());
        if (s <= 0.0) return params_.front();
        if (s >= lengths_.back()) return params_.back();

        // the table interval that contains s
        const auto it = std::lower_bound(lengths_.begin(), lengths_.end(), s);
        const std::size_t hiIdx = static_cast<std::size_t>(it - lengths_.begin());
        const std::size_t loIdx = hiIdx == 0 ? 0 : hiIdx - 1;
        double lo = params_[loIdx], hi = params_[hiIdx];
        const double segLen = lengths_[hiIdx] - lengths_[loIdx];
        if (segLen <= 0.0) return lo;

        // safeguarded Newton: g(t) = length(t) - s, g'(t) = speed(t)
        double t = lo + (hi - lo) * (s - lengths_[loIdx]) / segLen;
        for (int iter = 0; iter < 40; ++iter) {
            const double g = lengths_[loIdx] + integrate(speed_, params_[loIdx], t, 1e-12) - s;
            if (std::fabs(g) <= 1e-12 * std::max(1.0, lengths_.back())) break;
            if (g > 0) hi = t; else lo = t;
            const double v = speed_(t);
            double next = v > 1e-300 ? t - g / v : 0.5 * (lo + hi);
            if (!(next > lo && next < hi)) next = 0.5 * (lo + hi);
            if (std::fabs(next - t) < 1e-15) { t = next; break; }
            t = next;
        }
        return t;
    }
}
