#pragma once

/*
 * Special Notes: Curves Math Utilities
 *
 * Small numeric building blocks shared by every curve type in ob::curves:
 *
 *   Aabb                 axis-aligned bounding box
 *   bernstein()          Bernstein basis polynomial B(i,n)(t) = C(n,i) t^i (1-t)^(n-i)
 *   bernsteinRoots()     robust roots of a scalar polynomial given in Bernstein form
 *   solveTridiagonal()   Thomas algorithm, O(n)
 *   integrate()          adaptive 5-point Gauss-Legendre quadrature
 *   ArcLengthTable       cumulative arc length s(t) and its inverse t(s)
 *
 * Nothing here draws; everything is plain math on ob::Vec2 and doubles.
 */

#include <algorithm>
#include <array>
#include <cmath>
#include <concepts>
#include <functional>
#include <limits>
#include <span>
#include <type_traits>
#include <vector>

#include "optibench/Vec2.h"

namespace ob::curves {

    // an ordered list of points joined by straight segments
    using Polyline = std::vector<Vec2>;

    // axis-aligned bounding box; empty (min > max) until a point is added
    struct Aabb {
        Vec2 min{std::numeric_limits<double>::infinity(), std::numeric_limits<double>::infinity()};
        Vec2 max{-std::numeric_limits<double>::infinity(), -std::numeric_limits<double>::infinity()};

        // true if no point has been added yet
        bool empty() const { return min.x > max.x || min.y > max.y; }

        // grows the box to include p
        void expand(Vec2 p) {
            min = {std::min(min.x, p.x), std::min(min.y, p.y)};
            max = {std::max(max.x, p.x), std::max(max.y, p.y)};
        }

        // grows the box to include another box
        void expand(const Aabb& o) {
            if (o.empty()) return;
            expand(o.min);
            expand(o.max);
        }

        // width and height
        Vec2 size() const { return empty() ? Vec2{} : max - min; }

        // centre point
        Vec2 center() const { return empty() ? Vec2{} : (min + max) * 0.5; }

        // true if p is inside the box grown by tol on every side
        bool contains(Vec2 p, double tol = 0.0) const {
            return p.x >= min.x - tol && p.x <= max.x + tol && p.y >= min.y - tol && p.y <= max.y + tol;
        }
    };

    // the smallest box containing every point
    Aabb boundsOf(std::span<const Vec2> points);

    // linear interpolation between two points; t = 0 gives a, t = 1 gives b
    inline Vec2 lerp(Vec2 a, Vec2 b, double t) { return a + (b - a) * t; }

    /**
     * How parameter values (knots) are assigned to interpolated points: the knot gap between
     * neighbours is |P(i+1) - Pi|^alpha with alpha = 0 (uniform), 0.5 (centripetal) or 1 (chordal).
     * Centripetal avoids cusps and self-intersections within a segment (Yuksel et al. 2011);
     * chordal follows the point spacing most closely.
     */
    enum class Parametrization { Uniform, Centripetal, Chordal };

    // the exponent alpha for a parametrization
    constexpr double alphaOf(Parametrization p) {
        return p == Parametrization::Uniform ? 0.0 : p == Parametrization::Centripetal ? 0.5 : 1.0;
    }

    /**
     * cumulative knot values t0 = 0, t(i+1) = ti + |P(i+1) - Pi|^alpha
     * Coincident neighbours would create a zero gap; callers remove them first with
     * removeConsecutiveDuplicates().
     */
    std::vector<double> knotValues(std::span<const Vec2> points, double alpha);

    // drops points closer than tol to their predecessor (keeps the first of each run)
    std::vector<Vec2> removeConsecutiveDuplicates(std::vector<Vec2> points, double tol = 1e-12);

    // binomial coefficient C(n, k) as a double; 0 when k is out of range
    double binomial(int n, int k);

    /**
     * Bernstein basis polynomial B(i,n)(t) = C(n,i) t^i (1-t)^(n-i)
     * @param n int: degree (>= 0)
     * @param i int: index; returns 0 outside 0..n
     * @param t double: parameter, usually in [0, 1]
     */
    double bernstein(int n, int i, double t);

    // evaluates a scalar polynomial in Bernstein form at t (de Casteljau)
    double evalBernstein1D(std::span<const double> coeffs, double t);

    /**
     * Finds the roots in [0, 1] of the scalar polynomial f(t) = sum c[i] B(i,n)(t).
     *
     * Method: recursive subdivision using the convex hull property (if every coefficient has
     * the same sign there is no root in the interval), a bracketed bisection fast path as soon
     * as an interval has exactly one coefficient sign change (Descartes' rule for Bernstein
     * polynomials guarantees exactly one root there), and Newton polishing of the remaining
     * candidates. Double roots (tangencies) are found because the hull test treats coefficients
     * within zeroTol of zero as touching zero; clustered candidates are merged.
     *
     * @param coeffs Bernstein coefficients c[0..n]
     * @param zeroTol double: absolute magnitude treated as zero (scale it to your data)
     * @return sorted roots; empty if there are none or if f is identically zero (|c[i]| <= zeroTol)
     */
    std::vector<double> bernsteinRoots(std::span<const double> coeffs, double zeroTol);

    // true if every coefficient is within zeroTol of zero
    bool isZeroPolynomial(std::span<const double> coeffs, double zeroTol);

    /**
     * Solves a tridiagonal system with the Thomas algorithm in O(n).
     *
     *   diag[0] x0 + sup[0] x1                              = rhs[0]
     *   sub[i] x(i-1) + diag[i] x(i) + sup[i] x(i+1)        = rhs[i]
     *   sub[n-1] x(n-2) + diag[n-1] x(n-1)                  = rhs[n-1]
     *
     * T may be double or Vec2 (anything with +, - and * / by double). The method needs no
     * pivoting for diagonally dominant matrices, which is the case for spline systems.
     *
     * @return the solution, or an empty vector if sizes mismatch or a pivot is (near) zero
     */
    template <class T>
    std::vector<T> solveTridiagonal(std::span<const double> sub, std::span<const double> diag,
                                    std::span<const double> sup, std::span<const T> rhs) {
        const std::size_t n = diag.size();
        if (n == 0 || sub.size() != n || sup.size() != n || rhs.size() != n) return {};

        std::vector<double> c(n);
        std::vector<T> d(n);
        double pivot = diag[0];
        if (std::fabs(pivot) < 1e-300) return {};
        c[0] = sup[0] / pivot;
        d[0] = rhs[0] / pivot;
        for (std::size_t i = 1; i < n; ++i) {
            pivot = diag[i] - sub[i] * c[i - 1];
            if (std::fabs(pivot) < 1e-300) return {};
            c[i] = sup[i] / pivot;
            d[i] = (rhs[i] - d[i - 1] * sub[i]) / pivot;
        }
        for (std::size_t i = n - 1; i-- > 0;) d[i] = d[i] - d[i + 1] * c[i];
        return d;
    }

    namespace detail {
        // 5-point Gauss-Legendre rule on [a, b]
        template <class F>
        double gauss5(F& f, double a, double b) {
            static constexpr std::array<double, 5> kNodes{0.0, -0.5384693101056831, 0.5384693101056831,
                                                          -0.9061798459386640, 0.9061798459386640};
            static constexpr std::array<double, 5> kWeights{0.5688888888888889, 0.4786286704993665,
                                                            0.4786286704993665, 0.2369268850561891,
                                                            0.2369268850561891};
            const double half = 0.5 * (b - a);
            const double mid = 0.5 * (a + b);
            double sum = 0.0;
            for (std::size_t i = 0; i < kNodes.size(); ++i) sum += kWeights[i] * f(mid + half * kNodes[i]);
            return sum * half;
        }

        template <class F>
        double adaptiveGauss(F& f, double a, double b, double whole, double tol, int depth) {
            const double m = 0.5 * (a + b);
            const double left = gauss5(f, a, m);
            const double right = gauss5(f, m, b);
            if (depth <= 0 || std::fabs(left + right - whole) <= tol) return left + right;
            return adaptiveGauss(f, a, m, left, 0.5 * tol, depth - 1) +
                   adaptiveGauss(f, m, b, right, 0.5 * tol, depth - 1);
        }
    }

    /**
     * Integrates f over [a, b] with adaptive 5-point Gauss-Legendre quadrature.
     * Exact for polynomials up to degree 9 per panel, so a few panels suffice for smooth
     * curves; panels are halved until neighbouring estimates agree within tol.
     */
    template <class F>
        requires std::invocable<F&, double> && std::convertible_to<std::invoke_result_t<F&, double>, double>
    double integrate(F&& f, double a, double b, double tol = 1e-10, int maxDepth = 24) {
        if (!(b > a)) return 0.0;
        return detail::adaptiveGauss(f, a, b, detail::gauss5(f, a, b), tol, maxDepth);
    }

    /**
     * Cumulative arc length table for any parametric curve, given its speed |C'(t)|.
     *
     * Building: O(intervals) quadratures. lengthAt(t) and paramAt(s) use a binary search in
     * the table plus one local quadrature (and a safeguarded Newton solve for paramAt), so
     * they stay accurate to roughly the quadrature tolerance, not the table resolution.
     */
    class ArcLengthTable {
    public:
        /**
         * @param speed |C'(t)|, must be finite and >= 0 on [t0, t1]
         * @param t0 double: parameter where arc length is 0
         * @param t1 double: end parameter (> t0)
         * @param intervals int: table resolution (>= 1)
         */
        ArcLengthTable(std::function<double(double)> speed, double t0, double t1, int intervals = 64);

        // total length between t0 and t1
        double totalLength() const { return lengths_.back(); }

        // arc length from t0 to t (t is clamped to [t0, t1])
        double lengthAt(double t) const;

        // parameter whose arc length from t0 is s (s is clamped to [0, totalLength()])
        double paramAt(double s) const;

    private:
        std::function<double(double)> speed_;
        std::vector<double> params_;
        std::vector<double> lengths_;
    };
}
