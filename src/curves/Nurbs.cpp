#include "optibench/curves/Nurbs.h"

#include <numbers>
#include <stdexcept>

#include "optibench/Line.h"

namespace ob::curves {
    namespace {
        // homogeneous point (w x, w y, w)
        struct Homog {
            Vec2 xy;
            double w;
        };

        Homog lerpH(const Homog& a, const Homog& b, double t) { return {lerp(a.xy, b.xy, t), a.w + (b.w - a.w) * t}; }

        void validateWeights(std::span<const double> weights) {
            for (double w : weights)
                if (!std::isfinite(w) || w <= 0.0) throw std::invalid_argument("rational curve: weights must be finite and > 0");
        }

        constexpr int kMaxFlattenDepth = 18;

        bool isFlat(std::span<const Vec2> p, double tol) {
            const Segment chord{p.front(), p.back()};
            for (std::size_t i = 1; i + 1 < p.size(); ++i)
                if (distanceToSegment(p[i], chord) > tol) return false;
            return true;
        }

        void flattenRecursive(const RationalBezier& c, double tol, int depth, Polyline& out) {
            if (depth >= kMaxFlattenDepth || isFlat(c.controlPoints(), tol)) {
                out.push_back(c.back());
                return;
            }
            const auto [left, right] = c.split(0.5);
            flattenRecursive(left, tol, depth + 1, out);
            flattenRecursive(right, tol, depth + 1, out);
        }

        // B-spline control points as Vec2 (w x, w y) and (w, 0)
        std::pair<std::vector<Vec2>, std::vector<Vec2>> toHomogeneous(std::span<const Vec2> pts, std::span<const double> w) {
            std::vector<Vec2> a, b;
            for (std::size_t i = 0; i < pts.size(); ++i) {
                a.push_back(pts[i] * w[i]);
                b.push_back({w[i], 0.0});
            }
            return {a, b};
        }
    }

    /*
     * RationalBezier
     */
    RationalBezier::RationalBezier(std::vector<Vec2> points, std::vector<double> weights)
        : points_(std::move(points)), weights_(std::move(weights)) {
        if (points_.empty()) throw std::invalid_argument("RationalBezier: at least one control point is required");
        if (points_.size() != weights_.size()) throw std::invalid_argument("RationalBezier: one weight per point");
        validateWeights(weights_);
    }

    RationalBezier RationalBezier::fromBezier(const Bezier& b) {
        return RationalBezier(std::vector<Vec2>(b.controlPoints().begin(), b.controlPoints().end()),
                              std::vector<double>(b.controlPoints().size(), 1.0));
    }

    Vec2 RationalBezier::evaluate(double t) const {
        std::vector<Homog> work;
        work.reserve(points_.size());
        for (std::size_t i = 0; i < points_.size(); ++i) work.push_back({points_[i] * weights_[i], weights_[i]});
        for (std::size_t r = work.size() - 1; r > 0; --r)
            for (std::size_t i = 0; i < r; ++i) work[i] = lerpH(work[i], work[i + 1], t);
        return work[0].xy / work[0].w;
    }

    Vec2 RationalBezier::derivative(double t) const {
        if (points_.size() < 2) return {};
        // A(t) = sum Bi wi Pi, w(t) = sum Bi wi; C' = (A' - w' C) / w
        std::vector<Vec2> a, wv;
        for (std::size_t i = 0; i < points_.size(); ++i) {
            a.push_back(points_[i] * weights_[i]);
            wv.push_back({weights_[i], 0.0});
        }
        const Bezier ab(a), wb(wv);
        const double w = wb.evaluate(t).x;
        const Vec2 c = ab.evaluate(t) / w;
        return (ab.derivative(t) - c * wb.derivative(t).x) / w;
    }

    double RationalBezier::curvature(double t) const {
        constexpr double h = 1e-5;
        const Vec2 d1 = derivative(t);
        const double speed = d1.length();
        if (speed < 1e-12) return 0.0;
        const double lo = std::max(0.0, t - h), hi = std::min(1.0, t + h);
        const Vec2 d2 = (derivative(hi) - derivative(lo)) / (hi - lo);
        return d1.cross(d2) / (speed * speed * speed);
    }

    std::pair<RationalBezier, RationalBezier> RationalBezier::split(double t) const {
        const std::size_t n = points_.size();
        std::vector<Homog> work;
        for (std::size_t i = 0; i < n; ++i) work.push_back({points_[i] * weights_[i], weights_[i]});
        std::vector<Homog> left(n), right(n);
        for (std::size_t r = 0; r < n; ++r) {
            left[r] = work[0];
            right[n - 1 - r] = work[n - 1 - r];
            for (std::size_t i = 0; i + 1 < n - r; ++i) work[i] = lerpH(work[i], work[i + 1], t);
        }
        auto build = [](const std::vector<Homog>& h) {
            std::vector<Vec2> p;
            std::vector<double> w;
            for (const Homog& q : h) {
                p.push_back(q.xy / q.w);
                w.push_back(q.w);
            }
            return RationalBezier(std::move(p), std::move(w));
        };
        return {build(left), build(right)};
    }

    Polyline RationalBezier::flatten(double tolerance) const {
        const Vec2 s = controlBounds().size();
        const double minTol = 1e-9 * std::max({1.0, s.x, s.y});
        if (!(tolerance > minTol)) tolerance = minTol;
        Polyline out{points_.front()};
        if (points_.size() == 1) return out;
        flattenRecursive(*this, tolerance, 0, out);
        return out;
    }

    std::vector<CurveHit> intersectRay(const Ray& ray, const RationalBezier& curve, const RayCurveOptions& options,
                                       double normalSign) {
        std::vector<CurveHit> hits;
        const Vec2 d = ray.direction.normalized();
        if (d.length() == 0.0 || curve.degree() < 1) return hits;

        std::vector<double> coeffs;
        double reach = 1.0, maxW = 0.0;
        for (std::size_t i = 0; i < curve.controlPoints().size(); ++i) {
            const Vec2 p = curve.controlPoints()[i];
            const double w = curve.weights()[i];
            coeffs.push_back(w * d.cross(p - ray.origin));
            reach = std::max(reach, (p - ray.origin).length());
            maxW = std::max(maxW, w);
        }
        for (double t : bernsteinRoots(coeffs, 1e-10 * reach * maxW)) {
            CurveHit h;
            h.t = h.u = t;
            h.point = curve.evaluate(t);
            h.distance = d.dot(h.point - ray.origin);
            if (h.distance < options.minDistance || h.distance > options.maxDistance) continue;
            const Vec2 tangent = curve.tangent(t);
            h.tangential = std::fabs(tangent.cross(d)) < 1e-7;
            if (h.tangential && !options.includeTangential) continue;
            h.surfaceNormal = tangent.perp() * normalSign;
            h.frontFace = h.surfaceNormal.dot(d) < 0.0;
            if (h.tangential) {
                const double k = curve.curvature(t) * normalSign;
                h.frontFace = (k != 0.0 ? -k : h.surfaceNormal.dot(ray.origin - h.point)) >= 0.0;
            }
            h.normal = h.frontFace ? h.surfaceNormal : -h.surfaceNormal;
            hits.push_back(h);
        }
        std::sort(hits.begin(), hits.end(), [](const CurveHit& a, const CurveHit& b) { return a.distance < b.distance; });
        return hits;
    }

    /*
     * Nurbs
     */
    Nurbs::Nurbs(int degree, std::vector<Vec2> points, std::vector<double> weights, std::vector<double> knots)
        : Nurbs(makeHomogeneous(degree, std::move(points), std::move(weights), std::move(knots))) {}

    Nurbs::Nurbs(std::pair<BSpline, BSpline> parts)
        : homogeneous_(std::move(parts.first)), weightCurve_(std::move(parts.second)) {}

    std::pair<BSpline, BSpline> Nurbs::makeHomogeneous(int degree, std::vector<Vec2> points, std::vector<double> weights,
                                                       std::vector<double> knots) {
        if (degree < 1) throw std::invalid_argument("Nurbs: degree must be >= 1");
        if (points.size() != weights.size()) throw std::invalid_argument("Nurbs: one weight per control point");
        validateWeights(weights);
        auto [a, w] = toHomogeneous(points, weights);
        BSpline homogeneous(degree, std::move(a), knots);
        BSpline weightCurve(degree, std::move(w), std::move(knots));
        return {std::move(homogeneous), std::move(weightCurve)};
    }

    Nurbs Nurbs::circularArc(Vec2 center, double radius, double startAngle, double endAngle) {
        if (!(radius > 0.0)) throw std::invalid_argument("Nurbs::circularArc: radius must be positive");
        double sweep = endAngle - startAngle;
        if (!(std::fabs(sweep) > 0.0) || !std::isfinite(sweep)) throw std::invalid_argument("Nurbs::circularArc: empty sweep");
        sweep = std::clamp(sweep, -2.0 * std::numbers::pi, 2.0 * std::numbers::pi);

        // at most 90 degrees per rational quadratic piece ("The NURBS Book", A7.1)
        const int arcs = std::max(1, static_cast<int>(std::ceil(std::fabs(sweep) / (std::numbers::pi / 2.0) - 1e-12)));
        const double dTheta = sweep / arcs;
        const double wMid = std::cos(dTheta / 2.0);
        auto onCircle = [&](double a, double r) { return center + Vec2(std::cos(a), std::sin(a)) * r; };

        std::vector<Vec2> pts{onCircle(startAngle, radius)};
        std::vector<double> w{1.0};
        std::vector<double> knots{0, 0, 0};
        for (int i = 0; i < arcs; ++i) {
            const double a0 = startAngle + i * dTheta;
            pts.push_back(onCircle(a0 + dTheta / 2.0, radius / wMid)); // tangent lines meet here
            w.push_back(wMid);
            pts.push_back(onCircle(a0 + dTheta, radius));
            w.push_back(1.0);
            if (i + 1 < arcs) {
                const double k = static_cast<double>(i + 1) / arcs;
                knots.push_back(k);
                knots.push_back(k);
            }
        }
        knots.insert(knots.end(), {1, 1, 1});
        return Nurbs(2, std::move(pts), std::move(w), std::move(knots));
    }

    Nurbs Nurbs::circle(Vec2 center, double radius) { return circularArc(center, radius, 0.0, 2.0 * std::numbers::pi); }

    Nurbs Nurbs::ellipse(Vec2 center, double rx, double ry) {
        if (!(rx > 0.0) || !(ry > 0.0)) throw std::invalid_argument("Nurbs::ellipse: radii must be positive");
        // an affine map of the unit circle: rational curves are affinely invariant
        const Nurbs unit = circle({0, 0}, 1.0);
        std::vector<Vec2> pts = unit.controlPoints();
        for (Vec2& p : pts) p = center + Vec2(p.x * rx, p.y * ry);
        return Nurbs(2, std::move(pts), unit.weights(), std::vector<double>(unit.knots().begin(), unit.knots().end()));
    }

    std::vector<Vec2> Nurbs::controlPoints() const {
        std::vector<Vec2> out;
        for (std::size_t i = 0; i < homogeneous_.controlPoints().size(); ++i)
            out.push_back(homogeneous_.controlPoints()[i] / weightCurve_.controlPoints()[i].x);
        return out;
    }

    std::vector<double> Nurbs::weights() const {
        std::vector<double> out;
        for (const Vec2& w : weightCurve_.controlPoints()) out.push_back(w.x);
        return out;
    }

    Vec2 Nurbs::evaluate(double u) const { return homogeneous_.evaluate(u) / weightCurve_.evaluate(u).x; }

    Vec2 Nurbs::derivative(double u) const {
        const double w = weightCurve_.evaluate(u).x;
        const Vec2 c = homogeneous_.evaluate(u) / w;
        return (homogeneous_.derivative(u) - c * weightCurve_.derivative(u).x) / w;
    }

    int Nurbs::insertKnot(double u, int times) {
        const int inserted = homogeneous_.insertKnot(u, times);
        weightCurve_.insertKnot(u, inserted);
        return inserted;
    }

    std::vector<RationalBezier> Nurbs::toRationalBezier() const {
        const CompositeBezier a = homogeneous_.toBezier();
        const CompositeBezier w = weightCurve_.toBezier();
        std::vector<RationalBezier> out;
        for (std::size_t s = 0; s < a.segmentCount(); ++s) {
            std::vector<Vec2> pts;
            std::vector<double> wts;
            for (std::size_t i = 0; i < a.segment(s).controlPoints().size(); ++i) {
                const double wi = w.segment(s).controlPoints()[i].x;
                pts.push_back(a.segment(s).controlPoints()[i] / wi);
                wts.push_back(wi);
            }
            out.emplace_back(std::move(pts), std::move(wts));
        }
        return out;
    }

    Polyline Nurbs::flatten(double tolerance) const {
        Polyline out;
        for (const RationalBezier& seg : toRationalBezier()) {
            const Polyline piece = seg.flatten(tolerance);
            out.insert(out.end(), out.empty() ? piece.begin() : piece.begin() + 1, piece.end());
        }
        return out;
    }

    std::vector<CurveHit> Nurbs::intersect(const Ray& ray, const RayCurveOptions& options, double normalSign) const {
        const std::vector<RationalBezier> segments = toRationalBezier();
        std::vector<CurveHit> all;
        for (std::size_t i = 0; i < segments.size(); ++i) {
            for (CurveHit h : intersectRay(ray, segments[i], options, normalSign)) {
                h.segment = i;
                h.u = static_cast<double>(i) + h.t;
                all.push_back(h);
            }
        }
        std::sort(all.begin(), all.end(), [](const CurveHit& a, const CurveHit& b) { return a.distance < b.distance; });
        std::vector<CurveHit> unique;
        for (const CurveHit& h : all)
            if (unique.empty() || (h.point - unique.back().point).length() > 1e-9 * std::max(1.0, h.distance))
                unique.push_back(h);
        return unique;
    }
}
