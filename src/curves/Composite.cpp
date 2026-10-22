#include "optibench/curves/Composite.h"

#include <cmath>

namespace ob::curves {
    namespace {
        // size of a box, used to make tolerances relative
        double extent(const Aabb& box) {
            const Vec2 s = box.size();
            return std::max({1.0, s.x, s.y});
        }

        // derivative at the end of p and at the start of q, as used by the C1 condition
        Vec2 endDerivative(const Bezier& p) { return p.derivative(1.0); }
        Vec2 startDerivative(const Bezier& q) { return q.derivative(0.0); }
    }

    void CompositeBezier::append(Bezier segment, Continuity continuity) {
        segments_.push_back(std::move(segment));
        if (segments_.size() >= 2) enforce(segments_.size() - 2, continuity);
    }

    std::pair<std::size_t, double> CompositeBezier::locate(double u) const {
        if (segments_.empty()) return {0, 0.0};
        u = std::clamp(u, 0.0, paramMax());
        if (!(u == u)) u = 0.0; // NaN guard
        const std::size_t i = std::min(static_cast<std::size_t>(std::floor(u)), segments_.size() - 1);
        return {i, u - static_cast<double>(i)};
    }

    Vec2 CompositeBezier::evaluate(double u) const {
        if (empty()) return {};
        const auto [i, t] = locate(u);
        return segments_[i].evaluate(t);
    }

    Vec2 CompositeBezier::derivative(double u, int order) const {
        if (empty()) return {};
        const auto [i, t] = locate(u);
        return segments_[i].derivative(t, order);
    }

    Vec2 CompositeBezier::tangent(double u) const {
        if (empty()) return {};
        const auto [i, t] = locate(u);
        return segments_[i].tangent(t);
    }

    Vec2 CompositeBezier::normal(double u) const { return tangent(u).perp(); }

    double CompositeBezier::curvature(double u) const {
        if (empty()) return 0.0;
        const auto [i, t] = locate(u);
        return segments_[i].curvature(t);
    }

    bool CompositeBezier::isClosed(double tol) const {
        if (empty()) return false;
        return (back() - front()).length() <= tol * extent(controlBounds());
    }

    std::size_t CompositeBezier::joinCount() const {
        if (segments_.size() < 2) return isClosed() ? segments_.size() : 0;
        return segments_.size() - 1 + (isClosed() ? 1 : 0);
    }

    Continuity CompositeBezier::continuityAt(std::size_t join, double tol) const {
        if (segments_.empty() || join >= joinCount()) return Continuity::None;
        const Bezier& p = segments_[join];
        const Bezier& q = segments_[(join + 1) % segments_.size()];
        const double scale = extent(controlBounds());

        if ((p.back() - q.front()).length() > tol * scale) return Continuity::None;
        const Vec2 dp = endDerivative(p);
        const Vec2 dq = startDerivative(q);
        if ((dp - dq).length() <= tol * scale * std::max(1, std::max(p.degree(), q.degree())))
            return Continuity::C1;
        const Vec2 tp = p.tangent(1.0);
        const Vec2 tq = q.tangent(0.0);
        if (tp.length() > 0.0 && tq.length() > 0.0 && tp.dot(tq) >= 1.0 - tol) return Continuity::G1;
        return Continuity::C0;
    }

    Continuity CompositeBezier::continuity(double tol) const {
        Continuity weakest = Continuity::C1;
        for (std::size_t j = 0; j < joinCount(); ++j) weakest = std::min(weakest, continuityAt(j, tol));
        return weakest;
    }

    void CompositeBezier::enforce(std::size_t join, Continuity c) {
        if (c == Continuity::None || segments_.size() < 2) return;
        const std::size_t qi = (join + 1) % segments_.size();
        if (join >= segments_.size() || (qi == 0 && join + 1 != segments_.size())) return;
        const Bezier& p = segments_[join];
        Bezier& q = segments_[qi];

        // C0: snap the start of q onto the end of p
        q.setControlPoint(0, p.back());
        if (c == Continuity::C0 || q.degree() < 1 || p.degree() < 1) return;

        const Vec2 q0 = q.front();
        if (c == Continuity::C1) {
            // m (Q1 - Q0) = n (Pn - P(n-1))
            const Vec2 dp = endDerivative(p);
            q.setControlPoint(1, q0 + dp / static_cast<double>(q.degree()));
            return;
        }

        // G1: align Q1 with p's end tangent, keeping q's handle length when it has one
        const Vec2 dir = p.tangent(1.0);
        if (dir.length() == 0.0) return; // p is a single point: no direction to match
        double handle = (q.controlPoint(1) - q0).length();
        if (handle <= 0.0) handle = endDerivative(p).length() / static_cast<double>(q.degree());
        if (handle <= 0.0) handle = 1e-3 * extent(p.controlBounds());
        q.setControlPoint(1, q0 + dir * handle);
    }

    void CompositeBezier::enforceAll(Continuity c) {
        const std::size_t joins = joinCount();
        for (std::size_t j = 0; j < joins; ++j) enforce(j, c);
    }

    Polyline CompositeBezier::flatten(double tolerance) const {
        Polyline out;
        for (const Bezier& s : segments_) {
            const Polyline piece = s.flatten(tolerance);
            out.insert(out.end(), out.empty() ? piece.begin() : piece.begin() + 1, piece.end());
        }
        return out;
    }

    Polyline CompositeBezier::flattenUniform(int stepsPerSegment) const {
        Polyline out;
        for (const Bezier& s : segments_) {
            const Polyline piece = s.flattenUniform(stepsPerSegment);
            out.insert(out.end(), out.empty() ? piece.begin() : piece.begin() + 1, piece.end());
        }
        return out;
    }

    Aabb CompositeBezier::bounds() const {
        Aabb box;
        for (const Bezier& s : segments_) box.expand(s.bounds());
        return box;
    }

    Aabb CompositeBezier::controlBounds() const {
        Aabb box;
        for (const Bezier& s : segments_) box.expand(s.controlBounds());
        return box;
    }

    double CompositeBezier::arcLength() const {
        double sum = 0.0;
        for (const Bezier& s : segments_) sum += s.arcLength();
        return sum;
    }

    double CompositeBezier::closestParam(Vec2 p) const {
        double bestU = 0.0;
        double bestDist = std::numeric_limits<double>::infinity();
        for (std::size_t i = 0; i < segments_.size(); ++i) {
            const double t = segments_[i].closestParam(p);
            const double d = (segments_[i].evaluate(t) - p).length();
            if (d < bestDist) { bestDist = d; bestU = static_cast<double>(i) + t; }
        }
        return bestU;
    }

    std::vector<Vec2> CompositeBezier::controlPolygon() const {
        std::vector<Vec2> out;
        for (const Bezier& s : segments_) out.insert(out.end(), s.controlPoints().begin(), s.controlPoints().end());
        return out;
    }
}
