#include "optibench/curves/CurveEditor.h"

#include <stdexcept>

#include "optibench/Line.h"
#include "optibench/curves/BSpline.h"
#include "optibench/curves/CatmullRom.h"
#include "optibench/curves/CubicSpline.h"

namespace ob::curves {

    namespace builders {
        CurveBuilder bezier() {
            return {[](std::span<const Vec2> pts) {
                        return CompositeBezier({Bezier(std::vector<Vec2>(pts.begin(), pts.end()))});
                    },
                    2};
        }

        CurveBuilder catmullRom(double alpha, double tension, bool closed) {
            return {[=](std::span<const Vec2> pts) {
                        return CatmullRom(std::vector<Vec2>(pts.begin(), pts.end()), alpha, tension, closed).toBezier();
                    },
                    closed ? 3u : 2u};
        }

        CurveBuilder naturalCubic(Parametrization parametrization) {
            return {[=](std::span<const Vec2> pts) {
                        return NaturalCubicSpline(std::vector<Vec2>(pts.begin(), pts.end()), parametrization).toBezier();
                    },
                    2};
        }

        CurveBuilder clampedBSpline(int degree) {
            return {[=](std::span<const Vec2> pts) {
                        const int p = std::min(degree, static_cast<int>(pts.size()) - 1);
                        return BSpline::clamped(p, std::vector<Vec2>(pts.begin(), pts.end())).toBezier();
                    },
                    2};
        }
    }

    CurveEditor::CurveEditor(std::vector<Vec2> points, CurveBuilder builder) : builder_(std::move(builder)) {
        if (!builder_.build) throw std::invalid_argument("CurveEditor: builder has no build function");
        if (points.size() < builder_.minPoints) throw std::invalid_argument("CurveEditor: too few points for this curve");
        curve_ = builder_.build(points); // throws on unusable input
        points_ = std::move(points);
    }

    bool CurveEditor::commit(std::vector<Vec2> candidate) {
        if (candidate.size() < builder_.minPoints) return false;
        try {
            CompositeBezier rebuilt = builder_.build(candidate);
            curve_ = std::move(rebuilt);
        } catch (const std::invalid_argument&) {
            return false;
        }
        points_ = std::move(candidate);
        ++revision_;
        return true;
    }

    std::optional<std::size_t> CurveEditor::hitTest(Vec2 p, double radius) const {
        std::optional<std::size_t> best;
        double bestDist = radius;
        for (std::size_t i = 0; i < points_.size(); ++i) {
            const double d = (points_[i] - p).length();
            if (d < bestDist || (d == bestDist && !best)) {
                best = i;
                bestDist = d;
            }
        }
        return best;
    }

    std::optional<CurvePick> CurveEditor::hitTestCurve(Vec2 p, double radius) const {
        if (curve_.empty()) return std::nullopt;
        const double u = curve_.closestParam(p);
        const Vec2 q = curve_.evaluate(u);
        const double d = (q - p).length();
        if (d > radius) return std::nullopt;
        return CurvePick{u, q, d};
    }

    bool CurveEditor::move(std::size_t i, Vec2 p) {
        if (i >= points_.size()) return false;
        std::vector<Vec2> candidate = points_;
        candidate[i] = p;
        return commit(std::move(candidate));
    }

    bool CurveEditor::insert(std::size_t index, Vec2 p) {
        if (index > points_.size()) return false;
        std::vector<Vec2> candidate = points_;
        candidate.insert(candidate.begin() + static_cast<std::ptrdiff_t>(index), p);
        if (dragged_ && *dragged_ >= index) ++*dragged_;
        if (commit(std::move(candidate))) return true;
        if (dragged_ && *dragged_ > index) --*dragged_;
        return false;
    }

    std::optional<std::size_t> CurveEditor::insertNear(Vec2 p) {
        if (points_.empty()) return insert(0, p) ? std::optional<std::size_t>(0) : std::nullopt;
        // nearest control polygon edge; a single point means "append"
        std::size_t index = points_.size();
        std::size_t edge = 0;
        double best = std::numeric_limits<double>::infinity();
        for (std::size_t i = 0; i + 1 < points_.size(); ++i) {
            const double d = distanceToSegment(p, Segment{points_[i], points_[i + 1]});
            if (d < best) { best = d; edge = i; index = i + 1; }
        }
        // beyond the start of the first edge or the end of the last edge: extend the polygon there
        if (points_.size() >= 2) {
            const Vec2 a = points_[edge], b = points_[edge + 1];
            const double lenSq = (b - a).dot(b - a);
            const double t = lenSq > 0.0 ? (p - a).dot(b - a) / lenSq : 0.5;
            if (edge == 0 && t < 0.0) index = 0;
            if (edge + 2 == points_.size() && t > 1.0) index = points_.size();
        }
        if (!insert(index, p)) return std::nullopt;
        return index;
    }

    bool CurveEditor::remove(std::size_t i) {
        if (i >= points_.size() || points_.size() <= builder_.minPoints) return false;
        std::vector<Vec2> candidate = points_;
        candidate.erase(candidate.begin() + static_cast<std::ptrdiff_t>(i));
        if (!commit(std::move(candidate))) return false;
        if (dragged_) {
            if (*dragged_ == i) dragged_.reset();
            else if (*dragged_ > i) --*dragged_;
        }
        return true;
    }

    bool CurveEditor::setBuilder(CurveBuilder builder) {
        if (!builder.build) return false;
        CurveBuilder previous = std::move(builder_);
        builder_ = std::move(builder);
        if (commit(points_)) return true;
        builder_ = std::move(previous);
        return false;
    }

    bool CurveEditor::beginDrag(Vec2 p, double radius) {
        dragged_ = hitTest(p, radius);
        if (!dragged_) return false;
        grabOffset_ = points_[*dragged_] - p;
        return true;
    }

    bool CurveEditor::dragTo(Vec2 p) {
        if (!dragged_) return false;
        return move(*dragged_, p + grabOffset_);
    }
}
