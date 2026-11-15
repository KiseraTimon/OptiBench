#include "optibench/geometry/Lens.h"

#include <algorithm>
#include <cmath>
#include <stdexcept>

#include "optibench/geometry/Intersect.h"

namespace ob::geom {
    namespace {
        bool isFlat(double r) { return std::isinf(r) && r > 0.0; }

        // 1 / R, with 0 for a flat face
        double curvature(double r) { return isFlat(r) ? 0.0 : 1.0 / r; }

        bool validRadius(double r) { return isFlat(r) || (std::isfinite(r) && r > 0.0); }
    }

    Lens::Lens(double frontRadius, double backRadius, double thickness, double aperture)
        : r1_(frontRadius), r2_(backRadius), d_(thickness), aperture_(aperture), h_(0.0) {
        if (!validRadius(r1_) || !validRadius(r2_)) {
            throw std::invalid_argument("Lens: face radii must be positive (or Lens::kFlat)");
        }
        if (!(std::isfinite(d_) && d_ > 0.0)) throw std::invalid_argument("Lens: thickness must be positive");
        if (!(aperture_ > 0.0)) throw std::invalid_argument("Lens: aperture must be positive (or Lens::kFlat)");
        // where the two faces meet (the natural rim): its x position and half-height
        double natural, rimX;
        const double c1 = -d_ / 2.0 + (isFlat(r1_) ? 0.0 : r1_);    // front face centre (curved)
        const double c2 = d_ / 2.0 - (isFlat(r2_) ? 0.0 : r2_);     // back face centre (curved)
        if (isFlat(r1_) && isFlat(r2_)) {
            natural = kFlat;                                         // a slab: only the aperture bounds it
            rimX = 0.0;
        } else if (isFlat(r2_)) {
            rimX = d_ / 2.0;                                         // front circle meets the plane x = d/2
            natural = std::sqrt(std::max(0.0, d_ * (2.0 * r1_ - d_)));
        } else if (isFlat(r1_)) {
            rimX = -d_ / 2.0;                                        // back circle meets the plane x = -d/2
            natural = std::sqrt(std::max(0.0, d_ * (2.0 * r2_ - d_)));
        } else {
            // radical line of the two circles: x = (c1 + c2)/2 + (R1^2 - R2^2) / (2 (c2 - c1)),
            // where c2 - c1 = d - R1 - R2, so the circles must overlap (d < R1 + R2) for a rim to exist
            if (d_ >= r1_ + r2_) throw std::invalid_argument("Lens: thickness too large for the face radii");
            rimX = (c1 + c2) / 2.0 + (r1_ * r1_ - r2_ * r2_) / (2.0 * (c2 - c1));
            natural = std::sqrt(std::max(0.0, r1_ * r1_ - (rimX - c1) * (rimX - c1)));
        }

        // each curved face is at most a hemisphere: its rim must not pass its own centre.
        // (Real lenses always satisfy this; it keeps the rim the widest part of the lens.)
        if ((!isFlat(r1_) && rimX > c1) || (!isFlat(r2_) && rimX < c2) || !(natural > 0.0)) {
            throw std::invalid_argument("Lens: each curved face can be at most a hemisphere "
                                        "(thickness too large for the radii)");
        }
        if (std::isinf(natural) && std::isinf(aperture_)) {
            throw std::invalid_argument("Lens: two flat faces need a finite aperture");
        }
        h_ = std::min(natural, aperture_);
    }

    double Lens::edgeThickness() const {
        const auto frontX = [&](double y) {
            if (isFlat(r1_)) return -d_ / 2.0;
            const double c1 = -d_ / 2.0 + r1_;
            return c1 - std::sqrt(std::max(0.0, r1_ * r1_ - y * y));
        };
        const auto backX = [&](double y) {
            if (isFlat(r2_)) return d_ / 2.0;
            const double c2 = d_ / 2.0 - r2_;
            return c2 + std::sqrt(std::max(0.0, r2_ * r2_ - y * y));
        };
        return std::max(0.0, backX(h_) - frontX(h_));
    }

    bool Lens::contains(Vec2 p, double tol) const {
        if (std::fabs(p.y) > h_ + tol) return false;
        const bool front = isFlat(r1_) ? p.x >= -d_ / 2.0 - tol
                                       : (p - Vec2{-d_ / 2.0 + r1_, 0.0}).length() <= r1_ + tol;
        const bool back = isFlat(r2_) ? p.x <= d_ / 2.0 + tol
                                      : (p - Vec2{d_ / 2.0 - r2_, 0.0}).length() <= r2_ + tol;
        return front && back;
    }

    Hit Lens::intersect(const Ray& ray, double tMin) const {
        const auto front = isFlat(r1_) ? lineHalfPlane(ray, {-d_ / 2.0, 0.0}, {-1.0, 0.0})
                                       : lineDisc(ray, {-d_ / 2.0 + r1_, 0.0}, r1_);
        const auto back = isFlat(r2_) ? lineHalfPlane(ray, {d_ / 2.0, 0.0}, {1.0, 0.0})
                                      : lineDisc(ray, {d_ / 2.0 - r2_, 0.0}, r2_);
        if (!front || !back) return Hit{};

        std::optional<LineSpan> span = intersectSpans(*front, *back);
        if (span && std::isfinite(h_)) {
            // the rim (natural or cut) is bounded by |y| <= h_: harmless when the faces already meet there
            const auto top = lineHalfPlane(ray, {0.0, h_}, {0.0, 1.0});
            const auto bottom = lineHalfPlane(ray, {0.0, -h_}, {0.0, -1.0});
            span = (top && bottom) ? intersectSpans(*span, *top) : std::nullopt;
            if (span) span = intersectSpans(*span, *bottom);
        }
        if (!span) return Hit{};
        return firstHit(ray, *span, tMin);
    }

    std::vector<Vec2> Lens::outline(int segmentsPerFace) const {
        const int seg = std::max(2, segmentsPerFace);
        std::vector<Vec2> pts;
        pts.reserve(2 * static_cast<std::size_t>(seg) + 2);
        const auto add = [&](Vec2 p) {
            if (pts.empty() || !samePoint(pts.back(), p, 1e-9)) pts.push_back(p);
        };

        // back face, bottom to top (the right-hand side going up is counter-clockwise)
        if (isFlat(r2_)) {
            add({d_ / 2.0, -h_});
            add({d_ / 2.0, h_});
        } else {
            const Vec2 c2{d_ / 2.0 - r2_, 0.0};
            const double b = std::asin(std::clamp(h_ / r2_, -1.0, 1.0));
            for (int i = 0; i <= seg; ++i) {
                const double a = -b + 2.0 * b * i / seg;
                add(c2 + Vec2{std::cos(a), std::sin(a)} * r2_);
            }
        }

        // front face, top to bottom
        if (isFlat(r1_)) {
            add({-d_ / 2.0, h_});
            add({-d_ / 2.0, -h_});
        } else {
            const Vec2 c1{-d_ / 2.0 + r1_, 0.0};
            const double a1 = std::asin(std::clamp(h_ / r1_, -1.0, 1.0));
            for (int i = 0; i <= seg; ++i) {
                const double a = a1 - 2.0 * a1 * i / seg;
                add(c1 + Vec2{-std::cos(a), std::sin(a)} * r1_);
            }
        }
        if (pts.size() > 1 && samePoint(pts.front(), pts.back(), 1e-9)) pts.pop_back();
        return pts;
    }

    Aabb Lens::bounds() const {
        Aabb box;
        box.expand({-d_ / 2.0, -h_});
        box.expand({d_ / 2.0, h_});
        return box;
    }

    double Lens::focalLength(double n) const {
        const double k1 = curvature(r1_), k2 = curvature(r2_);
        const double power = (n - 1.0) * (k1 + k2 - (n - 1.0) * d_ * k1 * k2 / n);
        return power != 0.0 ? 1.0 / power : kFlat;
    }

    double Lens::backFocalDistance(double n) const {
        const double f = focalLength(n);
        return f * (1.0 - (n - 1.0) * d_ * curvature(r1_) / n);
    }

    double Lens::frontFocalDistance(double n) const {
        const double f = focalLength(n);
        return f * (1.0 - (n - 1.0) * d_ * curvature(r2_) / n);
    }
}
