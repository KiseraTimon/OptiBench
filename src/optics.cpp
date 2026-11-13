#include "optibench/optics.h"

#include <cmath>

namespace ob {
    Vec2 reflect(const Vec2& d, const Vec2& n) {
        return d - n * (2.0 * d.dot(n));
    }

    bool refract(const Vec2& d, const Vec2& n, double n1, double n2, Vec2& out) {
        const double eta = n1 / n2;
        const double cosI = -d.dot(n);

        // k < 0 means sin(theta2) would exceed 1: total internal reflection
        const double k = 1.0 - eta * eta * (1.0 - cosI * cosI);
        if (k < 0.0) return false;

        out = (d * eta + n * (eta * cosI - std::sqrt(k))).normalized();
        return true;
    }
}
