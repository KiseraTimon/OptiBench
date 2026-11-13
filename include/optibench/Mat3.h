#pragma once
#include <optional>
#include "Vec2.h"

namespace ob {
    /**
     * CONTRACT: 3x3 homogeneous transformation matrix.            OWNER: Sean
     * (declarations only: implement in src/Mat3.cpp, signatures can be adjusted via PR)
     *
     * Convention: column vectors, p' = M * (x, y, 1)^T, matrices stored row-major.
     * Composite order from the lecture:  M = T * R * S   (the rightmost acts first).
     * Axes: y grows DOWN on screen, so a positive angle looks CLOCKWISE on screen.
     *       (Transform2D in Transform2D.h follows the same convention; tests should cross-check them.)
     */
    struct Mat3 {
        double m[3][3];

        static Mat3 identity();
        static Mat3 translation(double tx, double ty);
        static Mat3 scaling(double sx, double sy);
        static Mat3 rotation(double radians);                       // about the origin
        static Mat3 rotationAbout(double radians, const Vec2& pivot); // T(pivot) * R * T(-pivot)
        static Mat3 reflectX();                                     // x' = x,  y' = -y
        static Mat3 reflectY();                                     // x' = -x, y' = y
        static Mat3 reflectOrigin();                                // x' = -x, y' = -y
        static Mat3 shearX(double k);                               // x' = x + k*y
        static Mat3 shearY(double k);                               // y' = y + k*x

        Mat3 operator*(const Mat3& rhs) const;                      // row-by-column rule
        Vec2 transformPoint(const Vec2& p) const;                   // w = 1: translation applies
        Vec2 transformDirection(const Vec2& d) const;               // w = 0: translation ignored
        std::optional<Mat3> inverse() const;                        // empty if singular
    };
}
