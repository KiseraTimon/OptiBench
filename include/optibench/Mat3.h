#pragma once
#include <optional>
#include <vector>
#include "Transform2D.h"
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

        /*
         * Transform & optics maths additions
         */
        /**
         * mirror image across the infinite line through a and b
         * The linear part is I - 2 n n^T (n = unit normal of the line), so on a direction it gives
         * exactly optics.h reflect(d, n). If a == b the line has no direction: returns identity().
         */
        static Mat3 reflectLine(const Vec2& a, const Vec2& b);

        // the lecture's composite T * R * S: scale first, then rotate about the origin, then translate
        static Mat3 composite(const Vec2& translate, double radians, const Vec2& scale);

        // the same local -> world matrix a Transform2D applies (composite(position, rotation, scale))
        static Mat3 fromTransform(const Transform2D& t);

        // area scale factor of the linear part; negative means the transform mirrors (winding flips)
        double determinant() const;

        // surface normal: inverse-transpose of the linear part, re-normalised (zero vector if singular)
        Vec2 transformNormal(const Vec2& n) const;

        // every point of a vertex list (e.g. Polygon from clip_fill.h, or Polygon::vertices)
        std::vector<Vec2> transformPoints(const std::vector<Vec2>& points) const;
    };
}
