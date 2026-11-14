#include "optibench/Mat3.h"

#include <cmath>

namespace ob {
    namespace {
        // determinants smaller than this are treated as zero (the matrix squashes space flat)
        constexpr double kSingular = 1e-12;
    }


    /*
     * Basic matrices
     */
    // logic building the identity: leaves every point where it is
    Mat3 Mat3::identity() {
        return {{{1.0, 0.0, 0.0},
                 {0.0, 1.0, 0.0},
                 {0.0, 0.0, 1.0}}};
    }

    // logic building a translation: x' = x + tx, y' = y + ty
    Mat3 Mat3::translation(double tx, double ty) {
        return {{{1.0, 0.0, tx},
                 {0.0, 1.0, ty},
                 {0.0, 0.0, 1.0}}};
    }

    // logic building a scale about the origin: x' = sx * x, y' = sy * y
    Mat3 Mat3::scaling(double sx, double sy) {
        return {{{sx, 0.0, 0.0},
                 {0.0, sy, 0.0},
                 {0.0, 0.0, 1.0}}};
    }

    // logic building a rotation about the origin (same sense as Transform2D::rotateBy)
    Mat3 Mat3::rotation(double radians) {
        const double c = std::cos(radians), s = std::sin(radians);
        return {{{c, -s, 0.0},
                 {s, c, 0.0},
                 {0.0, 0.0, 1.0}}};
    }

    // logic rotating about a pivot: move the pivot to the origin, rotate, move it back
    Mat3 Mat3::rotationAbout(double radians, const Vec2& pivot) {
        return translation(pivot.x, pivot.y) * rotation(radians) * translation(-pivot.x, -pivot.y);
    }

    // logic mirroring across the x-axis
    Mat3 Mat3::reflectX() { return scaling(1.0, -1.0); }

    // logic mirroring across the y-axis
    Mat3 Mat3::reflectY() { return scaling(-1.0, 1.0); }

    // logic mirroring through the origin (the same as a half turn)
    Mat3 Mat3::reflectOrigin() { return scaling(-1.0, -1.0); }

    // logic shearing along x: points slide sideways in proportion to their height
    Mat3 Mat3::shearX(double k) {
        return {{{1.0, k, 0.0},
                 {0.0, 1.0, 0.0},
                 {0.0, 0.0, 1.0}}};
    }

    // logic shearing along y: points slide up/down in proportion to their x
    Mat3 Mat3::shearY(double k) {
        return {{{1.0, 0.0, 0.0},
                 {k, 1.0, 0.0},
                 {0.0, 0.0, 1.0}}};
    }


    /*
     * Matrix operations
     */
    // logic multiplying matrices: entry (i, j) is row i of this times column j of rhs
    Mat3 Mat3::operator*(const Mat3& rhs) const {
        Mat3 out{};
        for (int i = 0; i < 3; ++i)
            for (int j = 0; j < 3; ++j)
                for (int k = 0; k < 3; ++k)
                    out.m[i][j] += m[i][k] * rhs.m[k][j];
        return out;
    }

    // logic applying the matrix to a point (x, y, 1); divides by w so general homogeneous matrices work too
    Vec2 Mat3::transformPoint(const Vec2& p) const {
        const double x = m[0][0] * p.x + m[0][1] * p.y + m[0][2];
        const double y = m[1][0] * p.x + m[1][1] * p.y + m[1][2];
        const double w = m[2][0] * p.x + m[2][1] * p.y + m[2][2];
        return {x / w, y / w};
    }

    // logic applying the matrix to a direction (x, y, 0); the translation column drops out
    Vec2 Mat3::transformDirection(const Vec2& d) const {
        return {m[0][0] * d.x + m[0][1] * d.y,
                m[1][0] * d.x + m[1][1] * d.y};
    }

    // logic inverting with the adjugate: inverse = adj(M) / det(M)
    std::optional<Mat3> Mat3::inverse() const {
        // cofactors of the first row give the full 3x3 determinant
        const double c00 = m[1][1] * m[2][2] - m[1][2] * m[2][1];
        const double c01 = m[1][2] * m[2][0] - m[1][0] * m[2][2];
        const double c02 = m[1][0] * m[2][1] - m[1][1] * m[2][0];
        const double det = m[0][0] * c00 + m[0][1] * c01 + m[0][2] * c02;
        if (std::fabs(det) < kSingular) return std::nullopt;

        const double k = 1.0 / det;
        Mat3 inv{};
        inv.m[0][0] = c00 * k;
        inv.m[0][1] = (m[0][2] * m[2][1] - m[0][1] * m[2][2]) * k;
        inv.m[0][2] = (m[0][1] * m[1][2] - m[0][2] * m[1][1]) * k;
        inv.m[1][0] = c01 * k;
        inv.m[1][1] = (m[0][0] * m[2][2] - m[0][2] * m[2][0]) * k;
        inv.m[1][2] = (m[0][2] * m[1][0] - m[0][0] * m[1][2]) * k;
        inv.m[2][0] = c02 * k;
        inv.m[2][1] = (m[0][1] * m[2][0] - m[0][0] * m[2][1]) * k;
        inv.m[2][2] = (m[0][0] * m[1][1] - m[0][1] * m[1][0]) * k;
        return inv;
    }


    /*
     * Transform & optics maths additions
     */
    // logic mirroring across the line through a and b
    Mat3 Mat3::reflectLine(const Vec2& a, const Vec2& b) {
        // unit normal of the line; normalized() gives the zero vector when a == b, leaving identity
        const Vec2 n = (b - a).normalized().perp();

        /**
         * CONTEXT:
         *      a point p moves twice its signed distance to the line, against the normal:
         *          p' = p - 2((p - a).n) n  =  (I - 2 n n^T) p  +  2(a.n) n
         */
        const double d = 2.0 * a.dot(n);
        return {{{1.0 - 2.0 * n.x * n.x, -2.0 * n.x * n.y, d * n.x},
                 {-2.0 * n.x * n.y, 1.0 - 2.0 * n.y * n.y, d * n.y},
                 {0.0, 0.0, 1.0}}};
    }

    // logic building T * R * S (the rightmost, scale, acts first)
    Mat3 Mat3::composite(const Vec2& translate, double radians, const Vec2& scale) {
        return translation(translate.x, translate.y) * rotation(radians) * scaling(scale.x, scale.y);
    }

    // logic turning an element's Transform2D into its matrix
    Mat3 Mat3::fromTransform(const Transform2D& t) {
        return composite(t.position, t.rotation, t.scale);
    }

    // logic calculating the determinant of the 2x2 linear part
    double Mat3::determinant() const {
        return m[0][0] * m[1][1] - m[0][1] * m[1][0];
    }

    // logic carrying a normal: multiply by (A^-1)^T so it stays perpendicular under non-uniform scale/shear
    Vec2 Mat3::transformNormal(const Vec2& n) const {
        const double det = determinant();
        if (std::fabs(det) < kSingular) return {};

        // (A^-1)^T = (1/det) [[ d, -c], [-b, a]]; dividing by det keeps the side the normal points to
        const Vec2 out{(m[1][1] * n.x - m[1][0] * n.y) / det,
                       (-m[0][1] * n.x + m[0][0] * n.y) / det};
        return out.normalized();
    }

    // logic transforming every vertex in order
    std::vector<Vec2> Mat3::transformPoints(const std::vector<Vec2>& points) const {
        std::vector<Vec2> out;
        out.reserve(points.size());
        for (const Vec2& p : points) out.push_back(transformPoint(p));
        return out;
    }
}
