#include <cmath>
#include <cstdio>

#include "optibench/Mat3.h"
#include "optibench/Transform2D.h"
#include "optibench/clip_fill.h"    // Polygon (= std::vector<Vec2>)
#include "optibench/optics.h"

#define CHECK(cond) do { if (!(cond)) { std::printf("FAIL %s:%d  %s\n", __FILE__, __LINE__, #cond); return 1; } } while (0)

using namespace ob;

static bool near(double a, double b, double eps = 1e-9) { return std::fabs(a - b) < eps; }
static bool near(Vec2 a, Vec2 b, double eps = 1e-9) { return near(a.x, b.x, eps) && near(a.y, b.y, eps); }

// checks every entry of two matrices agrees
static bool near(const Mat3& a, const Mat3& b, double eps = 1e-9) {
    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j)
            if (!near(a.m[i][j], b.m[i][j], eps)) return false;
    return true;
}

static double deg(double d) { return d * kPi / 180.0; }

int main() {
    // ---------- identity and matrix multiplication ----------
    {
        CHECK(near(Mat3::identity().transformPoint({7, -3}), {7, -3}));

        // hand-multiplied: row i of A times column j of B
        const Mat3 a{{{1, 2, 3}, {4, 5, 6}, {7, 8, 9}}};
        const Mat3 b{{{9, 8, 7}, {6, 5, 4}, {3, 2, 1}}};
        CHECK(near(a * b, Mat3{{{30, 24, 18}, {84, 69, 54}, {138, 114, 90}}}));

        // identity is neutral on both sides
        CHECK(near(a * Mat3::identity(), a) && near(Mat3::identity() * a, a));
    }

    // ---------- translate: points move, directions do not ----------
    {
        const Mat3 t = Mat3::translation(4, 1);
        CHECK(near(t.transformPoint({2, 3}), {6, 4}));
        CHECK(near(t.transformDirection({2, 3}), {2, 3}));
    }

    // ---------- scale about the origin ----------
    {
        const Mat3 s = Mat3::scaling(2, 3);
        CHECK(near(s.transformPoint({2, 3}), {4, 9}));
        CHECK(near(s.determinant(), 6.0));                          // areas grow 2 x 3
    }

    // ---------- shear ----------
    {
        // x' = x + 2y:  (1, 3) -> (7, 3)
        CHECK(near(Mat3::shearX(2).transformPoint({1, 3}), {7, 3}));
        // y' = y + 0.5x:  (4, 2) -> (4, 4)
        CHECK(near(Mat3::shearY(0.5).transformPoint({4, 2}), {4, 4}));
        CHECK(near(Mat3::shearX(2).determinant(), 1.0));            // shear keeps area
    }

    // ---------- rotate about the origin ----------
    {
        // lecture example: (2,1) rotated 90 degrees -> (-1,2)
        CHECK(near(Mat3::rotation(deg(90)).transformPoint({2, 1}), {-1, 2}));
        // (1,0) rotated 30 degrees -> (cos 30, sin 30) = (0.8660254..., 0.5)
        CHECK(near(Mat3::rotation(deg(30)).transformPoint({1, 0}), {std::sqrt(3.0) / 2.0, 0.5}));
        CHECK(near(Mat3::rotation(1.234).determinant(), 1.0));      // rotation keeps area

        // same sense as the scene's Transform2D (positive angle = clockwise on a y-down screen)
        CHECK(near(Mat3::rotation(0.7).transformDirection({3, -1}), Transform2D::rotateBy({3, -1}, 0.7)));
    }

    // ---------- rotate about an arbitrary pivot ----------
    {
        // (2,1) about pivot (1,1) by 90:  -(1,1) -> (1,0),  rotate -> (0,1),  +(1,1) -> (1,2)
        const Mat3 r = Mat3::rotationAbout(deg(90), {1, 1});
        CHECK(near(r.transformPoint({2, 1}), {1, 2}));
        CHECK(near(r.transformPoint({1, 1}), {1, 1}));              // the pivot does not move

        // half turn about (3,2): (5,2) -> (1,2)
        CHECK(near(Mat3::rotationAbout(kPi, {3, 2}).transformPoint({5, 2}), {1, 2}));

        // it is exactly T(pivot) . R . T(-pivot), and differs from rotating about the origin
        CHECK(near(r, Mat3::translation(1, 1) * Mat3::rotation(deg(90)) * Mat3::translation(-1, -1)));
        CHECK(!near(r.transformPoint({2, 1}), Mat3::rotation(deg(90)).transformPoint({2, 1})));
    }

    // ---------- reflect across the axes ----------
    {
        CHECK(near(Mat3::reflectX().transformPoint({3, 4}), {3, -4}));
        CHECK(near(Mat3::reflectY().transformPoint({3, 4}), {-3, 4}));
        CHECK(near(Mat3::reflectOrigin().transformPoint({3, 4}), {-3, -4}));
        CHECK(near(Mat3::reflectX().determinant(), -1.0));          // mirrors flip winding
    }

    // ---------- reflect across an arbitrary line ----------
    {
        // across y = x: (3,1) -> (1,3)
        const Mat3 diag = Mat3::reflectLine({0, 0}, {1, 1});
        CHECK(near(diag.transformPoint({3, 1}), {1, 3}));

        // across the horizontal line y = 2: (3,5) -> (3,-1)
        CHECK(near(Mat3::reflectLine({0, 2}, {5, 2}).transformPoint({3, 5}), {3, -1}));

        // across y = x + 1 (does not pass through the origin): (x,y) -> (y-1, x+1), so (2,0) -> (-1,3)
        const Mat3 off = Mat3::reflectLine({0, 1}, {2, 3});
        CHECK(near(off.transformPoint({2, 0}), {-1, 3}));
        CHECK(near(off.transformPoint({4, 5}), {4, 5}));            // points on the line stay put
        CHECK(near(off * off, Mat3::identity()));                   // reflecting twice undoes it
        CHECK(near(off.determinant(), -1.0));

        // the axis reflections are special cases of it
        CHECK(near(Mat3::reflectLine({-1, 0}, {1, 0}), Mat3::reflectX()));
        CHECK(near(Mat3::reflectLine({0, 5}, {0, -5}), Mat3::reflectY()));

        // consistent with optics.h reflect(): a mirror line bounces directions identically
        const Vec2 a{10, 4}, b{-2, 13};
        const Vec2 n = (b - a).normalized().perp();
        const Vec2 d = Vec2{0.6, 0.8};
        CHECK(near(Mat3::reflectLine(a, b).transformDirection(d), reflect(d, n)));

        // a == b gives no line: identity rather than NaN
        CHECK(near(Mat3::reflectLine({3, 3}, {3, 3}), Mat3::identity()));
    }

    // ---------- composite transforms: T . R . S order ----------
    {
        // lecture example: (2,3) scaled by 2 then translated by (4,1) -> (8,7)
        CHECK(near(Mat3::composite({4, 1}, 0.0, {2, 2}).transformPoint({2, 3}), {8, 7}));

        // (1,0):  S(2,1) -> (2,0),  R(90) -> (0,2),  T(10,0) -> (10,2)
        const Mat3 trs = Mat3::composite({10, 0}, deg(90), {2, 1});
        CHECK(near(trs.transformPoint({1, 0}), {10, 2}));
        CHECK(near(trs, Mat3::translation(10, 0) * Mat3::rotation(deg(90)) * Mat3::scaling(2, 1)));

        // order matters: S . R . T sends (1,0) -> (11,0) -> (0,11) -> (0,11), not (10,2)
        const Mat3 srt = Mat3::scaling(2, 1) * Mat3::rotation(deg(90)) * Mat3::translation(10, 0);
        CHECK(near(srt.transformPoint({1, 0}), {0, 11}));
        CHECK(!near(srt, trs));

        // rotation about a pivot composes with the rest like any other matrix
        const Mat3 chain = Mat3::translation(5, 0) * Mat3::rotationAbout(deg(90), {1, 1});
        CHECK(near(chain.transformPoint({2, 1}), {6, 2}));
    }

    // ---------- cross-check against the scene's Transform2D ----------
    {
        const Transform2D t{.position = {33, -7}, .rotation = 0.9, .scale = {2.5, 0.4}};
        const Mat3 m = Mat3::fromTransform(t);
        for (const Vec2 p : {Vec2{3.2, -8.1}, Vec2{0, 0}, Vec2{-14, 6.5}}) {
            CHECK(near(m.transformPoint(p), t.toWorld(p)));
            CHECK(near(m.transformDirection(p), t.dirToWorld(p)));
            if (p.length() > 0.0) CHECK(near(m.transformNormal(p), t.normalToWorld(p)));
        }

        // the inverse matrix is Transform2D::toLocal
        const auto inv = m.inverse();
        CHECK(inv.has_value());
        CHECK(near(inv->transformPoint({100, 50}), t.toLocal({100, 50})));
    }

    // ---------- inverse ----------
    {
        CHECK(near(*Mat3::translation(3, -2).inverse(), Mat3::translation(-3, 2)));

        const Mat3 m = Mat3::composite({4, -6}, 0.3, {2, -0.5}) * Mat3::shearX(1.5);
        const auto inv = m.inverse();
        CHECK(inv.has_value());
        CHECK(near(m * *inv, Mat3::identity()) && near(*inv * m, Mat3::identity()));

        // squashing onto a line cannot be undone
        CHECK(!Mat3::scaling(0, 1).inverse().has_value());
    }

    // ---------- normals stay perpendicular under non-uniform scale ----------
    {
        // the line x + y = c has tangent (1,-1) and normal (1,1)/sqrt2;
        // scaling x by 2 makes the tangent (2,-1), so the normal must become (1,2)/sqrt5 (not (2,1))
        const Mat3 s = Mat3::scaling(2, 1);
        const Vec2 nWorld = s.transformNormal(Vec2{1, 1}.normalized());
        CHECK(near(nWorld, Vec2{1, 2}.normalized()));
        CHECK(near(nWorld.dot(s.transformDirection({1, -1})), 0.0));

        // under a mirror the normal is mirrored too (stays on the same side of the surface)
        CHECK(near(Mat3::reflectX().transformNormal({0, 1}), {0, -1}));
        CHECK(near(Mat3::scaling(0, 1).transformNormal({1, 0}), {0, 0}));   // singular: zero vector
    }

    // ---------- polygon vertices ----------
    {
        // a triangle rotated 90 degrees about its first vertex, then moved right by 10
        const Polygon tri = {{0, 0}, {4, 0}, {0, 3}};
        const Polygon out = (Mat3::translation(10, 0) * Mat3::rotationAbout(deg(90), {0, 0})).transformPoints(tri);
        CHECK(out.size() == 3);
        CHECK(near(out[0], {10, 0}) && near(out[1], {10, 4}) && near(out[2], {7, 0}));

        // area is preserved by rotation + translation, scaled by |det| otherwise
        CHECK(near(std::fabs(polygonArea(out)), std::fabs(polygonArea(tri))));
        CHECK(near(std::fabs(polygonArea(Mat3::scaling(2, 3).transformPoints(tri))), 6.0 * std::fabs(polygonArea(tri))));
        CHECK(Mat3::identity().transformPoints({}).empty());
    }

    std::puts("test_mat3: PASS");
    return 0;
}
