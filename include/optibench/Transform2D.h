#pragma once
#include <cmath>
#include "Vec2.h"

namespace ob {
    inline constexpr double kPi = 3.14159265358979323846;

    /**
     * Where an object lives in the world: translation, rotation and scale.
     *
     * Local -> world follows the lecture's composite order P' = T . R . S . P
     *      1. scale the local point
     *      2. rotate it
     *      3. translate it to `position`
     *
     * The object's local origin is its pivot, so rotation and scaling always
     * happen about `position`.
     *
     * Axis note: y grows DOWNWARD on screen. A positive `rotation` is
     * anticlockwise in maths axes, so on screen it LOOKS clockwise.
     */
    struct Transform2D {
        Vec2 position{0.0, 0.0};
        double rotation = 0.0;      // radians
        Vec2 scale{1.0, 1.0};       // non-uniform allowed; must not be 0

        // rotates a vector by `angle` radians about the origin
        static Vec2 rotateBy(const Vec2& v, double angle) {
            const double c = std::cos(angle), s = std::sin(angle);
            return {c * v.x - s * v.y, s * v.x + c * v.y};
        }

        // point: local space -> world space
        Vec2 toWorld(const Vec2& p) const {
            return position + rotateBy({p.x * scale.x, p.y * scale.y}, rotation);
        }

        // point: world space -> local space (the inverse of toWorld)
        Vec2 toLocal(const Vec2& p) const {
            const Vec2 q = rotateBy(p - position, -rotation);
            return {q.x / scale.x, q.y / scale.y};
        }

        // direction: local -> world (no translation, because directions do not "sit" anywhere)
        Vec2 dirToWorld(const Vec2& d) const {
            return rotateBy({d.x * scale.x, d.y * scale.y}, rotation);
        }

        // direction: world -> local
        Vec2 dirToLocal(const Vec2& d) const {
            const Vec2 q = rotateBy(d, -rotation);
            return {q.x / scale.x, q.y / scale.y};
        }

        /**
         * surface normal: local -> world.
         * Normals are NOT transformed like directions: under non-uniform scale they must
         * use the inverse-transpose, i.e. divide by the scale before rotating.
         */
        Vec2 normalToWorld(const Vec2& n) const {
            return rotateBy({n.x / scale.x, n.y / scale.y}, rotation).normalized();
        }
    };
}
