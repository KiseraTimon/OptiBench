#pragma once
#include <cmath>

namespace ob {
    // data components of a 2D vector/point
    struct Vec2 {
        // axes
        double x = 0.0, y = 0.0;

        // non-argument constructor is the default
        Vec2() = default;

        // parameterized constructor for instantiation
        Vec2(double x_, double y_) : x(x_), y(y_) {}

        /*
         * Overloaded operators
         */
        // adding vectors
        Vec2 operator+(const Vec2& o) const { return {x + o.x, y + o.y}; }

        // subtracting vectors
        Vec2 operator-(const Vec2& o) const { return {x - o.x, y - o.y}; }

        // negating vectors (unary)
        Vec2 operator-() const { return {-x, -y}; }

        // multiplying vectors
        Vec2 operator*(double s) const { return {x * s, y * s}; }

        // dividing vectors
        Vec2 operator/(double s) const { return {x / s, y / s}; }

        // calculating dot product
        double dot(const Vec2& o) const { return x * o.x + y * o.y; }

        // calculating 2D cross product scalar
        double cross(const Vec2& o) const { return x * o.y - y * o.x; }

        // calculating magnitude/length of the vector
        double length() const { return std::sqrt(x * x + y * y); }

        // producing a new, aligned, unit vector
        Vec2 normalized() const {
            const double l = length();
            return l > 0.0 ? Vec2{x / l, y / l} : Vec2{};
        }

        // producing a new, perpendicular vector
        Vec2 perp() const { return {-y, x}; }
    };

    // inline overload for vector multiplications
    inline Vec2 operator*(double s, const Vec2& v) { return v * s; }
}