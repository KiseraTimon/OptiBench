#pragma once

#include <vector>
#include "Vec2.h"
#include "Color.h"

namespace ob {
    class Framebuffer;
    struct Polygon{
        //Vectors are ordered around the polygon's Boundary
        std::vector<Vec2> vertices;
        
        //Return no. of vertices
        std::size_t vertex_count() const { return vertices.size(); }

        //Return signed area of the polygon
        double signed_area() const;

        //Return absolute area of the polygon
        double area() const;

        //Return total length of the polygon's boundary
        double perimeter() const;

        //Return the centroid of the polygon
        Vec2 centroid() const;

        //Return true if thepoint is inside the polygon
        bool contains_point(const Vec2& point) const;

        //Draw the polygon on the given framebuffer with the specified color
        void draw(Framebuffer& framebuffer, const Color& color) const;
    };
}