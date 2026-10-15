#include "optibench/Polygon.h"

#include <cmath>
#include <cstddef>

namespace ob {
    //Validate that the polygon has enough vertices to form a valid shape
    double Polygon::signed_area() const {
        if (vertices.size() < 3) {
            return 0.0;
        }
        //Determine no.of vertices of the polygon
        double sum = 0.0;
        const std::size_t n = vertices.size();

        for (std::size_t i = 0; i < n; ++i) {
            const std::size_t nextIndex = (i + 1) % n;
            //Shoelace formula for signed area
            sum += vertices[i].cross(vertices[nextIndex]);
        }

        return sum / 2.0;
    }
    //Return the absolute area of the polygon as a signed area would produce negative values for clockwise-ordered vertices
    double Polygon::area() const {
        return std::abs(signed_area());
    }

    //Calculate the perimeter of the polygon
    double Polygon::perimeter() const {
        //Validate that the polygon has enough vertices to form a valid shape, doesn't have to be closed
        if (vertices.size() < 2) {
            return 0.0;
        }

        double total = 0.0;
        const std::size_t n = vertices.size();

        for (std::size_t i = 0; i < n; ++i) {
            const std::size_t nextIndex = (i + 1) % n;

            const Vec2 edge = vertices[nextIndex] - vertices[i];
            total += edge.length();
        }

        return total;
    }  
      // Return the number of vertices in the polygon
    std::size_t Polygon::vertex_count() const {
        return vertices.size();
    }

}