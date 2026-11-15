#include "optibench/Polygon.h"

#include <cmath>
#include <cstddef>

#include "optibench/Line.h"                     // drawLine
#include "optibench/geometry/PolygonOps.h"      // centroid, contains

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

    /*
     * Declared in Polygon.h but previously never defined (any call failed to link).
     * Implemented with the geometry module, which accepts this struct through its vertices.
     */

    //Area centroid; degenerate polygons fall back to the centre of their outline (see geom::centroid)
    Vec2 Polygon::centroid() const {
        return geom::centroid(vertices);
    }

    //Inside or on the boundary (winding-number rule: correct for concave polygons)
    bool Polygon::contains_point(const Vec2& point) const {
        return geom::contains(vertices, point);
    }

    //Outline only: every edge, closing edge included, with the line module's Bresenham drawLine
    void Polygon::draw(Framebuffer& framebuffer, const Color& color) const {
        const std::size_t n = vertices.size();
        if (n == 1) drawLine(framebuffer, vertices[0], vertices[0], color);
        if (n < 2) return;
        for (std::size_t i = 0; i < n; ++i) {
            if (n == 2 && i == 1) break;        // a 2-point polygon is a single segment
            drawLine(framebuffer, vertices[i], vertices[(i + 1) % n], color);
        }
    }
}
