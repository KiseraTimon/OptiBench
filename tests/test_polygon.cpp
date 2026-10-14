#include <cassert>
#include <cmath>
#include <iostream>

#include "optibench/Polygon.h"

int main() {
    ob::Polygon rectangle;
    rectangle.vertices = {
        {1, 1},
        {5, 1},
        {5, 4},
        {1, 4}
    };


    // Test signed area
    assert(std::abs(rectangle.signed_area() - 12.0) < 1e-9);

    // Test ordinary area
    assert(std::abs(rectangle.area() - 12.0) < 1e-9);

    // Test perimeter
    assert(std::abs(rectangle.perimeter() - 14.0) < 1e-9);

        // Edge case 1: Empty polygon
    ob::Polygon empty;

    assert(empty.signed_area() == 0.0);
    assert(empty.area() == 0.0);
    assert(empty.perimeter() == 0.0);

    // Edge case 2: Polygon with one vertex
    ob::Polygon singlePoint;
    singlePoint.vertices = {
        {2, 3}
    };

    assert(singlePoint.signed_area() == 0.0);
    assert(singlePoint.area() == 0.0);
    assert(singlePoint.perimeter() == 0.0);

    // Edge case 3: Triangle
    ob::Polygon triangle;
    triangle.vertices = {
        {0, 0},
        {4, 0},
        {0, 3}
    };

    assert(std::abs(triangle.signed_area() - 6.0) < 1e-9);
    assert(std::abs(triangle.area() - 6.0) < 1e-9);
    assert(std::abs(triangle.perimeter() - 12.0) < 1e-9);

    std::cout << "All polygon tests passed!\n";
    return 0;
}