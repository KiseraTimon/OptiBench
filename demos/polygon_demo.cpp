
#include "optibench/Polygon.h"
#include "optibench/Framebuffer.h"
#include "optibench/Color.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <iomanip>
#include <iostream>

namespace {

// Convert a point in polygon coordinates into image pixel coordinates.
struct Transform {
    double originX;
    double originY;
    double scale;

    int screenX(double x) const {
        return static_cast<int>(std::lround(originX + x * scale));
    }

    int screenY(double y) const {
        // Flip y because framebuffer coordinates increase downward.
        return static_cast<int>(std::lround(originY - y * scale));
    }
};

// Draw a line between two pixel coordinates using Bresenham's algorithm.
void drawLine(ob::Framebuffer& fb,
              int x0, int y0,
              int x1, int y1,
              ob::Color color) {
    const int dx = std::abs(x1 - x0);
    const int dy = std::abs(y1 - y0);

    const int sx = (x0 < x1) ? 1 : -1;
    const int sy = (y0 < y1) ? 1 : -1;

    int error = dx - dy;

    while (true) {
        fb.setPixel(x0, y0, color);

        if (x0 == x1 && y0 == y1) {
            break;
        }

        const int twiceError = 2 * error;

        if (twiceError > -dy) {
            error -= dy;
            x0 += sx;
        }

        if (twiceError < dx) {
            error += dx;
            y0 += sy;
        }
    }
}

// Draw a small filled circle to highlight a polygon vertex.
void drawVertex(ob::Framebuffer& fb,
                int cx, int cy,
                int radius,
                ob::Color color) {
    for (int dy = -radius; dy <= radius; ++dy) {
        for (int dx = -radius; dx <= radius; ++dx) {
            if (dx * dx + dy * dy <= radius * radius) {
                fb.setPixel(cx + dx, cy + dy, color);
            }
        }
    }
}

// Draw every polygon edge, including the closing edge.
void drawPolygon(ob::Framebuffer& fb,
                 const ob::Polygon& polygon,
                 const Transform& transform,
                 ob::Color outline,
                 ob::Color vertexColor) {
    const std::size_t n = polygon.vertex_count();

    if (n == 0) {
        return;
    }

    // Connect each vertex to the next one.
    // The modulo operator connects the last vertex back to the first.
    for (std::size_t i = 0; i < n; ++i) {
        const std::size_t next = (i + 1) % n;

        const auto& a = polygon.vertices[i];
        const auto& b = polygon.vertices[next];

        drawLine(
            fb,
            transform.screenX(a.x),
            transform.screenY(a.y),
            transform.screenX(b.x),
            transform.screenY(b.y),
            outline
        );
    }

    // Highlight all vertices after drawing the edges.
    for (const auto& vertex : polygon.vertices) {
        drawVertex(
            fb,
            transform.screenX(vertex.x),
            transform.screenY(vertex.y),
            5,
            vertexColor
        );
    }
}

// Draw a subtle grid over the image.
void drawGrid(ob::Framebuffer& fb, int spacing) {
    const ob::Color gridColor{32, 36, 50};

    for (int x = 0; x < fb.width(); x += spacing) {
        drawLine(fb, x, 0, x, fb.height() - 1, gridColor);
    }

    for (int y = 0; y < fb.height(); y += spacing) {
        drawLine(fb, 0, y, fb.width() - 1, y, gridColor);
    }
}

} // namespace

int main() {
    using namespace ob;

    // Create two polygons using model-space coordinates.
    Polygon rectangle;
    rectangle.vertices = {
        {1, 1},
        {5, 1},
        {5, 4},
        {1, 4}
    };

    Polygon triangle;
    triangle.vertices = {
        {0, 0},
        {4, 0},
        {0, 3}
    };

    // Create the output image.
    Framebuffer fb(800, 500, colors::BenchDark);

    // Background grid.
    drawGrid(fb, 25);

    // Map the polygon coordinates to different areas of the image.
    const Transform rectangleTransform{105, 370, 45};
    const Transform triangleTransform{455, 370, 45};

    // Draw the rectangle in cyan.
    drawPolygon(
        fb,
        rectangle,
        rectangleTransform,
        colors::Cyan,
        colors::White
    );

    // Draw the triangle in yellow.
    drawPolygon(
        fb,
        triangle,
        triangleTransform,
        colors::Yellow,
        colors::White
    );

    // Save the rendered image.
    if (!fb.saveBMP("output/polygon_demo.bmp")) {
        std::puts("FAILED to write output/polygon_demo.bmp");
        return 1;
    }

    // Print the geometric results.
    std::cout << std::fixed << std::setprecision(2);

    std::cout << "\n====================================\n";
    std::cout << "       OPTIBENCH POLYGON DEMO\n";
    std::cout << "====================================\n";

    std::cout << "\nRECTANGLE (cyan)\n";
    std::cout << "Vertices:   " << rectangle.vertex_count() << '\n';
    std::cout << "Signed area: " << rectangle.signed_area() << '\n';
    std::cout << "Area:        " << rectangle.area() << '\n';
    std::cout << "Perimeter:   " << rectangle.perimeter() << '\n';
    std::cout << "Expected area: 4 x 3 = 12\n";
    std::cout << "Expected perimeter: 2 x (4 + 3) = 14\n";

    std::cout << "\nTRIANGLE (yellow)\n";
    std::cout << "Vertices:   " << triangle.vertex_count() << '\n';
    std::cout << "Signed area: " << triangle.signed_area() << '\n';
    std::cout << "Area:        " << triangle.area() << '\n';
    std::cout << "Perimeter:   " << triangle.perimeter() << '\n';
    std::cout << "Expected area: (4 x 3) / 2 = 6\n";
    std::cout << "Expected perimeter: 4 + 3 + 5 = 12\n";

    std::cout << "\nImage saved to output/polygon_demo.bmp\n";

    return 0;
}
