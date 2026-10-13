#include "optibench/clip_fill.h"
#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <stack>
#include <utility>

namespace ob {
    namespace {
        /**
         * helper method comparing two colours channel by channel
         * @param a Color
         * @param b Color
         * @return bool
         */
        bool sameColor(Color a, Color b) {
            return a.r == b.r && a.g == b.g && a.b == b.b;
        }


        /**
         * helper method to draw a line between two pixels (Bresenham)
         *
         * TODO: replace with the lines module once it is merged
         *
         * @param fb Framebuffer
         * @param x0 int
         * @param y0 int
         * @param x1 int
         * @param y1 int
         * @param c Color
         */
        void bresenham(Framebuffer& fb, int x0, int y0, int x1, int y1, Color c) {
            const int dx = std::abs(x1 - x0), sx = x0 < x1 ? 1 : -1;
            const int dy = -std::abs(y1 - y0), sy = y0 < y1 ? 1 : -1;
            int err = dx + dy;

            while (true) {
                fb.setPixel(x0, y0, c);
                if (x0 == x1 && y0 == y1) break;

                const int e2 = 2 * err;

                // stepping along x
                if (e2 >= dy) { err += dy; x0 += sx; }

                // stepping along y
                if (e2 <= dx) { err += dx; y0 += sy; }
            }
        }


        /**
         * data component of an edge in the scanline edge table
         */
        struct Edge {
            int yEnd;           // first scanline the edge no longer covers (exclusive)
            double x;           // x where the edge crosses the current scanline
            double invSlope;    // change in x per scanline (1/m)
        };
    }


    /**
     * method to draw the outline of a polygon
     * @param fb Framebuffer
     * @param poly Polygon
     * @param c Color
     */
    void drawPolygon(Framebuffer& fb, const Polygon& poly, Color c) {
        const std::size_t n = poly.size();
        for (std::size_t i = 0; i < n; ++i) {
            const Vec2& a = poly[i];
            const Vec2& b = poly[(i + 1) % n];
            bresenham(fb,
                      static_cast<int>(std::lround(a.x)), static_cast<int>(std::lround(a.y)),
                      static_cast<int>(std::lround(b.x)), static_cast<int>(std::lround(b.y)), c);
        }
    }


    /**
     * method to fill a polygon using the scanline algorithm
     *
     * pixels are sampled at their centres (x + 0.5, y + 0.5); each edge covers
     * the half-open range [yMin, yMax) so a shared vertex is never counted twice
     *
     * @param fb Framebuffer
     * @param poly Polygon
     * @param c Color
     */
    void fillPolygon(Framebuffer& fb, const Polygon& poly, Color c) {
        if (poly.size() < 3) return;

        // finding the range of scanlines the polygon covers, clamped to the canvas
        double minY = poly[0].y, maxY = poly[0].y;
        for (const Vec2& v : poly) { minY = std::min(minY, v.y); maxY = std::max(maxY, v.y); }

        const int yFirst = std::max(0, static_cast<int>(std::ceil(minY - 0.5)));
        const int yLast = std::min(fb.height(), static_cast<int>(std::ceil(maxY - 0.5)));  // exclusive
        if (yFirst >= yLast) return;

        // building the edge table: edges bucketed by the first scanline they cross
        std::vector<std::vector<Edge>> edgeTable(static_cast<std::size_t>(yLast - yFirst));
        const std::size_t n = poly.size();
        for (std::size_t i = 0; i < n; ++i) {
            Vec2 a = poly[i], b = poly[(i + 1) % n];

            // horizontal edges never cross a scanline, so they are skipped
            if (a.y == b.y) continue;

            // orienting every edge top -> bottom
            if (a.y > b.y) std::swap(a, b);

            const int yStart = static_cast<int>(std::ceil(a.y - 0.5));
            const int yEnd = static_cast<int>(std::ceil(b.y - 0.5));
            if (yStart >= yEnd || yEnd <= yFirst || yStart >= yLast) continue;

            const double invSlope = (b.x - a.x) / (b.y - a.y);

            // edges starting above the canvas join at the first visible scanline
            const int yEntry = std::max(yStart, yFirst);
            const double x = a.x + (yEntry + 0.5 - a.y) * invSlope;

            edgeTable[static_cast<std::size_t>(yEntry - yFirst)].push_back({yEnd, x, invSlope});
        }

        // walking the scanlines with an active edge table
        std::vector<Edge> active;
        std::vector<double> xs;
        for (int y = yFirst; y < yLast; ++y) {
            // adding edges that start on this scanline
            const auto& starting = edgeTable[static_cast<std::size_t>(y - yFirst)];
            active.insert(active.end(), starting.begin(), starting.end());

            // removing edges that have ended
            active.erase(std::remove_if(active.begin(), active.end(),
                                        [y](const Edge& e) { return e.yEnd <= y; }),
                         active.end());

            // sorting crossings left -> right
            xs.clear();
            for (const Edge& e : active) xs.push_back(e.x);
            std::sort(xs.begin(), xs.end());

            // even-odd rule: filling between pairs of crossings
            for (std::size_t i = 0; i + 1 < xs.size(); i += 2) {
                const int xStart = std::max(0, static_cast<int>(std::ceil(xs[i] - 0.5)));
                const int xEnd = std::min(fb.width(), static_cast<int>(std::ceil(xs[i + 1] - 0.5)));
                for (int x = xStart; x < xEnd; ++x) fb.setPixel(x, y, c);
            }

            // stepping every active edge down to the next scanline
            for (Edge& e : active) e.x += e.invSlope;
        }
    }


    /**
     * method to repaint a 4-connected region of same-coloured pixels
     *
     * uses an explicit stack rather than recursion so large regions do not
     * overflow the call stack
     *
     * @param fb Framebuffer
     * @param x int
     * @param y int
     * @param fill Color
     */
    void floodFill(Framebuffer& fb, int x, int y, Color fill) {
        // bounds are checked here because getPixel reports out-of-bounds as Black
        auto inBounds = [&](int px, int py) { return px >= 0 && py >= 0 && px < fb.width() && py < fb.height(); };
        if (!inBounds(x, y)) return;

        const Color target = fb.getPixel(x, y);

        // nothing to do; also prevents an endless loop
        if (sameColor(target, fill)) return;

        std::stack<std::pair<int, int>> pending;
        pending.push({x, y});

        while (!pending.empty()) {
            const auto [px, py] = pending.top();
            pending.pop();

            if (!inBounds(px, py) || !sameColor(fb.getPixel(px, py), target)) continue;

            fb.setPixel(px, py, fill);

            // visiting the 4 neighbours
            pending.push({px + 1, py});
            pending.push({px - 1, py});
            pending.push({px, py + 1});
            pending.push({px, py - 1});
        }
    }


    /**
     * method to paint outward from a seed until the boundary colour is reached
     * @param fb Framebuffer
     * @param x int
     * @param y int
     * @param fill Color
     * @param boundary Color
     */
    void boundaryFill(Framebuffer& fb, int x, int y, Color fill, Color boundary) {
        auto inBounds = [&](int px, int py) { return px >= 0 && py >= 0 && px < fb.width() && py < fb.height(); };

        std::stack<std::pair<int, int>> pending;
        pending.push({x, y});

        while (!pending.empty()) {
            const auto [px, py] = pending.top();
            pending.pop();

            if (!inBounds(px, py)) continue;

            // stopping at the boundary and at pixels already painted
            const Color p = fb.getPixel(px, py);
            if (sameColor(p, boundary) || sameColor(p, fill)) continue;

            fb.setPixel(px, py, fill);

            // visiting the 4 neighbours
            pending.push({px + 1, py});
            pending.push({px - 1, py});
            pending.push({px, py + 1});
            pending.push({px, py - 1});
        }
    }
}
