#include "optibench/circle_ellipse.h"
#include <cmath>

namespace ob {
    namespace {
        /**
         * helper method for plotting a circle
         * circles are symmetric 8 ways:
         *      so by calculating 1/8 of a circle, we can mirror
         *      the pixels at once around a center point (cx, cy)
         *
         * @param fb Framebuffer
         * @param cx int
         * @param cy int
         * @param x int
         * @param y int
         * @param c Color
         */
        void plot8(Framebuffer& fb, int cx, int cy, int x, int y, Color c) {
            // drawing bottom-right/bottom-left pixels
            fb.setPixel(cx + x, cy + y, c); fb.setPixel(cx - x, cy + y, c);

            // drawing top-right/top-left pixels
            fb.setPixel(cx + x, cy - y, c); fb.setPixel(cx - x, cy - y, c);

            // drawing outer bottom-right/bottom-left pixels
            fb.setPixel(cx + y, cy + x, c); fb.setPixel(cx - y, cy + x, c);

            // drawing outer top-right/top-left pixels
            fb.setPixel(cx + y, cy - x, c); fb.setPixel(cx - y, cy - x, c);
        }


        /**
         * helper method for plotting an ellipse
         * ellipses are symmetric 4 ways (top/down; left/right)
         *
         * @param fb Framebuffer
         * @param cx int
         * @param cy int
         * @param x int
         * @param y int
         * @param c Color
         */
        void plot4(Framebuffer& fb, int cx, int cy, int x, int y, Color c) {
            // drawing the bottom-right/bottom-left pixels
            fb.setPixel(cx + x, cy + y, c); fb.setPixel(cx - x, cy + y, c);

            // drawing the top-right/top-left pixels
            fb.setPixel(cx + x, cy - y, c); fb.setPixel(cx - x, cy - y, c);
        }


        /**
         * helper method to draw a span:
         *      a horizontal line between two x coordinates
         *
         * @param fb Framebuffer
         * @param x0 int
         * @param x1 int
         * @param y int
         * @param c Color
         */
        void hspan(Framebuffer& fb, int x0, int x1, int y, Color c) {
            for (int x = x0; x <= x1; ++x) fb.setPixel(x, y, c);
        }
    }


    /**
     * method to draw a hollow circle outline
     * @param fb Framebuffer
     * @param cx int
     * @param cy int
     * @param r int
     * @param c Color
     */
    void drawCircle(Framebuffer& fb, int cx, int cy, int r, Color c) {
        // safety check preventing a negative radius
        if (r < 0) return;

        // initializing drawing from the top center of a circle
        int x = 0, y = r;

        // decision param; validates movement of the pen downwards
        int p = 1 - r;

        // drawing to the 45deg mark; an 1/8th of the circle
        while (x <= y) {
            // drawing the 8 mirrored dots
            plot8(fb, cx, cy, x, y, c);

            // moving the pen one pixel to the right
            ++x;

            // for negative decision values, the pen height is unchanged
            if (p < 0) {
                p += 2 * x + 1;
            }

            // otherwise, the pen drops one pixel down to match the curve of the circle
            else {
                --y;
                p += 2 * (x - y) + 1;
            }
        }
    }


    /**
     * paints the pixels bound by the circle outline for a fill effect
     * @param fb Framebuffer
     * @param cx int
     * @param cy int
     * @param r int
     * @param c Color
     */
    void fillCircle(Framebuffer& fb, int cx, int cy, int r, Color c) {
        // safety check preventing a negative radius
        if (r < 0) return;

        // initializing drawing from the top center of a circle
        int x = 0, y = r;
        int p = 1 - r;

        // looped drawing to the 45deg angle
        while (x <= y) {
            // drawing 4 solid horizontal lines to fill the space
            hspan(fb, cx - x, cx + x, cy + y, c);
            hspan(fb, cx - x, cx + x, cy - y, c);
            hspan(fb, cx - y, cx + y, cy + x, c);
            hspan(fb, cx - y, cx + y, cy - x, c);

            ++x;

            if (p < 0) {
                p += 2 * x + 1;
            }

            else {
                --y;
                p += 2 * (x - y) + 1;
            }
        }
    }


    void drawEllipse(Framebuffer& fb, int cx, int cy, int rx, int ry, Color c) {
        if (rx <= 0 || ry <= 0) return;

        const double rx2 = static_cast<double>(rx) * rx;
        const double ry2 = static_cast<double>(ry) * ry;

        int x = 0, y = ry;
        double px = 0.0;            // loop condition parameter: 2(ry^2)x
        double py = 2.0 * rx2 * y;  // loop condition parameter: 2(rx^2)y
        double p = ry2 - rx2 * ry + 0.25 * rx2;

        // region 1
        while (px < py) {
            plot4(fb, cx, cy, x, y, c);

            ++x;
            px += 2.0 * ry2;

            if (p < 0) {
                p += ry2 + px;
            }

            else {
                --y;
                py -= 2.0 * rx2;

                // p += ry2 + px -py;
                p += 2.0 + px - py + ry2;
            }
        }


        // region 2
        // p = ry2 * (x + 0.5) * (x + 0.5) + rx2 * (y - 1.0) * (y -1.0) - rx2 * ry2;
        p = ry2 * (std::pow((x + 0.5), 2)) + rx2 * (std::pow((y - 1.0), 2)) - rx2 * ry2;

        while (y >= 0) {
            plot4(fb, cx, cy, x, y, c);

            --y;
            py -= 2.0 * rx2;

            if (p > 0) {
                p += rx2 - py;
            }

            else {
                ++x;

                px += 2.0 * ry2;
                p += px - py + rx2;
            }
        }
    }


    /**
     * method describing the geometry and algebra used to determine
     * the exact point a laser beam (ray) hits a circle. It borrows
     * from the quadratic formula
     *
     * @param ray Ray
     * @param center Vec2
     * @param radius double
     * @return Hit
     */
    Hit rayCircleIntersect(const Ray& ray, const Vec2& center , double radius) {
        constexpr double eps = 1e-9;
        Hit result;

        /*
         * Quadratic formula:
         *      (-b)[+/-]sqrt((b^2)-4ac)
         *      ---
         *      2a
         */

        // calculating vector pointing from the circle's center to the ray's starting point
        const Vec2 oc = ray.origin -center;

        // the 'a' part of the quadratic formula
        const double a = ray.direction.dot(ray.direction);

        // half of 'b' in the quadratic formula
        const double h = ray.direction.dot(oc);

        // the 'c' part of the quadratic formula
        const double k = oc.dot(oc) - radius * radius;

        // the discriminant part of the quadratic formula (b^2 - 4ac)
        const double discriminant = h * h - a * k;

        // discriminant below 0 indicates the ray missed;
        // a being 0 implies the ray lacks direction
        if (discriminant < 0.0 || a == 0.0) return result;

        const double sq = std::sqrt(discriminant);

        // time/distance for first entry point into the circle (near crossing)
        const double t0 = (-h - sq) / a;

        // time/distance for first exit point out of the circle
        const double t1 = (-h + sq) / a;

        // correct hit distance
        double t;

        if (t0 > eps) t = t0;           // ray starts outside, hits near side
        else if (t1 > eps) t = t1;      // ray starts inside, hits far side
        else return result;             // circle is behind ray

        // updating Hit object with collision data
        result.hit = true;
        result.t = t;
        result.point = ray.origin + ray.direction * t;
        result.normal = circleNormal(center, result.point);

        return result;
    }


    /**
     * starts the helper function to calculate the outward facing direction (normal)
     * of the circle
     *
     * @param center Vec2
     * @param point  Vec2
     * @return  Vec2
     */
    Vec2 circleNormal(const Vec2& center, const Vec2& point) {
        return (point - center).normalized();
    }
}

