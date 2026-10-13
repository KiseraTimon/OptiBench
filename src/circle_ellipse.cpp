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
            fb.setPixel(cx + y, cy + x, c); fb.setPixel(cx - y, cy - x, c);
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
    void drawCircle(Framebuffer &fb, int cx, int cy, int r, Color c) {
        // safety check preventing a negative radius
        if (r < 0) return;

        // initializing drawing from the top center of a circle
        int x = 0, y = r;

        // decision param; validates movement of the pen downwards
        int p = 1 - r;

        // drawing to the 45deg mark; an 1/8th of the circle
        while (x <= y) {
            // drawing the 8 mirrored dots
            plot8(fb, cx, cy, x, y, c );

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
    void fillCircle(Framebuffer &fb, int cx, int cy, int r, Color c) {
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


    void drawEllipse(Framebuffer &fb, int cx, int cy, int rx, int ry, Color c) {
        if (rx <= 0 || ry <= 0) return;

        const double rx2 = static_cast<double>(rx) * rx;
        const double ry2 = static_cast<double>(ry) * ry;

        int x = 0, y = ry;
        double px = 0.0;
        double py = 2.0 * rx2 * y;
        double p = ry2 -rx2 + ry + 0.25 * rx2;

        while (px < py) {
            plot4(fb, cx, cy, x, y, c);

            ++x;

            if (p < 0) {
                p += ry2 + px;
            }

            else {
                --y;
                py -= 2.0 * rx2;
                p += ry2 + px -py;
            }
        }
    }
}

