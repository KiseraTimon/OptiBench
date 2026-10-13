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
    }
}

