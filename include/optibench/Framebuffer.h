#pragma once
#include <string>
#include <vector>
#include "Color.h"

namespace ob {
    /**
     * The Framebuffer is the grid of pixels
     * we are drawing into
     *
     * Pixels coordinates:
     *      (0, 0) is the top left corner
     *      x grows to the right
     *      y grows downwards
     */

    // Framebuffer canvas
    class Framebuffer {
    public:
        // constructor
        Framebuffer(
            int width,  // canvas width in pixels
            int height, // canvas height in pixels
            Color background = colors::BenchDark    // canvas background color
        );

        // returns the width of the canvas
        int width() const { return w_; }

        // returns the height of the canvas
        int height() const { return h_; }

        // erases points by painting the whole canvas a single color
        void clear(Color c);

        // maps a point by changing the color of a specific pixel
        void setPixel(int x, int y, Color c);

        // reads a point by getting the color of a specific pixel
        Color getPixel(int x, int y) const;

        // saves the canvas as a .bmp image
        bool saveBMP(const std::string& path) const;


    private:
        // internal width & height of the canvas
        int w_, h_;

        // color data of every pixel in the canvas
        std::vector<Color> pixels_;
    };
}