#pragma once
#include <array>
#include <cstddef>

#include "Color.h"
#include "Framebuffer.h"

namespace ob {
    /**
     * Double buffering: two framebuffers of the same size.
     *
     *      back()   the hidden buffer the next frame is drawn into
     *      front()  the finished frame, the only one that is ever displayed or saved
     *      swap()   exchanges the roles in O(1): no pixels are copied, an index is flipped
     *
     * Because the viewer only ever sees front(), a half-drawn frame (scene cleared, grid drawn,
     * beams not traced yet) never reaches the screen or a saved image: no flicker, no tearing.
     */
    class DoubleBuffer {
    public:
        DoubleBuffer(int width, int height, Color background = colors::BenchDark)
            : buffers_{Framebuffer(width, height, background), Framebuffer(width, height, background)} {}

        Framebuffer& back() { return buffers_[1 - front_]; }
        const Framebuffer& back() const { return buffers_[1 - front_]; }
        const Framebuffer& front() const { return buffers_[front_]; }

        // the finished back buffer becomes the front buffer and vice versa
        void swap() noexcept {
            front_ = 1 - front_;
            ++swaps_;
        }

        int width() const { return buffers_[0].width(); }
        int height() const { return buffers_[0].height(); }
        std::size_t swapCount() const { return swaps_; }

    private:
        std::array<Framebuffer, 2> buffers_;
        int front_ = 0;
        std::size_t swaps_ = 0;
    };
}
