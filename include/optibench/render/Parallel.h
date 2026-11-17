#pragma once
#include <algorithm>
#include <thread>
#include <vector>

namespace ob::render {
    /**
     * runs fn(begin, end) over [0, count) split into contiguous chunks, one per hardware thread.
     * Callers make every chunk write only its own rows / columns / bands, so the result is identical
     * to a serial loop (same pixels, same order of additions per pixel).
     */
    template <class Fn>
    void parallelFor(int count, Fn fn) {
        const int threads = std::clamp(static_cast<int>(std::thread::hardware_concurrency()), 1, 16);
        if (threads == 1 || count < 64) { fn(0, count); return; }
        std::vector<std::thread> pool;
        pool.reserve(static_cast<std::size_t>(threads - 1));
        const int chunk = (count + threads - 1) / threads;
        for (int t = 1; t < threads; ++t) {
            const int b = t * chunk, e = std::min(count, b + chunk);
            if (b < e) pool.emplace_back(fn, b, e);
        }
        fn(0, std::min(count, chunk));
        for (std::thread& th : pool) th.join();
    }
}
