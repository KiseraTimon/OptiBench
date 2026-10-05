#pragma once
#include <cstdint>

namespace ob {

    // data components of a color
    struct Color {
        std::uint8_t r, g, b;
    };

    // basic colors
    namespace colors {
        constexpr Color Black{0,0,0};
        constexpr Color White{255, 255, 255};
        constexpr Color Red{255, 60, 60};
        constexpr Color Green{60, 255, 90};
        constexpr Color Blue{70, 120, 255};
        constexpr Color Yellow{255, 220, 60};
        constexpr Color Cyan{60, 230, 255};
        constexpr Color Grey{70, 70, 80};

        // background color for the optical bench
        constexpr Color BenchDark{16, 18, 28};
    }
}