#pragma once
#include "Color.h"

namespace ob {
    // what a surface does to light that hits it
    enum class SurfaceType {
        Mirror,     // bounces the beam (law of reflection)
        Glass,      // bends the beam (Snell's law), or bounces it (total internal reflection)
        Screen,     // absorbs the beam; the beam path ends here
        Emitter     // a light source; beams pass through it
    };

    // optical + visual properties of an element
    struct Material {
        SurfaceType type = SurfaceType::Glass;
        double refractiveIndex = 1.0;           // only meaningful for Glass
        Color fill = colors::Grey;              // body colour
        Color outline = colors::White;          // edge colour

        static Material mirror() { return {SurfaceType::Mirror, 1.0, Color{185, 190, 200}, colors::White}; }
        static Material glass(double n) { return {SurfaceType::Glass, n, Color{25, 55, 110}, colors::Cyan}; }
        static Material screen() { return {SurfaceType::Screen, 1.0, Color{60, 60, 70}, colors::Yellow}; }
        static Material emitter() { return {SurfaceType::Emitter, 1.0, Color{120, 40, 45}, colors::Grey}; }
    };
}
