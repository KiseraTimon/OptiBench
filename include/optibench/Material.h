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
        double refractiveIndex = 1.0;           // only meaningful for Glass: n_d, the index at kDesignWavelength
        Color fill = colors::Grey;              // body colour
        Color outline = colors::White;          // edge colour
        double abbeNumber = 0.0;                // Glass dispersion V_d = (n_d - 1) / (n_F - n_C); 0 = no dispersion

        // Fraunhofer lines used by glass catalogues (nanometres)
        static constexpr double kDesignWavelength = 587.56;    // d (helium), where refractiveIndex applies
        static constexpr double kBlueF = 486.13;               // F (hydrogen)
        static constexpr double kRedC = 656.27;                // C (hydrogen)

        /**
         * refractive index at a wavelength (nm), from Cauchy's equation n(L) = A + B / L^2 fitted to the
         * catalogue pair (n_d, V_d): B = (n_d - 1) / V_d / (1/F^2 - 1/C^2), and n(d) = n_d exactly.
         * Blue light sees a higher index than red, so it bends more: that is what splits white light.
         * Without dispersion (abbeNumber <= 0) every wavelength sees refractiveIndex.
         */
        constexpr double indexAt(double wavelengthNm) const {
            if (abbeNumber <= 0.0 || wavelengthNm <= 0.0) return refractiveIndex;
            const double um = wavelengthNm / 1000.0, d = kDesignWavelength / 1000.0;
            const double f = kBlueF / 1000.0, c = kRedC / 1000.0;
            const double b = (refractiveIndex - 1.0) / abbeNumber / (1.0 / (f * f) - 1.0 / (c * c));
            return refractiveIndex + b * (1.0 / (um * um) - 1.0 / (d * d));
        }

        static Material mirror() { return {SurfaceType::Mirror, 1.0, Color{185, 190, 200}, colors::White}; }
        static Material glass(double n) { return {SurfaceType::Glass, n, Color{25, 55, 110}, colors::Cyan}; }
        static Material glass(double n, double abbe) {
            Material m = glass(n);
            m.abbeNumber = abbe;
            return m;
        }
        static Material screen() { return {SurfaceType::Screen, 1.0, Color{60, 60, 70}, colors::Yellow}; }
        static Material emitter() { return {SurfaceType::Emitter, 1.0, Color{120, 40, 45}, colors::Grey}; }

        // catalogue glasses (Schott data sheets: n_d, V_d)
        static Material crownBK7() { return glass(1.5168, 64.17); }        // the everyday lens glass
        static Material flintF2() { return glass(1.6200, 36.37); }
        static Material denseFlintSF11() { return glass(1.7847, 25.68); }
        static Material denseFlintSF66() { return glass(1.9229, 20.88); }  // one of the most dispersive glasses
    };
}
