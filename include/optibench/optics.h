#pragma once
#include "Vec2.h"

namespace ob {
    /**
     * Light-bending maths.
     *
     * STARTER VERSION: written so the tracer runs end to end today.
     * OWNER: Sean. Review it, extend it (tests, edge cases, wavelength-dependent n for dispersion).
     */

    // law of reflection: returns d mirrored about the surface whose unit normal is n  (r = d - 2(d.n)n)
    Vec2 reflect(const Vec2& d, const Vec2& n);

    /**
     * Snell's law:  n1 sin(theta1) = n2 sin(theta2)
     *
     * @param d unit direction of the incoming ray
     * @param n unit normal facing AGAINST the ray (d.n < 0)
     * @param n1 refractive index the ray is leaving
     * @param n2 refractive index the ray is entering
     * @param out unit direction of the refracted ray (only written on success)
     * @return false on total internal reflection (no refracted ray exists)
     */
    bool refract(const Vec2& d, const Vec2& n, double n1, double n2, Vec2& out);
}
