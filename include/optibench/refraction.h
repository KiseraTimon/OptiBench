#pragma once
#include <optional>

#include "Line.h"           // Segment, intersectRaySegment
#include "Ray.h"
#include "Vec2.h"

namespace ob {
    /**
     * Snell's-law refraction with total internal reflection (TIR).
     *
     * Built on the existing optics primitives instead of re-deriving them:
     *      - optics.h  refract()              Snell's law solver (reports when no refracted ray exists)
     *      - optics.h  reflect()              the TIR fallback
     *      - Line.h    intersectRaySegment()  where a ray meets a flat interface
     *
     * Angles are in radians, measured from the surface normal.
     */

    // what light does at the boundary between two media
    struct Refraction {
        bool totalInternal = false;     // true: no refracted ray exists, `direction` is the reflection
        Vec2 direction;                 // unit direction of the outgoing ray
        double incidence = 0.0;         // angle between the incoming ray and the normal
        double exitAngle = 0.0;         // angle between the outgoing ray and the normal (equals incidence on TIR)
    };

    // one ray meeting one flat interface segment
    struct SurfaceInteraction {
        Hit hit;                        // from intersectRaySegment (hit.normal faces the incoming ray)
        Refraction light;               // only meaningful when hit.hit is true
        Ray outgoing;                   // continues from the hit, nudged off the surface; the input ray on a miss
    };

    /**
     * smallest incidence angle that is totally internally reflected going from n1 into n2
     * @return asin(n2 / n1); empty when n1 <= n2 (light entering an equal or denser medium always refracts)
     */
    std::optional<double> criticalAngle(double n1, double n2);

    /**
     * Snell's law with TIR fallback:  n1 sin(theta1) = n2 sin(theta2)
     *
     * @param d direction of the incoming ray (need not be unit length)
     * @param n unit surface normal; either side is accepted (it is flipped to face against the ray)
     * @param n1 refractive index the ray is leaving
     * @param n2 refractive index the ray is entering
     * @return the refracted ray, or, beyond the critical angle, the reflected ray with totalInternal set.
     *         At exactly the critical angle the refracted ray grazes along the surface.
     */
    Refraction refractOrReflect(const Vec2& d, const Vec2& n, double n1, double n2);

    /**
     * full step at a flat boundary: intersect, then refract (or totally reflect)
     * @param ray incoming ray
     * @param boundary the interface between the two media
     * @param n1 refractive index on the side the ray arrives from
     * @param n2 refractive index on the far side
     */
    SurfaceInteraction refractAtSegment(const Ray& ray, const Segment& boundary, double n1, double n2);
}
