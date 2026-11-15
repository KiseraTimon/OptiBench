#pragma once
#include <string>
#include <vector>

#include "Material.h"
#include "Ray.h"
#include "Vec2.h"

namespace ob {
    class Scene;
    class OpticalElement;

    // one straight stretch of a beam
    struct BeamSegment {
        Vec2 a, b;
        bool inGlass = false;       // true while travelling inside a solid
    };

    // what happened to the beam at one surface
    struct BeamEvent {
        enum class Kind { Reflect, Refract, TotalInternalReflection, Absorb };

        Kind kind = Kind::Reflect;
        std::string element;        // name of the element that was hit
        Vec2 point;                 // where it happened
        double incidenceDeg = 0.0;  // angle between the ray and the surface normal
        double refractionDeg = 0.0; // Refract only: angle of the bent ray from the normal
        double n1 = 1.0, n2 = 1.0;  // Refract only: indices before and after
    };

    // the complete journey of one beam
    struct BeamPath {
        std::vector<BeamSegment> segments;
        std::vector<BeamEvent> events;
        Vec2 end;                               // last point of the beam
        bool hitScreen = false;                 // true if it finished on a Screen element
    };

    /**
     * follows a beam through the scene: reflecting off mirrors, refracting through glass
     * (or bouncing on total internal reflection) until it lands on a screen, leaves the scene,
     * or runs out of steps.
     *
     * A beam may start inside glass (Scene::solidAt decides): it then leaves through the far face.
     * Limitation: glass elements must not overlap (nested media are not modelled).
     * Scene::overlappingPairs() reports any overlap, so a bench can be checked before tracing.
     *
     * wavelengthNm: the colour of the light. Glass refracts it with Material::indexAt(wavelengthNm),
     * so dispersive glass (abbeNumber > 0) bends blue more than red. The default is the design
     * wavelength, where every glass has exactly its refractiveIndex.
     */
    BeamPath traceBeam(const Scene& scene, const Ray& start, int maxSteps = 32,
                       double wavelengthNm = Material::kDesignWavelength);
}
