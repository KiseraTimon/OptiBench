#pragma once
#include <string>

#include "Framebuffer.h"
#include "Material.h"
#include "Ray.h"
#include "Transform2D.h"

namespace ob {
    /**
     * Anything that sits on the optical bench.
     *
     * Every element has a name (so animation code can find it), a Transform2D
     * (position / rotation / scale) and a Material. Both are plain public data on purpose:
     * animation just assigns to `transform` between frames.
     *
     * Hit conventions returned by intersect():
     *      - solid elements (isSolid() == true): hit.normal points OUT of the solid
     *      - flat elements (mirror, screen):     hit.normal faces back towards the ray
     */
    class OpticalElement {
    public:
        OpticalElement(std::string name, Transform2D transform, Material material)
            : name(std::move(name)), transform(transform), material(material) {}
        virtual ~OpticalElement() = default;

        std::string name;
        Transform2D transform;
        Material material;

        // nearest intersection of a world-space ray with this element; hit.t is a world distance
        virtual Hit intersect(const Ray& ray) const = 0;

        // paints the element onto the framebuffer
        virtual void draw(Framebuffer& fb) const = 0;

        // true for glass bodies that have an inside and an outside
        virtual bool isSolid() const { return false; }
    };
}
