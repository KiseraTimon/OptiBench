#pragma once
#include <string>
#include <vector>

#include "Framebuffer.h"
#include "Material.h"
#include "Ray.h"
#include "Transform2D.h"
#include "geometry/Primitives.h"    // geom::Aabb

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

        /*
         * Geometry queries (geometry module). Used by the scene for its broad phase, to find which
         * solid a beam starts in, and to check that no two elements overlap.
         */

        /**
         * world-space box that contains everything intersect() can hit.
         * It must never be too small: Scene::nearestHit skips the element when a ray misses it.
         * An empty box means "unknown": the element is then always tested.
         */
        virtual geom::Aabb bounds() const { return {}; }

        // true if the world point p is inside (or on) the element's body
        virtual bool contains(const Vec2& p) const { (void)p; return false; }

        // the body's outline in world space, as a simple polygon (curved parts sampled); empty if none
        virtual std::vector<Vec2> outline() const { return {}; }
    };
}
