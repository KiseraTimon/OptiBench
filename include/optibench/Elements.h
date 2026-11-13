#pragma once
#include <vector>

#include "OpticalElement.h"
#include "clip_fill.h"      // Polygon (= std::vector<Vec2>)
#include "Line.h"           // Segment

namespace ob {
    /**
     * A flat, thin slab: used for mirrors and screens.
     *
     * Local layout: the active face is the segment from (-halfLength, 0) to (+halfLength, 0);
     * the slab body extends towards local +y (so the rays should arrive from the -y side).
     */
    class FlatElement : public OpticalElement {
    public:
        FlatElement(std::string name, Transform2D transform, double halfLength,
                    Material material, double thickness = 8.0);

        Segment face() const;           // the active face in world space
        Polygon bodyWorld() const;      // the slab outline in world space

        Hit intersect(const Ray& ray) const override;
        void draw(Framebuffer& fb) const override;

    private:
        double halfLength_;
        double thickness_;
    };


    /**
     * A lens: a circle in local space, transformed into the world.
     * A uniform scale gives a circular lens; a non-uniform scale gives an elliptical lens
     * (rotated or not). Rays are intersected in LOCAL space by reusing rayCircleIntersect.
     */
    class LensElement : public OpticalElement {
    public:
        LensElement(std::string name, Transform2D transform, double radius,
                    Material material = Material::glass(1.5));

        bool isSolid() const override { return true; }
        Hit intersect(const Ray& ray) const override;
        void draw(Framebuffer& fb) const override;

        double radiusX() const;     // world half-width  (before any rotation)
        double radiusY() const;     // world half-height (before any rotation)

    private:
        double radius_;
    };


    /**
     * A prism: any CONVEX polygon given in local space (concave shapes are not supported).
     */
    class PrismElement : public OpticalElement {
    public:
        PrismElement(std::string name, Transform2D transform, Polygon localVertices,
                     Material material = Material::glass(1.5));

        // isosceles triangle centred on its centroid, apex pointing up on screen
        static Polygon triangle(double base, double height);

        Polygon worldVertices() const;

        bool isSolid() const override { return true; }
        Hit intersect(const Ray& ray) const override;
        void draw(Framebuffer& fb) const override;

    private:
        Polygon local_;
    };


    /**
     * A laser: emits one beam from its position along its local +x axis.
     * Beams pass through the source itself.
     */
    class LaserSource : public OpticalElement {
    public:
        LaserSource(std::string name, Transform2D transform, Color beamColor,
                    double bodyLength = 60.0, double bodyWidth = 34.0);

        Ray emissionRay() const;
        Color beamColor() const { return beamColor_; }

        Hit intersect(const Ray&) const override { return Hit{}; }
        void draw(Framebuffer& fb) const override;

    private:
        Color beamColor_;
        double bodyLength_;
        double bodyWidth_;
    };
}
