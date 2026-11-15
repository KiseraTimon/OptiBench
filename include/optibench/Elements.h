#pragma once
#include <vector>

#include "OpticalElement.h"
#include "clip_fill.h"      // Polygon (= std::vector<Vec2>)
#include "Line.h"           // Segment
#include "curves/Bezier.h"  // CurvedMirrorElement's profile
#include "geometry/Lens.h"  // ThickLensElement's shape

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

        geom::Aabb bounds() const override;
        bool contains(const Vec2& p) const override;
        std::vector<Vec2> outline() const override { return bodyWorld(); }

        double halfLength() const { return halfLength_; }
        double thickness() const { return thickness_; }

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

        geom::Aabb bounds() const override;                 // exact box of the (rotated) ellipse
        bool contains(const Vec2& p) const override;
        std::vector<Vec2> outline() const override;         // 96-point polygon

        double radiusX() const;     // world half-width  (before any rotation)
        double radiusY() const;     // world half-height (before any rotation)

    private:
        double radius_;
    };

    /**
     * A prism: any SIMPLE polygon given in local space, convex or concave.
     * Rays are intersected with geom::rayPolygon, whose outward normals come from the winding
     * order, so notched and concave prisms refract correctly.
     * @throws std::invalid_argument (constructor) if the polygon is not simple (geom::isSimple)
     */
    class PrismElement : public OpticalElement {
    public:
        PrismElement(std::string name, Transform2D transform, Polygon localVertices,
                     Material material = Material::glass(1.5));

        static Polygon triangle(double base, double height);

        Polygon worldVertices() const;

        bool isSolid() const override { return true; }
        Hit intersect(const Ray& ray) const override;
        void draw(Framebuffer& fb) const override;

        geom::Aabb bounds() const override;
        bool contains(const Vec2& p) const override;
        std::vector<Vec2> outline() const override { return worldVertices(); }

    private:
        Polygon local_;
    };

    /**
     * A real thick lens (geometry module): biconvex, plano-convex, edged or slab, with exact
     * spherical faces. Local frame as in geom::Lens: optical axis along local +x, centred on
     * the transform's position. Rays are intersected in local space and carried back.
     *
     * Keep the scale uniform for physically meaningful focal lengths: a uniform scale s scales
     * every length, including the focal distances, by s.
     */
    class ThickLensElement : public OpticalElement {
    public:
        ThickLensElement(std::string name, Transform2D transform, geom::Lens shape,
                         Material material = Material::glass(1.5));

        const geom::Lens& shape() const { return shape_; }

        bool isSolid() const override { return true; }
        Hit intersect(const Ray& ray) const override;
        void draw(Framebuffer& fb) const override;

        geom::Aabb bounds() const override;
        bool contains(const Vec2& p) const override;
        std::vector<Vec2> outline() const override;

        // world position of the back focus (lensmaker, using the material's refractive index)
        Vec2 backFocus() const;

    private:
        geom::Lens shape_;
    };

    /**
     * A curved mirror whose face is a Bezier curve (curves module), e.g. an exact parabola.
     * Behaves like a flat mirror: hit.normal faces back towards the ray. The body (drawn, and
     * used for overlap checks) is the face thickened towards local +x.
     */
    class CurvedMirrorElement : public OpticalElement {
    public:
        CurvedMirrorElement(std::string name, Transform2D transform, curves::Bezier localProfile,
                            Material material = Material::mirror(), double thickness = 6.0);

        /**
         * local profile of a parabolic mirror: vertex at the origin, opening towards local -x,
         * focus at (-focalLength, 0). Rays travelling along +x converge on the focus.
         */
        static curves::Bezier parabola(double focalLength, double halfAperture);

        // the face in world space (Bezier curves are affine invariant: transform the control points)
        curves::Bezier worldProfile() const;

        Hit intersect(const Ray& ray) const override;
        void draw(Framebuffer& fb) const override;

        geom::Aabb bounds() const override;                 // control polygon box (convex hull property)
        bool contains(const Vec2& p) const override;
        std::vector<Vec2> outline() const override;

    private:
        curves::Bezier local_;
        double thickness_;
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

        geom::Aabb bounds() const override;
        bool contains(const Vec2& p) const override;
        std::vector<Vec2> outline() const override { return bodyWorld(); }

        Polygon bodyWorld() const;      // the housing behind the nose, in world space
        double bodyLength() const { return bodyLength_; }
        double bodyWidth() const { return bodyWidth_; }

    private:
        Color beamColor_;
        double bodyLength_;
        double bodyWidth_;
    };
}
