#pragma once

/*
 * Special Notes: Lens shape
 *
 * A real (thick) converging lens, in its own local frame:
 *
 *                 y
 *                 ^         rim (halfHeight)
 *             .---|---.
 *            /    |    \          optical axis = x axis, light normally travels towards +x
 *   front   |     +-----|---> x   front vertex at x = -thickness / 2
 *   (R1)     \    |    /  back    back vertex  at x = +thickness / 2
 *             '---|---'   (R2)
 *
 * Each face is either spherical and convex (radius > 0: the face bulges outwards) or flat
 * (radius = Lens::kFlat). An optional aperture trims the rim with two flat edges at |y| = aperture.
 * So biconvex, plano-convex and edged lenses are all covered.
 *
 * The solid is the INTERSECTION of convex pieces (one disc or half-plane per face, plus the
 * aperture half-planes), so a ray hit is just the overlap of each piece's span along the ray.
 * That gives exact entry and exit points and outward normals, also for rays starting inside.
 *
 * Concave (diverging) faces are not convex pieces and are not supported.
 */

#include <limits>
#include <vector>

#include "optibench/Ray.h"
#include "optibench/geometry/Primitives.h"

namespace ob::geom {

    class Lens {
    public:
        static constexpr double kFlat = std::numeric_limits<double>::infinity();

        /**
         * @param frontRadius radius of the face at -x (> 0, or kFlat)
         * @param backRadius radius of the face at +x (> 0, or kFlat)
         * @param thickness centre thickness, vertex to vertex (> 0)
         * @param aperture half-height at which the rim is cut flat (> 0, or kFlat for no cut)
         * @throws std::invalid_argument if a value is out of range, if a curved face would be more
         *         than a hemisphere (the thickness is too large for the radii; for a plano-convex lens
         *         this means thickness > radius), or if the lens would be unbounded (two flat faces
         *         and no aperture)
         */
        Lens(double frontRadius, double backRadius, double thickness, double aperture = kFlat);

        double frontRadius() const { return r1_; }
        double backRadius() const { return r2_; }
        double thickness() const { return d_; }

        // half-height of the rim: the aperture if it trims the lens, else where the faces meet
        double halfHeight() const { return h_; }

        // thickness at the rim (0 when the faces meet in a sharp edge)
        double edgeThickness() const;

        // true if local point p is inside or on the lens (within tol)
        bool contains(Vec2 p, double tol = kEps) const;

        // nearest boundary crossing at t >= tMin, normal pointing OUT (works from inside too)
        Hit intersect(const Ray& ray, double tMin = kEps) const;

        /**
         * closed outline, counter-clockwise (maths axes), for drawing and polygon queries
         * @param segmentsPerFace points used for each curved face (at least 2)
         */
        std::vector<Vec2> outline(int segmentsPerFace = 48) const;

        // tight local box: [-thickness/2, thickness/2] x [-halfHeight, halfHeight]
        Aabb bounds() const;

        /**
         * effective focal length in air, thick-lens lensmaker's equation
         *      1/f = (n - 1) [ 1/R1 + 1/R2 - (n - 1) d / (n R1 R2) ]
         * (radii positive for convex faces, 1/R = 0 for a flat face)
         */
        double focalLength(double n) const;

        // back focal distance: from the back vertex (x = +thickness/2) to the focus
        double backFocalDistance(double n) const;

        // front focal distance: from the front vertex (x = -thickness/2) back to the front focus
        double frontFocalDistance(double n) const;

    private:
        double r1_, r2_, d_, aperture_, h_;
    };
}
