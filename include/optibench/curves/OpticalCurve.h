#pragma once

/*
 * Special Notes: Curves as Optical Surfaces
 *
 * Lets a Bezier / spline profile act as a lens or mirror surface in OptiBench. This header
 * provides the geometry the physics needs (where a ray hits, and the surface normal there);
 * reflection and Snell's law refraction belong to the optics module.
 *
 * Ray-curve intersection
 * ----------------------
 * The ray o + s d (|d| = 1, s >= 0) lies on the line { p : cross(d, p - o) = 0 }. Substituting
 * the curve B(t) = sum B(i,n)(t) Pi gives the scalar Bernstein polynomial
 *
 *      f(t) = sum B(i,n)(t) ci,      ci = cross(d, Pi - o)
 *
 * whose roots in [0, 1] are exactly the crossings of the curve with the ray's line (the
 * "Bezier clipping" fat line collapsed to the ray itself). Roots are isolated by recursive
 * subdivision with the convex hull test of the coefficients (the 1D bounding box of the
 * curve's distance to the line), refined by bisection / Newton (see bernsteinRoots()), then
 * mapped to the ray distance s = dot(d, B(t) - o) and filtered to s in [minDistance, maxDistance].
 *
 *   - tangent rays produce a double root and are reported once, flagged `tangential`
 *   - rays that miss are rejected immediately when every ci has the same sign
 *   - a ray running exactly along a straight segment (f identically zero) has infinitely
 *     many intersections; that segment reports none
 *   - hits at a shared join point of a composite curve are reported once
 *
 * Cost per Bezier segment of degree n: O(n^2) per subdivision step, O(n^2 log(1/eps)) per root.
 *
 * Normals
 * -------
 * surfaceNormal is the curve's own normal: the tangent turned with Vec2::perp() for open
 * profiles (flip with flipNormals()), and the outward normal for closed profiles.
 * normal is the same vector flipped, if needed, so dot(normal, d) < 0: it always faces the
 * incoming ray, which is the convention of the reflection formula r = d - 2 (d.n) n and of the
 * usual vector form of Snell's law. frontFace tells the physics which side the ray came from:
 * for a closed lens, frontFace == true means the ray is entering the glass (n1 = outside,
 * n2 = inside), false means it is leaving.
 */

#include <limits>
#include <optional>
#include <vector>

#include "optibench/Ray.h"
#include "optibench/curves/Composite.h"

namespace ob::curves {

    // one intersection of a ray with a curve
    struct CurveHit {
        double distance = 0.0;   // ray distance s: point = origin + s * normalized(direction)
        double u = 0.0;          // global curve parameter (segment + t)
        std::size_t segment = 0; // index of the Bezier segment that was hit
        double t = 0.0;          // local parameter within that segment
        Vec2 point;              // the hit point
        Vec2 normal;             // unit normal facing the incoming ray: dot(normal, d) < 0
        Vec2 surfaceNormal;      // unit normal of the surface itself (outward for closed profiles)
        bool frontFace = false;  // the ray travels against surfaceNormal (entering a closed profile)
        bool tangential = false; // the ray grazes the curve (double root)
    };

    // filters for ray-curve intersection
    struct RayCurveOptions {
        // hits closer than this are ignored (avoids re-hitting the surface a ray starts on)
        double minDistance = 1e-9;

        // hits farther than this are ignored
        double maxDistance = std::numeric_limits<double>::infinity();

        // include grazing (tangential) hits
        bool includeTangential = true;
    };

    /**
     * all intersections of a ray with a Bezier curve, sorted by distance
     * @param normalSign double: +1 uses tangent.perp() as surfaceNormal, -1 the opposite side
     * @return empty if the direction is zero or nothing is hit
     */
    std::vector<CurveHit> intersectRay(const Ray& ray, const Bezier& curve, const RayCurveOptions& options = {},
                                       double normalSign = 1.0);

    // all intersections with a composite curve, sorted by distance (join points reported once)
    std::vector<CurveHit> intersectRay(const Ray& ray, const CompositeBezier& curve,
                                       const RayCurveOptions& options = {}, double normalSign = 1.0);

    // the nearest hit as the project's ob::Hit (hit = false when hits is empty)
    Hit toHit(const std::vector<CurveHit>& hits);

    // what a surface does with light; interpreted by the physics module
    enum class SurfaceKind { Refractive, Reflective, Absorbing };

    // a curve profile used as an optical surface
    class OpticalSurface {
    public:
        /**
         * @param profile the surface shape; if it is closed, surface normals point outward
         * @param kind refractive (lens), reflective (mirror) or absorbing (detector / stop)
         * @param refractiveIndex index of the material inside a closed profile (or behind an open one)
         */
        explicit OpticalSurface(CompositeBezier profile, SurfaceKind kind = SurfaceKind::Refractive,
                                double refractiveIndex = 1.5);

        const CompositeBezier& profile() const { return profile_; }
        SurfaceKind kind() const { return kind_; }
        double refractiveIndex() const { return refractiveIndex_; }

        // true if the profile is a closed loop
        bool closed() const { return closed_; }

        // reverses which side surfaceNormal points to
        void flipNormals() { normalSign_ = -normalSign_; }

        // all hits, sorted by distance
        std::vector<CurveHit> intersect(const Ray& ray, const RayCurveOptions& options = {}) const;

        // the nearest hit, if any
        std::optional<CurveHit> firstHit(const Ray& ray, const RayCurveOptions& options = {}) const;

        // closed profiles only: true if p is inside (even-odd rule); always false for open profiles
        bool contains(Vec2 p) const;

        // unit surface normal at global parameter u (same orientation as CurveHit::surfaceNormal)
        Vec2 surfaceNormalAt(double u) const;

    private:
        CompositeBezier profile_;
        SurfaceKind kind_;
        double refractiveIndex_;
        bool closed_;
        double normalSign_ = 1.0;
    };

    /*
     * Profile builders
     */
    /**
     * exact parabola y'^2 = 4 f x' (in the frame of the axis) as a quadratic Bezier
     * @param vertex Vec2: the parabola's vertex
     * @param axis Vec2: direction the parabola opens towards (the focus is vertex + f * axis)
     * @param focalLength double: f > 0
     * @param halfAperture double: half the height of the mirror, measured across the axis
     */
    Bezier parabolicProfile(Vec2 vertex, Vec2 axis, double focalLength, double halfAperture);

    // shape parameters of a lens whose optical axis is horizontal (along +x)
    struct LensSpec {
        Vec2 center;                // centre of the lens
        double aperture = 100.0;    // full height of the lens
        double edgeThickness = 6.0; // width of the flat rim at the top and bottom (>= 0)
        double frontSag = 15.0;     // how far the left face bulges out (negative: concave)
        double backSag = 15.0;      // how far the right face bulges out (negative: concave)
    };

    /**
     * closed lens profile: two parabolic faces (quadratic Beziers) joined by flat rims
     * Biconvex: both sags positive; plano-convex: one sag 0; meniscus / concave: negative sags.
     */
    CompositeBezier lensProfile(const LensSpec& spec);
}
