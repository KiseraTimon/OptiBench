#include "optibench/Elements.h"

#include <algorithm>
#include <cmath>
#include <stdexcept>

#include "optibench/circle_ellipse.h"
#include "optibench/curves/CurveRender.h"
#include "optibench/curves/OpticalCurve.h"
#include "optibench/geometry/Intersect.h"
#include "optibench/geometry/PolygonOps.h"

namespace ob {
    namespace {
        int px(double v) { return static_cast<int>(std::lround(v)); }

        // 0 = axis-aligned, 1 = turned a quarter, -1 = any other angle
        int quarterTurns(double rotation) {
            double a = std::fmod(rotation, kPi);
            if (a < 0.0) a += kPi;
            constexpr double tol = 1e-6;
            if (a < tol || kPi - a < tol) return 0;
            if (std::fabs(a - kPi / 2.0) < tol) return 1;
            return -1;
        }

        // world box of a local box: the four transformed corners bound everything inside it
        geom::Aabb worldBox(const Transform2D& t, const geom::Aabb& local) {
            geom::Aabb box;
            if (local.empty()) return box;
            box.expand(t.toWorld(local.min));
            box.expand(t.toWorld({local.max.x, local.min.y}));
            box.expand(t.toWorld(local.max));
            box.expand(t.toWorld({local.min.x, local.max.y}));
            return box;
        }

        // a world-space hit from a hit found in local space (t is recomputed as a world distance)
        Hit hitToWorld(const Transform2D& t, const Ray& worldRay, const Hit& local) {
            Hit out;
            if (!local.hit) return out;
            out.hit = true;
            out.point = t.toWorld(local.point);
            out.t = (out.point - worldRay.origin).length();
            out.normal = t.normalToWorld(local.normal);
            return out;
        }
    }


    // ---------------------------------------------------------------- FlatElement

    FlatElement::FlatElement(std::string name, Transform2D transform, double halfLength,
                             Material material, double thickness)
        : OpticalElement(std::move(name), transform, material),
          halfLength_(halfLength), thickness_(thickness) {}

    Segment FlatElement::face() const {
        return {transform.toWorld({-halfLength_, 0.0}), transform.toWorld({halfLength_, 0.0})};
    }

    Polygon FlatElement::bodyWorld() const {
        return {
            transform.toWorld({-halfLength_, 0.0}),
            transform.toWorld({halfLength_, 0.0}),
            transform.toWorld({halfLength_, thickness_}),
            transform.toWorld({-halfLength_, thickness_}),
        };
    }

    Hit FlatElement::intersect(const Ray& ray) const {
        // intersectRaySegment already returns a normal that faces the ray
        return intersectRaySegment(ray, face());
    }

    void FlatElement::draw(Framebuffer& fb) const {
        const Polygon body = bodyWorld();
        fillPolygon(fb, body, material.fill);
        drawPolygon(fb, body, material.outline);

        // bright reflective face
        if (material.type == SurfaceType::Mirror) {
            const Segment f = face();
            drawLineWu(fb, f.a, f.b, colors::White);
        }
    }

    geom::Aabb FlatElement::bounds() const { return geom::boundsOf(bodyWorld()); }

    bool FlatElement::contains(const Vec2& p) const { return geom::contains(bodyWorld(), p); }


    // ---------------------------------------------------------------- LensElement

    LensElement::LensElement(std::string name, Transform2D transform, double radius, Material material)
        : OpticalElement(std::move(name), transform, material), radius_(radius) {}

    double LensElement::radiusX() const { return radius_ * std::fabs(transform.scale.x); }
    double LensElement::radiusY() const { return radius_ * std::fabs(transform.scale.y); }

    Hit LensElement::intersect(const Ray& ray) const {
        const Vec2 d = ray.direction.normalized();

        /**
         * the lens is a circle in LOCAL space. Instead of writing ray-ellipse maths, the ray is
         * carried into local space, intersected with the circle there, and the result carried back.
         * A transform is affine, so the ray parameter t is the same in both spaces.
         */
        const Ray local{transform.toLocal(ray.origin), transform.dirToLocal(d)};
        const Hit h = rayCircleIntersect(local, Vec2{0.0, 0.0}, radius_);

        Hit out;
        if (!h.hit) return out;

        out.hit = true;
        out.t = h.t;
        out.point = ray.origin + d * h.t;
        out.normal = transform.normalToWorld(h.normal);     // outward
        return out;
    }

    void LensElement::draw(Framebuffer& fb) const {
        const int cx = px(transform.position.x), cy = px(transform.position.y);
        const double rx = radiusX(), ry = radiusY();
        const int turns = quarterTurns(transform.rotation);

        if (std::fabs(rx - ry) < 0.5) {
            // circular lens: midpoint circle
            const int r = px((rx + ry) / 2.0);
            fillCircle(fb, cx, cy, r, material.fill);
            drawCircle(fb, cx, cy, r, material.outline);
            drawCircle(fb, cx, cy, r - 1, material.outline);
        } else if (turns >= 0) {
            // axis-aligned ellipse (a quarter turn swaps the radii): midpoint ellipse
            const int ex = px(turns == 0 ? rx : ry), ey = px(turns == 0 ? ry : rx);
            fillEllipse(fb, cx, cy, ex, ey, material.fill);
            drawEllipse(fb, cx, cy, ex, ey, material.outline);
            drawEllipse(fb, cx, cy, ex - 1, ey - 1, material.outline);
        } else {
            // tilted ellipse: sample the transformed circle and draw it as a polygon
            constexpr int segments = 96;
            Polygon outline;
            outline.reserve(segments);
            for (int i = 0; i < segments; ++i) {
                const double a = 2.0 * kPi * i / segments;
                outline.push_back(transform.toWorld({radius_ * std::cos(a), radius_ * std::sin(a)}));
            }
            fillPolygon(fb, outline, material.fill);
            drawPolygon(fb, outline, material.outline);
        }
    }


    geom::Aabb LensElement::bounds() const {
        // the half-extents of an ellipse with semi-axes (a, b) rotated by theta
        const double a = radiusX(), b = radiusY();
        const double c = std::cos(transform.rotation), s = std::sin(transform.rotation);
        const double ex = std::sqrt(a * a * c * c + b * b * s * s);
        const double ey = std::sqrt(a * a * s * s + b * b * c * c);
        geom::Aabb box;
        box.expand(transform.position - Vec2{ex, ey});
        box.expand(transform.position + Vec2{ex, ey});
        return box;
    }

    bool LensElement::contains(const Vec2& p) const {
        return transform.toLocal(p).length() <= radius_ * (1.0 + 1e-12);
    }

    std::vector<Vec2> LensElement::outline() const {
        constexpr int segments = 96;
        std::vector<Vec2> out;
        out.reserve(segments);
        for (int i = 0; i < segments; ++i) {
            const double a = 2.0 * kPi * i / segments;
            out.push_back(transform.toWorld({radius_ * std::cos(a), radius_ * std::sin(a)}));
        }
        return out;
    }


    // --------------------------------------------------------------- PrismElement

    PrismElement::PrismElement(std::string name, Transform2D transform, Polygon localVertices, Material material)
        : OpticalElement(std::move(name), transform, material), local_(std::move(localVertices)) {
        if (!geom::isSimple(local_)) {
            throw std::invalid_argument("PrismElement '" + this->name + "': the outline must be a simple polygon");
        }
    }

    Polygon PrismElement::triangle(double base, double height) {
        // centroid of a triangle sits 1/3 of the height above the base
        return {{0.0, -2.0 * height / 3.0}, {base / 2.0, height / 3.0}, {-base / 2.0, height / 3.0}};
    }

    Polygon PrismElement::worldVertices() const {
        Polygon world;
        world.reserve(local_.size());
        for (const Vec2& v : local_) world.push_back(transform.toWorld(v));
        return world;
    }

    Hit PrismElement::intersect(const Ray& ray) const {
        // nearest crossing with an OUTWARD normal, from the winding order: correct for concave
        // outlines too (the old centroid test was only valid for convex ones)
        return geom::rayPolygon(ray, worldVertices());
    }

    void PrismElement::draw(Framebuffer& fb) const {
        const Polygon w = worldVertices();
        fillPolygon(fb, w, material.fill);
        drawPolygon(fb, w, material.outline);
    }

    geom::Aabb PrismElement::bounds() const { return geom::boundsOf(worldVertices()); }

    bool PrismElement::contains(const Vec2& p) const { return geom::contains(worldVertices(), p); }


    // ----------------------------------------------------------- ThickLensElement

    ThickLensElement::ThickLensElement(std::string name, Transform2D transform, geom::Lens shape, Material material)
        : OpticalElement(std::move(name), transform, material), shape_(shape) {}

    Hit ThickLensElement::intersect(const Ray& ray) const {
        // carry the ray into the lens frame, intersect the exact shape there, carry the hit back
        const Vec2 d = ray.direction.normalized();
        const Ray local{transform.toLocal(ray.origin), transform.dirToLocal(d)};
        return hitToWorld(transform, Ray{ray.origin, d}, shape_.intersect(local));
    }

    void ThickLensElement::draw(Framebuffer& fb) const {
        const Polygon w = outline();
        fillPolygon(fb, w, material.fill);
        drawPolygon(fb, w, material.outline);
    }

    geom::Aabb ThickLensElement::bounds() const { return worldBox(transform, shape_.bounds()); }

    bool ThickLensElement::contains(const Vec2& p) const { return shape_.contains(transform.toLocal(p)); }

    std::vector<Vec2> ThickLensElement::outline() const { return geom::transformed(shape_.outline(48), transform); }

    Vec2 ThickLensElement::backFocus() const {
        return transform.toWorld({shape_.thickness() / 2.0 + shape_.backFocalDistance(material.refractiveIndex), 0.0});
    }


    // -------------------------------------------------------- CurvedMirrorElement

    CurvedMirrorElement::CurvedMirrorElement(std::string name, Transform2D transform, curves::Bezier localProfile,
                                             Material material, double thickness)
        : OpticalElement(std::move(name), transform, material), local_(std::move(localProfile)), thickness_(thickness) {}

    curves::Bezier CurvedMirrorElement::parabola(double focalLength, double halfAperture) {
        return curves::parabolicProfile({0.0, 0.0}, {-1.0, 0.0}, focalLength, halfAperture);
    }

    curves::Bezier CurvedMirrorElement::worldProfile() const {
        const std::span<const Vec2> cps = local_.controlPoints();
        std::vector<Vec2> world;
        world.reserve(cps.size());
        for (const Vec2& p : cps) world.push_back(transform.toWorld(p));
        return curves::Bezier(std::move(world));
    }

    Hit CurvedMirrorElement::intersect(const Ray& ray) const {
        // curves::toHit already gives the flat-element convention: the normal faces the ray
        return curves::toHit(curves::intersectRay(ray, worldProfile()));
    }

    void CurvedMirrorElement::draw(Framebuffer& fb) const {
        const Polygon body = outline();
        fillPolygon(fb, body, material.fill);
        drawPolygon(fb, body, material.outline);
        if (material.type == SurfaceType::Mirror) curves::drawCurve(fb, worldProfile(), colors::White, 0.25, true);
    }

    geom::Aabb CurvedMirrorElement::bounds() const {
        // the curve lies in its control polygon's hull; the body adds the thickness behind it
        geom::Aabb local = geom::boundsOf(local_.controlPoints());
        local.max.x += thickness_;
        return worldBox(transform, local);
    }

    bool CurvedMirrorElement::contains(const Vec2& p) const { return geom::contains(outline(), p); }

    std::vector<Vec2> CurvedMirrorElement::outline() const {
        // the face, then the same points pushed back by the thickness, in reverse: a curved band
        const curves::Polyline face = local_.flatten(0.25);
        std::vector<Vec2> band;
        band.reserve(2 * face.size());
        for (const Vec2& p : face) band.push_back(transform.toWorld(p));
        for (auto it = face.rbegin(); it != face.rend(); ++it) band.push_back(transform.toWorld(*it + Vec2{thickness_, 0.0}));
        return band;
    }


    // ---------------------------------------------------------------- LaserSource

    LaserSource::LaserSource(std::string name, Transform2D transform, Color beamColor,
                             double bodyLength, double bodyWidth)
        : OpticalElement(std::move(name), transform, Material::emitter()),
          beamColor_(beamColor), bodyLength_(bodyLength), bodyWidth_(bodyWidth) {}

    Ray LaserSource::emissionRay() const {
        return {transform.position, Vec2{std::cos(transform.rotation), std::sin(transform.rotation)}};
    }

    Polygon LaserSource::bodyWorld() const {
        // body sits behind the nose (local -x), the nose is the local origin
        return {
            transform.toWorld({-bodyLength_, -bodyWidth_ / 2.0}),
            transform.toWorld({0.0, -bodyWidth_ / 2.0}),
            transform.toWorld({0.0, bodyWidth_ / 2.0}),
            transform.toWorld({-bodyLength_, bodyWidth_ / 2.0}),
        };
    }

    void LaserSource::draw(Framebuffer& fb) const {
        const Polygon body = bodyWorld();
        fillPolygon(fb, body, material.fill);
        drawPolygon(fb, body, material.outline);

        // glowing aperture at the nose
        drawCircle(fb, px(transform.position.x), px(transform.position.y), 4, beamColor_);
    }

    geom::Aabb LaserSource::bounds() const { return geom::boundsOf(bodyWorld()); }

    bool LaserSource::contains(const Vec2& p) const { return geom::contains(bodyWorld(), p); }
}
