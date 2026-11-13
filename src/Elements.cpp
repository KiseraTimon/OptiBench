#include "optibench/Elements.h"

#include <cmath>

#include "optibench/circle_ellipse.h"

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


    // --------------------------------------------------------------- PrismElement

    PrismElement::PrismElement(std::string name, Transform2D transform, Polygon localVertices, Material material)
        : OpticalElement(std::move(name), transform, material), local_(std::move(localVertices)) {}

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
        const Polygon w = worldVertices();
        Hit best;
        if (w.size() < 3) return best;

        // nearest crossing over all edges
        for (std::size_t i = 0; i < w.size(); ++i) {
            const Hit h = intersectRaySegment(ray, Segment{w[i], w[(i + 1) % w.size()]});
            if (h.hit && (!best.hit || h.t < best.t)) best = h;
        }
        if (!best.hit) return best;

        // edge normals face the ray; a solid needs them pointing OUT (valid for convex shapes)
        Vec2 centroid{0.0, 0.0};
        for (const Vec2& v : w) centroid = centroid + v;
        centroid = centroid / static_cast<double>(w.size());
        if (best.normal.dot(best.point - centroid) < 0.0) best.normal = -best.normal;

        return best;
    }

    void PrismElement::draw(Framebuffer& fb) const {
        const Polygon w = worldVertices();
        fillPolygon(fb, w, material.fill);
        drawPolygon(fb, w, material.outline);
    }


    // ---------------------------------------------------------------- LaserSource

    LaserSource::LaserSource(std::string name, Transform2D transform, Color beamColor,
                             double bodyLength, double bodyWidth)
        : OpticalElement(std::move(name), transform, Material::emitter()),
          beamColor_(beamColor), bodyLength_(bodyLength), bodyWidth_(bodyWidth) {}

    Ray LaserSource::emissionRay() const {
        return {transform.position, Vec2{std::cos(transform.rotation), std::sin(transform.rotation)}};
    }

    void LaserSource::draw(Framebuffer& fb) const {
        // body sits behind the nose (local -x), the nose is the local origin
        const Polygon body = {
            transform.toWorld({-bodyLength_, -bodyWidth_ / 2.0}),
            transform.toWorld({0.0, -bodyWidth_ / 2.0}),
            transform.toWorld({0.0, bodyWidth_ / 2.0}),
            transform.toWorld({-bodyLength_, bodyWidth_ / 2.0}),
        };
        fillPolygon(fb, body, material.fill);
        drawPolygon(fb, body, material.outline);

        // glowing aperture at the nose
        drawCircle(fb, px(transform.position.x), px(transform.position.y), 4, beamColor_);
    }
}
