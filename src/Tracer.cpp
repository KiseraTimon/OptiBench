#include "optibench/Tracer.h"

#include <algorithm>
#include <cmath>

#include "optibench/Scene.h"
#include "optibench/Transform2D.h"
#include "optibench/optics.h"

namespace ob {
    namespace {
        constexpr double kFar = 5000.0;     // how far a beam travels if it hits nothing
        constexpr double kNudge = 1e-6;     // pushes a new ray off the surface it just left

        double degrees(double cosine) {
            return std::acos(std::clamp(cosine, -1.0, 1.0)) * 180.0 / kPi;
        }
    }

    BeamPath traceBeam(const Scene& scene, const Ray& start, int maxSteps) {
        BeamPath path;
        Ray ray{start.origin, start.direction.normalized()};
        bool inGlass = false;

        for (int step = 0; step < maxSteps; ++step) {
            const Scene::Intersection hit = scene.nearestHit(ray);

            // nothing in the way: the beam leaves the scene
            if (!hit.element) {
                const Vec2 end = ray.origin + ray.direction * kFar;
                path.segments.push_back({ray.origin, end, inGlass});
                path.end = end;
                return path;
            }

            path.segments.push_back({ray.origin, hit.hit.point, inGlass});
            path.end = hit.hit.point;

            const Material& m = hit.element->material;
            const Vec2 d = ray.direction;

            // a normal that opposes the ray (needed by reflect and refract)
            const bool entering = d.dot(hit.hit.normal) < 0.0;
            const Vec2 facing = entering ? hit.hit.normal : -hit.hit.normal;

            BeamEvent ev;
            ev.element = hit.element->name;
            ev.point = hit.hit.point;
            ev.incidenceDeg = degrees(-d.dot(facing));

            Vec2 next = d;
            switch (m.type) {
                case SurfaceType::Mirror:
                    ev.kind = BeamEvent::Kind::Reflect;
                    next = reflect(d, facing);
                    break;

                case SurfaceType::Glass: {
                    // outward normal + direction tell us whether the ray is going in or coming out
                    const double n1 = entering ? 1.0 : m.refractiveIndex;
                    const double n2 = entering ? m.refractiveIndex : 1.0;
                    Vec2 bent;
                    if (refract(d, facing, n1, n2, bent)) {
                        ev.kind = BeamEvent::Kind::Refract;
                        ev.n1 = n1;
                        ev.n2 = n2;
                        ev.refractionDeg = degrees(bent.dot(-facing));
                        next = bent;
                        inGlass = entering;
                    } else {
                        ev.kind = BeamEvent::Kind::TotalInternalReflection;
                        next = reflect(d, facing);
                    }
                    break;
                }

                case SurfaceType::Screen:
                    ev.kind = BeamEvent::Kind::Absorb;
                    path.events.push_back(ev);
                    path.hitScreen = true;
                    return path;

                case SurfaceType::Emitter:
                    break;      // beams pass through sources (intersect() never reports a hit anyway)
            }

            path.events.push_back(ev);
            ray = Ray{hit.hit.point + next * kNudge, next};
        }
        return path;
    }
}
