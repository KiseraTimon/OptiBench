#include "Trace.h"

#include <stdexcept>

namespace ob::app {
    std::vector<TracedRay> traceSources(const Scene& scene, const std::vector<SourceSpec>& sources) {
        std::vector<TracedRay> out;
        for (const SourceSpec& spec : sources) {
            const auto* laser = dynamic_cast<const LaserSource*>(scene.find(spec.element));
            if (!laser) throw std::runtime_error("traceSources: no laser named " + spec.element);
            const Ray ray = laser->emissionRay();

            switch (spec.kind) {
                case SourceSpec::Kind::Laser:
                    out.push_back({traceBeam(scene, ray, 32, spec.wavelengthNm), spec.wavelengthNm,
                                   render::laserColor(spec.wavelengthNm), &spec, 0});
                    break;

                case SourceSpec::Kind::WhiteLight: {
                    // every wavelength leaves along the same line; dispersive glass pulls them apart
                    int i = 0;
                    for (const render::SpectralSample& s : render::whiteLightSamples(spec.rays))
                        out.push_back({traceBeam(scene, ray, 32, s.wavelengthNm), s.wavelengthNm, s.weight, &spec, i++});
                    break;
                }

                case SourceSpec::Kind::Collimated: {
                    // parallel rays across the beam diameter, perpendicular to the direction of travel
                    const Vec2 dir = ray.direction.normalized();
                    const Vec2 side = dir.perp();
                    const render::LinearRgb c = render::laserColor(spec.wavelengthNm);
                    for (int i = 0; i < spec.rays; ++i) {
                        const double u = spec.rays == 1 ? 0.0 : (static_cast<double>(i) / (spec.rays - 1) - 0.5);
                        const Ray r{ray.origin + side * (u * spec.width), dir};
                        out.push_back({traceBeam(scene, r, 32, spec.wavelengthNm), spec.wavelengthNm, c, &spec, i});
                    }
                    break;
                }
            }
        }
        return out;
    }
}
