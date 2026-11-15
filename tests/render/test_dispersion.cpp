// Dispersion: Material::indexAt (Cauchy fit to catalogue n_d and Abbe number) and wavelength tracing.
#include <algorithm>
#include <cmath>

#include "curves/TestHarness.h"
#include "optibench/Scene.h"
#include "optibench/Tracer.h"

using namespace ob;

namespace {
    void testIndexModel(test::Context& ctx) {
        for (const Material& m : {Material::crownBK7(), Material::flintF2(), Material::denseFlintSF11(), Material::denseFlintSF66()}) {
            // n_d is reproduced exactly, and the Abbe number is honoured: V = (n_d - 1) / (n_F - n_C)
            CHECK(m.indexAt(Material::kDesignWavelength) == m.refractiveIndex);
            const double v = (m.refractiveIndex - 1.0) / (m.indexAt(Material::kBlueF) - m.indexAt(Material::kRedC));
            CHECK_NEAR(v, m.abbeNumber, 1e-9);
            // normal dispersion: the index falls as the wavelength grows
            bool falling = true;
            for (double nm = 400; nm < 700; nm += 10) falling = falling && m.indexAt(nm + 10) < m.indexAt(nm);
            CHECK(falling);
        }
        // against the Schott catalogue: N-BK7 at the mercury h line (404.66 nm) is 1.53024
        CHECK_NEAR(Material::crownBK7().indexAt(404.66), 1.53024, 0.002);
        // a glass without dispersion has one index at every wavelength
        const Material plain = Material::glass(1.5);
        CHECK(plain.indexAt(400) == 1.5 && plain.indexAt(700) == 1.5);
        CHECK(Material::crownBK7().indexAt(0.0) == Material::crownBK7().refractiveIndex);
        // flints disperse more than crowns
        const auto spread = [](const Material& m) { return m.indexAt(400) - m.indexAt(700); };
        CHECK(spread(Material::denseFlintSF66()) > spread(Material::denseFlintSF11()));
        CHECK(spread(Material::denseFlintSF11()) > spread(Material::flintF2()));
        CHECK(spread(Material::flintF2()) > spread(Material::crownBK7()));
    }

    Scene prismBench(const Material& glass) {
        Scene s(ClipRect{0, 0, 1000, 600});
        s.emplace<PrismElement>("prism", Transform2D{.position = {500, 300}}, PrismElement::triangle(160, 140), glass);
        return s;
    }

    Vec2 exitDirection(const BeamPath& p) {
        const BeamSegment& last = p.segments.back();
        return (last.b - last.a).normalized();
    }

    void testPrismSplitsWhiteLight(test::Context& ctx) {
        // an F2 flint prism (critical angle 38 degrees), the ray near minimum deviation
        const Scene s = prismBench(Material::flintF2());
        const Ray in{{200, 320}, Vec2{1, -0.35}.normalized()};
        const BeamPath blue = traceBeam(s, in, 32, 450), red = traceBeam(s, in, 32, 650);
        CHECK(blue.events.size() == 2 && red.events.size() == 2);
        // both refract twice, obeying Snell's law with the index of their own wavelength
        for (const BeamPath* p : {&blue, &red})
            for (const BeamEvent& e : p->events) {
                CHECK(e.kind == BeamEvent::Kind::Refract);
                CHECK_NEAR(e.n1 * std::sin(e.incidenceDeg * kPi / 180), e.n2 * std::sin(e.refractionDeg * kPi / 180), 1e-9);
            }
        CHECK_NEAR(blue.events[0].n2, Material::flintF2().indexAt(450), 1e-15);
        CHECK_NEAR(red.events[0].n2, Material::flintF2().indexAt(650), 1e-15);
        // blue is bent further from its original direction than red
        const double blueTurn = std::acos(std::clamp(exitDirection(blue).dot(in.direction), -1.0, 1.0));
        const double redTurn = std::acos(std::clamp(exitDirection(red).dot(in.direction), -1.0, 1.0));
        CHECK(blueTurn > redTurn + 0.003);
    }

    void testNoDispersionMeansOnePath(test::Context& ctx) {
        const Scene s = prismBench(Material::glass(1.6));
        const Ray in{{200, 320}, Vec2{1, -0.25}.normalized()};
        const BeamPath a = traceBeam(s, in, 32, 420), b = traceBeam(s, in, 32, 690);
        CHECK(a.segments.size() == b.segments.size());
        for (std::size_t i = 0; i < a.segments.size() && i < b.segments.size(); ++i) {
            CHECK_VEC_NEAR(a.segments[i].a, b.segments[i].a, 0.0);
            CHECK_VEC_NEAR(a.segments[i].b, b.segments[i].b, 0.0);
        }
        // and the default wavelength is the design wavelength
        const Scene d = prismBench(Material::crownBK7());
        const BeamPath implicitD = traceBeam(d, in), explicitD = traceBeam(d, in, 32, Material::kDesignWavelength);
        CHECK_VEC_NEAR(implicitD.end, explicitD.end, 0.0);
    }
}

int main() {
    return test::runTests("dispersion", {
        {"Cauchy index from n_d and the Abbe number", testIndexModel},
        {"a flint prism splits blue from red", testPrismSplitsWhiteLight},
        {"without dispersion every colour takes one path", testNoDispersionMeansOnePath},
    });
}
