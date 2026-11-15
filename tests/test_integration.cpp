// Integration: the scene, elements and tracer working with the geometry, curves and optics modules.
// (Polygon.h's struct cannot share a file with Scene.h, which brings in clip_fill.h's Polygon alias:
//  its methods are tested in test_polygon_methods.cpp.)
#include <cmath>
#include <random>
#include <stdexcept>
#include <vector>

#include "curves/TestHarness.h"
#include "optibench/Scene.h"
#include "optibench/Tracer.h"
#include "optibench/geometry/Geometry.h"

using namespace ob;

namespace {
    using Poly = std::vector<Vec2>;

    // a bench with one of every element type, some rotated and scaled
    Scene everythingScene() {
        Scene s(ClipRect{0, 0, 1200, 800});
        s.emplace<LaserSource>("laser", Transform2D{.position = {80, 400}}, colors::Red);
        s.emplace<FlatElement>("mirror", Transform2D{.position = {250, 200}, .rotation = 0.6}, 40.0, Material::mirror());
        s.emplace<LensElement>("ellipse", Transform2D{.position = {420, 420}, .rotation = 0.7, .scale = {1.6, 0.8}}, 45.0);
        s.emplace<PrismElement>("prism", Transform2D{.position = {600, 250}, .rotation = 0.3}, PrismElement::triangle(90, 110));
        s.emplace<PrismElement>("notched", Transform2D{.position = {800, 500}, .rotation = -0.4},
                                Polygon{{-60, -80}, {60, -80}, {60, 80}, {-60, 80}, {0, 0}});
        s.emplace<ThickLensElement>("thick", Transform2D{.position = {950, 250}, .rotation = 0.2}, geom::Lens(120, 160, 30));
        s.emplace<CurvedMirrorElement>("dish", Transform2D{.position = {1100, 600}, .rotation = 2.5},
                                       CurvedMirrorElement::parabola(80, 60));
        s.emplace<FlatElement>("screen", Transform2D{.position = {600, 720}}, 150.0, Material::screen());
        return s;
    }

    void testBroadPhaseIsExact(test::Context& ctx) {
        const Scene s = everythingScene();
        std::mt19937 gen(99u);
        std::uniform_real_distribution<double> x(0, 1200), y(0, 800), a(0, 2 * kPi);
        int hits = 0;
        for (int i = 0; i < 4000; ++i) {
            const Ray ray{{x(gen), y(gen)}, {std::cos(a(gen)), std::sin(a(gen))}};
            // brute force, no bounding boxes
            Scene::Intersection brute;
            for (const auto& e : s.elements()) {
                const Hit h = e->intersect(ray);
                if (h.hit && (!brute.element || h.t < brute.hit.t)) { brute.hit = h; brute.element = e.get(); }
            }
            const Scene::Intersection fast = s.nearestHit(ray);
            CHECK(fast.element == brute.element);
            if (fast.element && brute.element) CHECK(fast.hit.t == brute.hit.t);
            if (brute.element) ++hits;

            // bounds are conservative: every hit point lies in the element's box
            for (const auto& e : s.elements()) {
                const Hit h = e->intersect(ray);
                if (h.hit && !e->bounds().empty()) CHECK(e->bounds().contains(h.point, 1e-6));
            }
        }
        CHECK(hits > 1000);
    }

    void testContainsAndOverlap(test::Context& ctx) {
        Scene s = everythingScene();
        CHECK(s.overlappingPairs().empty());                // the bench above is laid out cleanly

        CHECK(s.solidAt({420, 420}) == s.find("ellipse"));
        CHECK(s.solidAt({600, 250}) == s.find("prism"));
        CHECK(s.solidAt({950, 250}) == s.find("thick"));
        CHECK(s.solidAt({20, 20}) == nullptr);
        CHECK(s.solidAt({80 - 30, 400}) == nullptr);        // inside the laser housing: not glass
        CHECK(s.find("laser")->contains({80 - 30, 400}));

        // the notched prism's notch is outside it
        const Transform2D t{.position = {800, 500}, .rotation = -0.4};
        CHECK(!s.find("notched")->contains(t.toWorld({-40, 0})));
        CHECK(s.find("notched")->contains(t.toWorld({30, 0})));

        // push a prism into the lens: exactly that pair is reported
        s.emplace<PrismElement>("intruder", Transform2D{.position = {460, 420}}, PrismElement::triangle(40, 40));
        const auto pairs = s.overlappingPairs();
        CHECK(pairs.size() == 1);
        if (pairs.size() == 1) {
            CHECK(pairs[0].first->name == "ellipse");
            CHECK(pairs[0].second->name == "intruder");
        }

        // blocks set flush against each other are allowed
        Scene flush(ClipRect{0, 0, 400, 400});
        flush.emplace<PrismElement>("a", Transform2D{.position = {100, 100}}, Polygon{{-20, -20}, {20, -20}, {20, 20}, {-20, 20}});
        flush.emplace<PrismElement>("b", Transform2D{.position = {140, 100}}, Polygon{{-20, -20}, {20, -20}, {20, 20}, {-20, 20}});
        CHECK(flush.overlappingPairs().empty());

        // a self-intersecting prism outline is refused
        bool threw = false;
        try { PrismElement bad("bad", Transform2D{}, Polygon{{0, 0}, {10, 10}, {10, 0}, {0, 10}}); }
        catch (const std::invalid_argument&) { threw = true; }
        CHECK(threw);
    }

    void testConcavePrismRefraction(test::Context& ctx) {
        // a block with a V notch cut into its left face; the beam enters through the notch
        Scene s(ClipRect{0, 0, 800, 600});
        s.emplace<PrismElement>("notched", Transform2D{.position = {400, 300}},
                                Polygon{{-60, -100}, {60, -100}, {60, 100}, {-60, 100}, {10, 0}});
        const BeamPath path = traceBeam(s, Ray{{100, 330}, {1, 0}});
        CHECK(path.events.size() >= 2);
        if (path.events.size() >= 2) {
            // entering through the lower notch face (from (-60,100) to (10,0)), not the hidden left edge
            const BeamEvent& in = path.events[0];
            CHECK(in.kind == BeamEvent::Kind::Refract);
            CHECK(in.n1 == 1.0 && in.n2 == 1.5);
            CHECK(geom::onSegment(in.point, Segment{{340, 400}, {410, 300}}, 1e-6));
            // Snell's law holds at both faces
            for (const BeamEvent& e : path.events) {
                if (e.kind != BeamEvent::Kind::Refract) continue;
                CHECK_NEAR(e.n1 * std::sin(e.incidenceDeg * kPi / 180.0), e.n2 * std::sin(e.refractionDeg * kPi / 180.0), 1e-9);
            }
            CHECK(path.events[1].n1 == 1.5 && path.events[1].n2 == 1.0);     // and it leaves through the far face
            CHECK_NEAR(path.events[1].point.x, 460.0, 1e-9);
        }
    }

    void testBeamStartingInGlass(test::Context& ctx) {
        Scene s(ClipRect{0, 0, 800, 600});
        s.emplace<LensElement>("lens", Transform2D{.position = {400, 300}}, 60.0);
        const BeamPath path = traceBeam(s, Ray{{400, 300}, {1, 0}});
        CHECK(!path.segments.empty() && path.segments[0].inGlass);
        CHECK(path.segments.size() >= 2 && !path.segments[1].inGlass);
        CHECK(!path.events.empty());
        if (!path.events.empty()) {
            CHECK(path.events[0].kind == BeamEvent::Kind::Refract);
            CHECK(path.events[0].n1 == 1.5 && path.events[0].n2 == 1.0);
            CHECK_VEC_NEAR(path.events[0].point, Vec2(460, 300), 1e-9);
        }
    }

    // where a beam's last segment crosses the line through `p` along `axis`
    Vec2 crossingWithAxis(const BeamPath& path, Vec2 p, Vec2 axis) {
        const BeamSegment& last = path.segments.back();
        const Vec2 d = last.b - last.a;
        const double t = (p - last.a).cross(axis) / d.cross(axis);
        return last.a + d * t;
    }

    void testThickLensFocusInScene(test::Context& ctx) {
        // a rotated, translated thick lens: paraxial beams parallel to its axis meet at backFocus()
        Scene s(ClipRect{0, 0, 1200, 800});
        const Transform2D t{.position = {500, 400}, .rotation = 0.35};
        auto& lens = s.emplace<ThickLensElement>("lens", t, geom::Lens(150, 150, 30));
        const Vec2 axis = t.dirToWorld({1, 0}).normalized();
        const Vec2 side = axis.perp();

        for (const double h : {0.4, -0.4}) {
            const Vec2 start = t.position - axis * 300.0 + side * h;
            const BeamPath path = traceBeam(s, Ray{start, axis});
            CHECK(path.events.size() == 2);
            CHECK(path.segments.size() == 3 && path.segments[1].inGlass);
            const Vec2 cross = crossingWithAxis(path, t.position, axis);
            CHECK((cross - lens.backFocus()).length() < 0.05);
        }
    }

    void testParabolicMirrorFocus(test::Context& ctx) {
        // beams parallel to the axis reflect through the focus, wherever the mirror is turned
        Scene s(ClipRect{0, 0, 1200, 800});
        const Transform2D t{.position = {700, 400}, .rotation = 0.5};
        s.emplace<CurvedMirrorElement>("dish", t, CurvedMirrorElement::parabola(120, 90));
        const Vec2 focus = t.toWorld({-120, 0});
        const Vec2 axis = t.dirToWorld({1, 0}).normalized();      // incoming beams travel along +x local

        for (const double h : {-80.0, -40.0, 10.0, 60.0}) {
            const Vec2 start = t.toWorld({-400, h});
            const BeamPath path = traceBeam(s, Ray{start, axis});
            CHECK(path.events.size() == 1 && path.events[0].kind == BeamEvent::Kind::Reflect);
            const BeamSegment& out = path.segments.back();
            CHECK(geom::onSegment(focus, Segment{out.a, out.b}, 1e-6));     // the reflected beam passes through the focus
        }
    }
}

int main() {
    return test::runTests("integration", {
        {"broad phase matches brute force; bounds are conservative", testBroadPhaseIsExact},
        {"containment and overlap checks", testContainsAndOverlap},
        {"concave prism refraction", testConcavePrismRefraction},
        {"beam starting inside glass", testBeamStartingInGlass},
        {"thick lens focus in a scene", testThickLensFocusInScene},
        {"parabolic mirror focus in a scene", testParabolicMirrorFocus},
    });
}
