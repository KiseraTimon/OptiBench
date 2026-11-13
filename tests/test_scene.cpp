#include <cmath>
#include <cstdio>

#include "optibench/Scene.h"
#include "optibench/Tracer.h"
#include "optibench/optics.h"

#define CHECK(cond) do { if (!(cond)) { std::printf("FAIL %s:%d  %s\n", __FILE__, __LINE__, #cond); return 1; } } while (0)

using namespace ob;

static bool near(double a, double b, double eps = 1e-6) { return std::fabs(a - b) < eps; }
static bool near(Vec2 a, Vec2 b, double eps = 1e-6) { return near(a.x, b.x, eps) && near(a.y, b.y, eps); }

int main() {
    // ---------- Transform2D: lecture worked examples ----------
    {
        // "a point (2,3) is scaled by 2 and then translated by (4,1)"  ->  (8,7)
        Transform2D t{.position = {4, 1}, .scale = {2, 2}};
        CHECK(near(t.toWorld({2, 3}), {8, 7}));

        // "(2,1) rotated 90 degrees anticlockwise about the origin"  ->  (-1,2)
        Transform2D r{.rotation = kPi / 2};
        CHECK(near(r.toWorld({2, 1}), {-1, 2}));

        // "(6,7) rotated 90 degrees"  ->  (-7,6)
        CHECK(near(r.toWorld({6, 7}), {-7, 6}));

        // composite order T.R.S: scale first, then rotate, then translate
        Transform2D c{.position = {10, 0}, .rotation = kPi / 2, .scale = {2, 1}};
        CHECK(near(c.toWorld({1, 0}), {10, 2}));    // (1,0) -> (2,0) -> (0,2) -> (10,2)

        // toLocal undoes toWorld, even with rotation and non-uniform scale
        Transform2D g{.position = {33, -7}, .rotation = 0.9, .scale = {2.5, 0.4}};
        CHECK(near(g.toLocal(g.toWorld({3.2, -8.1})), {3.2, -8.1}));
        CHECK(near(g.dirToLocal(g.dirToWorld({1.5, 2.0})), {1.5, 2.0}));
    }

    // ---------- normals stay perpendicular under non-uniform scale ----------
    {
        Transform2D t{.rotation = 0.6, .scale = {3.0, 1.0}};
        const Vec2 localPoint{std::cos(0.8), std::sin(0.8)};          // on the unit circle
        const Vec2 localTangent{-localPoint.y, localPoint.x};
        const Vec2 worldTangent = t.dirToWorld(localTangent);
        const Vec2 worldNormal = t.normalToWorld(localPoint);        // circle normal = the point itself
        CHECK(near(worldTangent.dot(worldNormal), 0.0));
        CHECK(near(worldNormal.length(), 1.0));
    }

    // ---------- optics: reflect / refract ----------
    {
        // horizontal ray hitting a 45-degree surface turns 90 degrees
        const Vec2 r = reflect({1, 0}, Vec2{-1, 1}.normalized());
        CHECK(near(r, {0, 1}));

        // Snell: n1 sin(t1) = n2 sin(t2)
        const Vec2 n{0, -1};                                          // faces against the ray
        const Vec2 d = Vec2{std::sin(0.5), std::cos(0.5)}.normalized();
        Vec2 out;
        CHECK(refract(d, n, 1.0, 1.5, out));
        CHECK(near(1.0 * std::sin(0.5), 1.5 * std::fabs(out.x), 1e-9));

        // total internal reflection: glass -> air beyond the critical angle (41.8 deg for n = 1.5)
        const double steep = 60.0 * kPi / 180.0;
        CHECK(!refract({std::sin(steep), std::cos(steep)}, n, 1.5, 1.0, out));
        const double shallow = 30.0 * kPi / 180.0;
        CHECK(refract({std::sin(shallow), std::cos(shallow)}, n, 1.5, 1.0, out));
    }

    // ---------- elements ----------
    {
        // a 45-degree mirror turns a rightward ray upward (screen up = -y)
        FlatElement m("m", Transform2D{.position = {100, 100}, .rotation = -kPi / 4}, 30.0, Material::mirror());
        const Hit h = m.intersect(Ray{{0, 100}, {1, 0}});
        CHECK(h.hit && near(h.point, {100, 100}, 1e-6));
        CHECK(near(reflect({1, 0}, h.normal), {0, -1}, 1e-6));

        // lens: a scaled + rotated circle is an ellipse; every hit lies on it
        LensElement lens("l", Transform2D{.position = {200, 200}, .rotation = 0.7, .scale = {2.0, 1.0}}, 40.0);
        const Hit e = lens.intersect(Ray{{0, 170}, {1, 0}});
        CHECK(e.hit);
        CHECK(near(lens.transform.toLocal(e.point).length(), 40.0, 1e-6));
        CHECK(e.normal.dot(e.point - lens.transform.position) > 0.0);          // outward
        CHECK(!lens.intersect(Ray{{0, 0}, {1, 0}}).hit);                      // misses
        CHECK(near(lens.radiusX(), 80.0) && near(lens.radiusY(), 40.0));

        // prism: normals point out of the solid
        PrismElement prism("p", Transform2D{.position = {300, 300}}, PrismElement::triangle(100, 100));
        const Hit p = prism.intersect(Ray{{200, 320}, {1, 0}});
        CHECK(p.hit);
        CHECK(p.normal.dot(Vec2{1, 0}) < 0.0);                                  // left face: outward = left
        const Hit q = prism.intersect(Ray{{400, 320}, {-1, 0}});
        CHECK(q.hit && q.normal.dot(Vec2{1, 0}) > 0.0);                         // right face: outward = right

        // laser emits along its local +x, rotated
        LaserSource laser("s", Transform2D{.position = {5, 5}, .rotation = kPi / 2}, colors::Red);
        CHECK(near(laser.emissionRay().direction, {0, 1}, 1e-9));
        CHECK(!laser.intersect(Ray{{0, 5}, {1, 0}}).hit);
    }

    // ---------- scene lookup ----------
    {
        Scene s(ClipRect{0, 0, 100, 100});
        s.emplace<LaserSource>("laser", Transform2D{}, colors::Red);
        s.emplace<LensElement>("lens", Transform2D{}, 10.0);
        CHECK(s.elements().size() == 2);
        CHECK(s.sources().size() == 1);
        CHECK(s.find("lens") != nullptr && s.find("nope") == nullptr);
        CHECK(s.findAs<LensElement>("lens") != nullptr);
        CHECK(s.findAs<FlatElement>("lens") == nullptr);
    }

    // ---------- tracer: the periscope + lens + screen path ----------
    {
        Scene s(ClipRect{0, 0, 800, 500});
        s.emplace<FlatElement>("low", Transform2D{.position = {260, 360}, .rotation = -kPi / 4}, 38.0, Material::mirror());
        s.emplace<FlatElement>("top", Transform2D{.position = {260, 150}, .rotation = 3 * kPi / 4}, 38.0, Material::mirror());
        s.emplace<FlatElement>("screen", Transform2D{.position = {700, 265}, .rotation = -kPi / 2}, 170.0, Material::screen());

        const BeamPath path = traceBeam(s, Ray{{125, 360}, {1, 0}});
        CHECK(path.hitScreen);
        CHECK(path.events.size() == 3);                       // reflect, reflect, absorb
        CHECK(near(path.end, {700, 150}, 1e-4));              // periscope returns the beam to y = 150
        CHECK(near(path.events[0].incidenceDeg, 45.0, 1e-6));

        // add a lens: Snell's law must hold at BOTH surfaces
        s.emplace<LensElement>("lens", Transform2D{.position = {430, 180}}, 62.0);
        const BeamPath bent = traceBeam(s, Ray{{125, 360}, {1, 0}});
        CHECK(bent.hitScreen);
        int refractions = 0;
        for (const BeamEvent& ev : bent.events) {
            if (ev.kind != BeamEvent::Kind::Refract) continue;
            ++refractions;
            CHECK(near(ev.n1 * std::sin(ev.incidenceDeg * kPi / 180.0),
                       ev.n2 * std::sin(ev.refractionDeg * kPi / 180.0), 1e-9));
        }
        CHECK(refractions == 2);                              // in and out of the lens
        CHECK(bent.segments[2].inGlass || bent.segments[3].inGlass);
    }

    // ---------- tracer: total internal reflection inside a wide prism ----------
    {
        Scene s(ClipRect{0, 0, 800, 500});
        s.emplace<PrismElement>("wide", Transform2D{.position = {400, 215}}, PrismElement::triangle(120, 110));
        const BeamPath path = traceBeam(s, Ray{{300, 195}, {1, 0.12}});
        bool sawTir = false;
        for (const BeamEvent& ev : path.events) sawTir |= ev.kind == BeamEvent::Kind::TotalInternalReflection;
        CHECK(sawTir);
    }

    std::puts("test_scene: PASS");
    return 0;
}
