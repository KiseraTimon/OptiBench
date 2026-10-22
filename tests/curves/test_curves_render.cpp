#include <vector>

#include "TestHarness.h"
#include "optibench/curves/CatmullRom.h"
#include "optibench/curves/CurveRender.h"

using namespace ob;
using namespace ob::curves;

namespace {
    // compile-time checks of the adapter concepts
    struct NotASink {};
    static_assert(LineSink<void (*)(Vec2, Vec2)>);
    static_assert(!LineSink<NotASink>);
    static_assert(Flattenable<Bezier>);
    static_assert(Flattenable<CompositeBezier>);
    static_assert(!Flattenable<Vec2>);

    // true if the pixel differs from the background
    bool lit(const Framebuffer& fb, int x, int y, Color bg) {
        const Color p = fb.getPixel(x, y);
        return p.r != bg.r || p.g != bg.g || p.b != bg.b;
    }

    void testRecordingStub(test::Context& ctx) {
        const Bezier c({{10, 10}, {60, 90}, {110, -30}, {160, 50}});
        std::vector<std::pair<Vec2, Vec2>> segments;
        drawCurve(c, 0.25, [&](Vec2 a, Vec2 b) { segments.emplace_back(a, b); });
        CHECK(segments.size() + 1 == c.flatten(0.25).size());
        CHECK(!segments.empty());
        if (segments.empty()) return;
        CHECK_VEC_NEAR(segments.front().first, c.front(), 0.0);
        CHECK_VEC_NEAR(segments.back().second, c.back(), 1e-12);
        bool chained = true;
        for (std::size_t i = 1; i < segments.size(); ++i)
            chained = chained && (segments[i].first - segments[i - 1].second).length() == 0.0;
        CHECK(chained);

        // a mutable functor works as a sink too
        struct Counter {
            int n = 0;
            void operator()(Vec2, Vec2) { ++n; }
        } counter;
        drawPolyline(std::vector<Vec2>{{0, 0}, {1, 0}, {2, 0}}, counter);
        CHECK(counter.n == 2);
        drawPolyline(std::vector<Vec2>{{0, 0}}, counter);
        CHECK(counter.n == 2);
    }

    void testFramebufferDrawing(test::Context& ctx) {
        const CatmullRom cr({{20, 80}, {60, 20}, {100, 70}, {140, 30}}, Parametrization::Centripetal);
        for (bool smooth : {false, true}) {
            Framebuffer fb(160, 100, colors::Black);
            drawCurve(fb, cr.toBezier(), colors::White, 0.25, smooth);
            // the curve passes through its points, so those pixels must be lit
            for (const Vec2& p : cr.points()) CHECK(lit(fb, static_cast<int>(p.x), static_cast<int>(p.y), colors::Black));
            // and points on the curve between them too
            for (double u = 0.1; u < 3.0; u += 0.2) {
                const Vec2 p = cr.evaluate(u);
                bool near = false;
                for (int dy = -1; dy <= 1; ++dy)
                    for (int dx = -1; dx <= 1; ++dx)
                        near = near || lit(fb, static_cast<int>(std::lround(p.x)) + dx, static_cast<int>(std::lround(p.y)) + dy, colors::Black);
                CHECK(near);
            }
            CHECK(!lit(fb, 150, 95, colors::Black));
        }

        Framebuffer fb(100, 100, colors::Black);
        drawControlPolygon(fb, std::vector<Vec2>{{10, 10}, {90, 10}, {90, 90}}, colors::Grey, colors::Yellow, 3);
        CHECK(lit(fb, 13, 10, colors::Black)); // circle marker around (10, 10)
        // the edge is dashed (4 on, 3 off): roughly 4/7 of its pixels are lit
        int on = 0;
        for (int y = 20; y < 83; ++y) on += lit(fb, 90, y, colors::Black) ? 1 : 0;
        CHECK(on > 63 * 2 / 5 && on < 63 * 3 / 4);
        // drawing off-screen curves must not crash
        drawCurve(fb, Bezier({{-500, -500}, {1000, 50}, {50, 2000}}), colors::Red);
        CHECK(true);
    }
}

int main() {
    return test::runTests("curves: render adapter", {
        {"recording stub sink", testRecordingStub},
        {"framebuffer drawing", testFramebufferDrawing},
    });
}
