/*
 * OptiBench Live: the final scene in a window, running in real time and editable with the mouse.
 *
 * The same bench, tracer and Studio renderer as the offline application (main.cpp); every frame is
 * re-traced from scratch, so anything you move changes the light immediately.
 *
 *   LMB drag            move an element (refused if it would overlap another one: geometry module, SAT)
 *   wheel / RMB drag    rotate the element under the pointer (Shift: fine, Ctrl: coarse)
 *   P                   edit the lens's Bezier motion path (curves module, CurveEditor):
 *                       drag the control points, Ctrl+click on the path adds one, RMB on one removes it
 *   Space               play / pause      , .   step one frame      - =   slower / faster
 *   Del                 give the selected element back to the animation
 *   Tab                 select the next element      arrows  nudge it (Shift: 10 px)   Q / E  rotate it
 *   R reset   B bounding boxes   S screenshot   H help   Esc deselect / quit
 *
 * Run from the repository root:  ./build/OptiBench_live
 */

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <filesystem>
#include <map>
#include <optional>
#include <span>
#include <string>
#include <thread>
#include <vector>

#include "Bench.h"
#include "Studio.h"
#include "Window.h"
#include "optibench/Animator.h"
#include "optibench/DoubleBuffer.h"
#include "optibench/Line.h"
#include "optibench/curves/CurveEditor.h"
#include "optibench/geometry/Geometry.h"
#include "optibench/render/Text.h"

using namespace ob;
using namespace ob::app;
using ob::live::Event;
namespace keys = ob::live::keys;
using render::Align;
using render::Font;

namespace {
    constexpr int kW = Layout::kWidth, kH = Layout::kHeight;
    constexpr double kStepFps = 15.0;               // the offline frame rate: , and . step by one of its frames
    constexpr double kPickRadius = 10.0;            // px around an element's outline that still selects it
    constexpr double kHandleRadius = 9.0;           // px: grabbing a path control point

    constexpr Color kInk{226, 232, 245};
    constexpr Color kMuted{128, 140, 168};
    constexpr Color kChip{10, 13, 22};
    constexpr Color kSelect{110, 225, 255};
    constexpr Color kHover{150, 165, 200};
    constexpr Color kBlocked{255, 96, 96};
    constexpr Color kPinned{255, 184, 76};
    constexpr Color kGood{120, 230, 150};

    double deg(double rad) { return rad * 180.0 / kPi; }
    double rad(double degrees) { return degrees * kPi / 180.0; }

    std::string fmt(const char* format, double a, double b = 0.0, double c = 0.0) {
        char buf[160];
        std::snprintf(buf, sizeof buf, format, a, b, c);
        return buf;
    }

    // the element's body as a polygon (its outline, or its bounding box if it has none)
    std::vector<Vec2> shapeOf(const OpticalElement& e) {
        std::vector<Vec2> o = e.outline();
        if (o.size() >= 3) return o;
        const geom::Aabb b = e.bounds();
        if (b.empty()) return {};
        return {b.min, {b.max.x, b.min.y}, b.max, {b.min.x, b.max.y}};
    }

    // distance from p to the element's body (0 inside)
    double distanceTo(const OpticalElement& e, Vec2 p) {
        const std::vector<Vec2> s = shapeOf(e);
        if (s.size() < 3) return 1e9;
        return std::max(0.0, geom::signedDistance(s, p));
    }

    void polyline(Framebuffer& fb, std::span<const Vec2> pts, Color c, bool closed) {
        if (pts.size() < 2) return;
        for (std::size_t i = 0; i + 1 < pts.size(); ++i) drawLineWu(fb, pts[i], pts[i + 1], c);
        if (closed) drawLineWu(fb, pts.back(), pts.front(), c);
    }

    // a two-pixel outline, grown slightly so it sits just outside the element's own edge
    void outlineOf(Framebuffer& fb, const OpticalElement& e, Color c) {
        std::vector<Vec2> s = shapeOf(e);
        if (s.size() < 3) return;
        Vec2 centre{0, 0};
        for (const Vec2& p : s) centre = centre + p;
        centre = centre / static_cast<double>(s.size());
        for (double grow : {2.0, 3.0}) {
            std::vector<Vec2> g;
            g.reserve(s.size());
            for (const Vec2& p : s) {
                const Vec2 d = p - centre;
                const double len = d.length();
                g.push_back(len > 1e-9 ? p + d / len * grow : p);
            }
            polyline(fb, g, c, true);
        }
    }

    void circle(Framebuffer& fb, Vec2 c, double r, Color color, bool filled) {
        if (filled) {
            const int reach = static_cast<int>(std::ceil(r));
            for (int y = -reach; y <= reach; ++y)
                for (int x = -reach; x <= reach; ++x)
                    if (x * x + y * y <= r * r) fb.setPixel(static_cast<int>(c.x) + x, static_cast<int>(c.y) + y, color);
        }
        constexpr int kSides = 28;
        for (int i = 0; i < kSides; ++i) {
            const double a0 = 2 * kPi * i / kSides, a1 = 2 * kPi * (i + 1) / kSides;
            drawLineWu(fb, c + Vec2{std::cos(a0), std::sin(a0)} * r, c + Vec2{std::cos(a1), std::sin(a1)} * r, color);
        }
    }

    void dashed(Framebuffer& fb, Vec2 a, Vec2 b, Color c) {
        const double len = (b - a).length();
        if (len < 1e-6) return;
        const Vec2 u = (b - a) / len;
        for (double s = 0.0; s < len; s += 8.0) drawLineWu(fb, a + u * s, a + u * std::min(len, s + 4.0), c);
    }

    // the lens's motion path, exactly as Bench.cpp binds it (so an unedited path changes nothing)
    MotionPath lensMotion(const curves::CompositeBezier& curve) {
        return MotionPath(curve, 0.0, kDuration, CubicBezierEasing::easeInOut(), true);
    }

    class LiveBench {
    public:
        LiveBench()
            : scene_(buildBench()),
              sources_(benchSources()),
              buffers_(kW, kH),
              anim_(scene_, buffers_),
              tracks_(animateBench(anim_)),
              studio_(StudioOptions{showBounds_, kDuration, kStepFps}, tracks_),
              frame_(kW, kH),
              editor_(std::vector<Vec2>(lensPath().controlPoints().begin(), lensPath().controlPoints().end()),
                      curves::builders::bezier()),
              path_(lensMotion(editor_.curve())) {
            for (const auto& e : scene_.elements()) initial_[e->name] = e->transform;

            // after the animation bindings: the edited lens path, then the elements the user has placed
            anim_.addUpdate([this](double t) {
                if (pathEdited_ && !pinned_.count("lens"))
                    if (OpticalElement* lens = scene_.find("lens")) lens->transform.position = path_.sample(t);
            });
            anim_.addUpdate([this](double) {
                for (const auto& [name, transform] : pinned_)
                    if (OpticalElement* e = scene_.find(name)) e->transform = transform;
            });
            anim_.setRenderer([](Framebuffer&, const FrameInfo&) {});    // drawn below, with a stable frame number
        }

        int run() {
            ob::live::Window window("OptiBench Live", kW, kH);
            using clock = std::chrono::steady_clock;
            auto last = clock::now();
            bool dirty = true;
            while (!quit_) {
                Event e;
                while (window.poll(e)) dirty |= handle(e);
                if (quit_) break;

                const auto now = clock::now();
                const double dt = std::min(0.1, std::chrono::duration<double>(now - last).count());
                last = now;

                if (playing_ || dirty) {
                    advance(playing_ ? dt * speed_ : 0.0);
                    draw();
                    window.present(frame_);
                    dirty = false;
                    if (dt > 0.0) fps_ = fps_ * 0.9 + 0.1 / dt;
                } else {
                    std::this_thread::sleep_for(std::chrono::milliseconds(8));
                }
            }
            return 0;
        }

        /**
         * headless check of the interaction, for ctest: scripted mouse and key events go through the
         * same handle() as the window's, the results are verified, and screenshots are written to dir.
         * Returns the number of failed checks.
         */
        int selfTest(const std::string& dir) {
            std::filesystem::create_directories(dir);
            int failures = 0;
            const auto check = [&](bool ok, const char* what) {
                std::printf("  %-58s %s\n", what, ok ? "ok" : "FAILED");
                if (!ok) ++failures;
            };
            const auto mouse = [&](Event::Type type, Vec2 p, int button = 1, bool ctrl = false) {
                Event e;
                e.type = type;
                e.x = static_cast<int>(p.x);
                e.y = static_cast<int>(p.y);
                e.button = button;
                e.ctrl = ctrl;
                handle(e);
            };
            const auto drag = [&](Vec2 from, Vec2 to, int button = 1) {
                mouse(Event::Type::MouseMove, from);
                mouse(Event::Type::MouseDown, from, button);
                for (int i = 1; i <= 8; ++i) mouse(Event::Type::MouseMove, from + (to - from) * (i / 8.0));
                mouse(Event::Type::MouseUp, to, button);
            };
            const auto key = [&](int k, bool ctrl = false) {
                Event e;
                e.type = Event::Type::Key;
                e.key = k;
                e.ctrl = ctrl;
                handle(e);
            };
            const auto shot = [&](const char* name) {
                advance(0.0);
                draw();
                return frame_.saveBMP((std::filesystem::path(dir) / name).string());
            };

            playing_ = false;
            time_ = 2.0;
            check(shot("live_0_start.bmp"), "renders the bench");
            check(report_.overlaps == 0 && report_.raysOnScreen == report_.rays, "starts valid: no overlaps, every ray lands");

            // move the white lane's turning mirror: the light must follow
            const Vec2 m0 = scene_.find("mirrorWhite")->transform.position;
            drag(m0, m0 + Vec2{0, 24});
            advance(0.0);
            const Vec2 m1 = scene_.find("mirrorWhite")->transform.position;
            check(selected_ == "mirrorWhite", "a click selects the element under the pointer");
            check(std::abs(m1.y - m0.y - 24) < 1.5 && std::abs(m1.x - m0.x) < 1.5, "dragging moves it with the pointer");
            check(pinned_.count("mirrorWhite") == 1, "a moved element is kept out of the animation");
            check(shot("live_1_mirror_moved.bmp"), "renders after the move");

            // rotate the dispersing prism with the wheel (Ctrl: 5 degrees a notch)
            const double r0 = scene_.find("prism")->transform.rotation;
            mouse(Event::Type::MouseMove, scene_.find("prism")->transform.position);
            {
                Event w;
                w.type = Event::Type::Wheel;
                w.wheel = 1;
                w.ctrl = true;
                w.x = static_cast<int>(mouse_.x);
                w.y = static_cast<int>(mouse_.y);
                handle(w);
            }
            check(std::abs(scene_.find("prism")->transform.rotation - r0 - rad(5.0)) < 1e-9, "the wheel rotates the element under the pointer");

            // glass may not be pushed into glass: the move is refused and nothing changes
            key('r');
            playing_ = false;
            time_ = 2.0;
            advance(0.0);
            const Vec2 n0 = scene_.find("notched")->transform.position;
            const Vec2 target = scene_.find("prism")->transform.position;
            mouse(Event::Type::MouseMove, n0);
            mouse(Event::Type::MouseDown, n0);
            mouse(Event::Type::MouseMove, target);
            const bool blocked = !blockedBy_.empty();
            check(blocked, "moving onto another element is detected (SAT overlap)");
            check((scene_.find("notched")->transform.position - target).length() > 20, "...and refused");
            check(shot("live_2_blocked.bmp"), "renders the refusal");
            mouse(Event::Type::MouseUp, target);
            advance(0.0);
            check(scene_.overlappingPairs().empty(), "the bench never ends up overlapping");

            // edit the lens's Bezier motion path (curves module)
            key('p');
            const Vec2 p1 = editor_.points()[1];
            drag(p1, p1 + Vec2{0, -30});
            check(pathEdited_ && std::abs(editor_.points()[1].y - (p1.y - 30)) < 1.5, "dragging a control point reshapes the lens path");
            advance(0.0);
            check((scene_.find("lens")->transform.position - path_.sample(time_)).length() < 1e-9, "the lens follows the edited path");
            const std::size_t before = editor_.points().size();
            mouse(Event::Type::MouseDown, editor_.curve().evaluate(editor_.curve().paramMax() * 0.25), 1, true);
            mouse(Event::Type::MouseUp, mouse_);
            check(editor_.points().size() == before + 1, "Ctrl+click on the path adds a control point");
            check(shot("live_3_path_edit.bmp"), "renders the path editor");

            // playback controls
            key(keys::Space);
            check(playing_, "Space plays");
            key(keys::Space);
            const double t0 = time_;
            key('.');
            check(!playing_ && std::abs(time_ - std::fmod(t0 + 1.0 / kStepFps, kDuration)) < 1e-9, "'.' steps one frame");
            key('r');
            advance(0.0);
            check(pinned_.empty() && !pathEdited_ && (scene_.find("mirrorWhite")->transform.position - m0).length() < 1e-9, "R restores the bench");

            std::printf("OptiBench_live self-test: %s\n", failures == 0 ? "all checks passed" : "FAILED");
            return failures;
        }

    private:
        Scene scene_;
        std::vector<SourceSpec> sources_;
        DoubleBuffer buffers_;
        Animator anim_;
        std::vector<KeyframeTrack> tracks_;
        bool showBounds_ = true;
        Studio studio_;
        Framebuffer frame_;
        FrameReport report_;

        std::map<std::string, Transform2D> initial_;    // for R
        std::map<std::string, Transform2D> pinned_;     // elements the user has placed: the animation leaves them alone

        curves::CurveEditor editor_;
        MotionPath path_;
        bool pathEdited_ = false;
        bool pathMode_ = false;

        bool playing_ = true;
        double speed_ = 1.0;
        double time_ = 0.0;
        double fps_ = 0.0;
        bool help_ = false;
        bool quit_ = false;
        int shots_ = 0;
        std::string toast_;
        std::chrono::steady_clock::time_point toastUntil_{};

        std::string selected_, hovered_;
        enum class Drag { None, Move, Rotate, PathPoint } drag_ = Drag::None;
        Vec2 grabOffset_{0, 0};
        double grabAngle_ = 0.0, grabRotation_ = 0.0;
        std::string blockedBy_;
        Vec2 mouse_{0, 0};

        // ------------------------------------------------------------------ animation and drawing

        void advance(double dt) {
            time_ = std::fmod(time_ + dt, kDuration);     // the animation loops every kDuration seconds
            anim_.clock().reset(time_);
            anim_.clock().pause();
            anim_.step(0.0);                               // bindings and updates at time_, then swap
        }

        void draw() {
            const FrameInfo info{static_cast<int>(std::lround(time_ * kStepFps)), time_, 1.0 / kStepFps};
            report_ = studio_.render(frame_, scene_, sources_, info);
            drawOverlay();
        }

        OpticalElement* element(const std::string& name) { return name.empty() ? nullptr : scene_.find(name); }

        void drawOverlay() {
            // status in the header, between the title and the clock
            const std::string state = std::string(playing_ ? "PLAYING" : "PAUSED") + fmt("  %.2gx", speed_) +
                                      (playing_ ? fmt("   %.0f fps", fps_) : std::string()) + (pathMode_ ? "   PATH EDIT" : "");
            render::drawLabel(frame_, 720, 19, "LIVE  " + state, playing_ ? kGood : kPinned, kChip, Font::Regular,
                              Align::Center, 0.9);

            if (pathMode_) drawPath();

            if (const OpticalElement* h = element(hovered_); h && hovered_ != selected_ && drag_ == Drag::None)
                outlineOf(frame_, *h, kHover);
            if (const OpticalElement* s = element(selected_)) {
                const bool blocked = !blockedBy_.empty();
                outlineOf(frame_, *s, blocked ? kBlocked : kSelect);
                const geom::Aabb b = s->bounds();
                // above the element (its own name label sits below it); below if there is no room
                Vec2 at = b.empty() ? s->transform.position : Vec2{(b.min.x + b.max.x) / 2, b.min.y - 26};
                if (!b.empty() && at.y < Layout::kBench.ymin + 4) at.y = b.max.y + 26;
                std::string text = s->name + fmt("   x %.0f  y %.0f   %.1f deg", s->transform.position.x,
                                                 s->transform.position.y, deg(s->transform.rotation));
                if (pinned_.count(s->name)) text += "   placed (Del: animate)";
                if (blocked) text = "blocked: would overlap " + blockedBy_;
                render::drawLabel(frame_, std::clamp(static_cast<int>(at.x), 200, kW - 200), static_cast<int>(std::min(at.y, kH - 130.0)), text,
                                  blocked ? kBlocked : kSelect, kChip, Font::Small, Align::Center, 0.85);
            }

            if (help_) drawHelp();
            else render::drawLabel(frame_, 34, 72, "H  help", kMuted, kChip, Font::Small, Align::Left, 0.8);

            if (!toast_.empty() && std::chrono::steady_clock::now() < toastUntil_)
                render::drawLabel(frame_, kW / 2, Layout::kHudTop - 64, toast_, kInk, kChip, Font::Regular, Align::Center, 0.9);
        }

        void drawPath() {
            const auto pts = editor_.points();
            for (std::size_t i = 0; i + 1 < pts.size(); ++i) dashed(frame_, pts[i], pts[i + 1], kMuted);
            const curves::CompositeBezier& c = editor_.curve();
            std::vector<Vec2> samples;
            for (int i = 0; i <= 96; ++i) samples.push_back(c.evaluate(c.paramMax() * i / 96.0));
            polyline(frame_, samples, kPinned, false);
            const std::optional<std::size_t> hot = editor_.dragged() ? editor_.dragged() : editor_.hitTest(mouse_, kHandleRadius);
            for (std::size_t i = 0; i < pts.size(); ++i) {
                const bool h = hot && *hot == i;
                circle(frame_, pts[i], h ? 6.0 : 4.5, h ? kSelect : kPinned, true);
                circle(frame_, pts[i], h ? 7.0 : 5.5, kChip, false);
                render::drawText(frame_, static_cast<int>(pts[i].x) + 8, static_cast<int>(pts[i].y) + 6,
                                 "P" + std::to_string(i), kPinned, Font::Small);
            }
            render::drawLabel(frame_, static_cast<int>(pts.front().x), static_cast<int>(pts.front().y) - 34,
                              "lens path: Bezier, degree " + std::to_string(pts.size() - 1), kPinned, kChip,
                              Font::Small, Align::Left, 0.85);
        }

        void drawHelp() {
            const char* lines[] = {
                "LMB drag          move an element",
                "wheel / RMB drag  rotate it (Shift fine, Ctrl coarse)",
                "P                 edit the lens path (Bezier)",
                "  Ctrl+click adds a point, RMB removes one",
                "Space  play/pause    , .  step    - =  speed",
                "Del  back to animation    Tab  next element",
                "arrows nudge   Q E rotate   R reset",
                "B  bounds   S  screenshot   H  hide   Esc  quit",
            };
            const int x = 34, y0 = 72, lh = 17;
            for (int y = y0 - 6; y < y0 + 8 * lh + 4; ++y)        // a solid panel, so the bench does not show through
                for (int px = x - 8; px < x + 400; ++px) frame_.setPixel(px, y, kChip);
            for (int i = 0; i < 8; ++i)
                render::drawText(frame_, x, y0 + i * lh, lines[i], i == 0 ? kInk : kMuted, Font::Small);
        }

        void notify(const std::string& text) {
            toast_ = text;
            toastUntil_ = std::chrono::steady_clock::now() + std::chrono::milliseconds(1800);
        }

        // ------------------------------------------------------------------ editing

        std::string pick(Vec2 p) const {
            std::string best;
            double bestD = kPickRadius;
            double bestArea = 1e18;
            for (const auto& e : scene_.elements()) {
                const double d = distanceTo(*e, p);
                const geom::Aabb b = e->bounds();
                const double area = b.empty() ? 1e18 : (b.max.x - b.min.x) * (b.max.y - b.min.y);
                // prefer the closest; among elements under the pointer, the smallest (a mirror over a beam etc.)
                if (d < bestD - 1e-9 || (d <= 1e-9 && bestD <= 1e-9 && area < bestArea)) {
                    best = e->name;
                    bestD = d;
                    bestArea = area;
                }
            }
            return best;
        }

        // the name of an element the given one overlaps, or "" (geometry module: polygon SAT overlap)
        std::string overlapOf(const std::string& name) const {
            for (const auto& [a, b] : scene_.overlappingPairs()) {
                if (a->name == name) return b->name;
                if (b->name == name) return a->name;
            }
            return "";
        }

        // places an element; refused (and nothing changes) if it would leave the bench or overlap another
        bool place(const std::string& name, Transform2D t) {
            OpticalElement* e = element(name);
            if (!e) return false;
            const ClipRect& bench = Layout::kBench;
            t.position.x = std::clamp(t.position.x, bench.xmin + 8, bench.xmax - 8);
            t.position.y = std::clamp(t.position.y, bench.ymin + 8, bench.ymax - 8);
            const Transform2D before = e->transform;
            e->transform = t;
            const std::string hit = overlapOf(name);
            if (!hit.empty()) {
                e->transform = before;
                blockedBy_ = hit;
                return false;
            }
            blockedBy_.clear();
            pinned_[name] = t;
            return true;
        }

        void rotateBy(const std::string& name, double radians) {
            if (const OpticalElement* e = element(name)) {
                Transform2D t = e->transform;
                t.rotation += radians;
                place(name, t);
            }
        }

        void nudge(Vec2 d) {
            if (const OpticalElement* e = element(selected_)) {
                Transform2D t = e->transform;
                t.position = t.position + d;
                place(selected_, t);
            }
        }

        void reset() {
            pinned_.clear();
            for (const auto& [name, t] : initial_)
                if (OpticalElement* e = scene_.find(name)) e->transform = t;
            editor_ = curves::CurveEditor(std::vector<Vec2>(lensPath().controlPoints().begin(), lensPath().controlPoints().end()),
                                          curves::builders::bezier());
            path_ = lensMotion(editor_.curve());
            pathEdited_ = false;
            selected_.clear();
            blockedBy_.clear();
            time_ = 0.0;
            speed_ = 1.0;
            playing_ = true;
            notify("bench reset");
        }

        void pathChanged() {
            path_ = lensMotion(editor_.curve());
            pathEdited_ = true;
            pinned_.erase("lens");      // the lens follows its (new) path again
        }

        void screenshot() {
            std::error_code ec;
            std::filesystem::create_directories("output", ec);
            char name[48];
            std::snprintf(name, sizeof name, "output/live_%03d.bmp", shots_++);
            notify(frame_.saveBMP(name) ? std::string("saved ") + name : std::string("could not write ") + name);
        }

        // returns true if the frame must be redrawn
        bool handle(const Event& e) {
            const double fine = e.shift ? 0.1 : (e.ctrl ? 5.0 : 1.0);   // degrees per wheel notch / key press
            switch (e.type) {
                case Event::Type::Quit: quit_ = true; return false;
                case Event::Type::Redraw: return true;

                case Event::Type::MouseMove: {
                    mouse_ = {e.x + 0.5, e.y + 0.5};
                    if (drag_ == Drag::Move) {
                        if (const OpticalElement* s = element(selected_)) {
                            Transform2D t = s->transform;
                            t.position = mouse_ + grabOffset_;
                            place(selected_, t);
                        }
                        return true;
                    }
                    if (drag_ == Drag::Rotate) {
                        if (const OpticalElement* s = element(selected_)) {
                            const Vec2 d = mouse_ - s->transform.position;
                            Transform2D t = s->transform;
                            t.rotation = grabRotation_ + (std::atan2(d.y, d.x) - grabAngle_);
                            place(selected_, t);
                        }
                        return true;
                    }
                    if (drag_ == Drag::PathPoint) {
                        if (editor_.dragTo(mouse_)) pathChanged();
                        return true;
                    }
                    const std::string h = pick(mouse_);
                    const bool changed = h != hovered_;
                    hovered_ = h;
                    return changed || pathMode_;
                }

                case Event::Type::MouseDown: {
                    mouse_ = {e.x + 0.5, e.y + 0.5};
                    blockedBy_.clear();
                    if (pathMode_) {
                        if (e.button == 1 && e.ctrl) {
                            if (editor_.insertNear(mouse_)) { pathChanged(); notify("control point added"); }
                            return true;
                        }
                        if (const auto i = editor_.hitTest(mouse_, kHandleRadius)) {
                            if (e.button == 3) {
                                if (editor_.remove(*i)) { pathChanged(); notify("control point removed"); }
                                else notify("a path needs at least 2 points");
                                return true;
                            }
                            if (e.button == 1 && editor_.beginDrag(mouse_, kHandleRadius)) {
                                drag_ = Drag::PathPoint;
                                return true;
                            }
                        }
                    }
                    const std::string hit = pick(mouse_);
                    selected_ = hit;
                    const OpticalElement* s = element(hit);
                    if (!s) return true;
                    if (e.button == 1) {
                        drag_ = Drag::Move;
                        grabOffset_ = s->transform.position - mouse_;
                    } else if (e.button == 3) {
                        drag_ = Drag::Rotate;
                        const Vec2 d = mouse_ - s->transform.position;
                        grabAngle_ = std::atan2(d.y, d.x);
                        grabRotation_ = s->transform.rotation;
                    }
                    return true;
                }

                case Event::Type::MouseUp:
                    if (drag_ == Drag::PathPoint) editor_.endDrag();
                    drag_ = Drag::None;
                    blockedBy_.clear();
                    return true;

                case Event::Type::Wheel: {
                    const std::string target = !hovered_.empty() ? hovered_ : selected_;
                    if (target.empty()) return false;
                    selected_ = target;
                    rotateBy(target, rad(fine) * e.wheel);
                    return true;
                }

                case Event::Type::Key:
                    switch (e.key) {
                        case keys::Escape:
                            if (!selected_.empty() || pathMode_) { selected_.clear(); pathMode_ = false; }
                            else quit_ = true;
                            return true;
                        case keys::Space: playing_ = !playing_; return true;
                        case ',': playing_ = false; time_ = std::fmod(time_ - 1.0 / kStepFps + kDuration, kDuration); return true;
                        case '.': playing_ = false; time_ = std::fmod(time_ + 1.0 / kStepFps, kDuration); return true;
                        case '-': speed_ = std::max(0.125, speed_ / 2); notify(fmt("speed %gx", speed_)); return true;
                        case '=': case '+': speed_ = std::min(4.0, speed_ * 2); notify(fmt("speed %gx", speed_)); return true;
                        case 'h': case keys::F1: help_ = !help_; return true;
                        case 'p':
                            pathMode_ = !pathMode_;
                            notify(pathMode_ ? "lens path: drag the control points" : "path edit off");
                            return true;
                        case 'r': reset(); return true;
                        case 'b':
                            showBounds_ = !showBounds_;
                            studio_ = Studio(StudioOptions{showBounds_, kDuration, kStepFps}, tracks_);
                            notify(showBounds_ ? "bounding boxes on" : "bounding boxes off");
                            return true;
                        case 's': screenshot(); return true;
                        case keys::Delete:
                            if (pinned_.erase(selected_)) notify(selected_ + " follows the animation again");
                            return true;
                        case '\t': {
                            const auto& els = scene_.elements();
                            std::size_t i = 0;
                            while (i < els.size() && els[i]->name != selected_) ++i;
                            selected_ = els[(i + 1) % els.size()]->name;   // (i == size: starts at 0)
                            return true;
                        }
                        case 'q': rotateBy(selected_, -rad(fine)); return true;
                        case 'e': rotateBy(selected_, rad(fine)); return true;
                        case keys::Left: nudge({e.shift ? -10.0 : -1.0, 0}); return true;
                        case keys::Right: nudge({e.shift ? 10.0 : 1.0, 0}); return true;
                        case keys::Up: nudge({0, e.shift ? -10.0 : -1.0}); return true;
                        case keys::Down: nudge({0, e.shift ? 10.0 : 1.0}); return true;
                        default: return false;
                    }
            }
            return false;
        }
    };
}

int main(int argc, char** argv) {
    try {
        LiveBench bench;
        if (argc == 3 && std::string(argv[1]) == "--self-test") return bench.selfTest(argv[2]) == 0 ? 0 : 1;
        if (argc > 1) {
            std::printf("usage: %s              open the live bench in a window\n"
                        "       %s --self-test DIR   check the interaction without a window (screenshots in DIR)\n",
                        argv[0], argv[0]);
            return argc == 2 && (std::string(argv[1]) == "-h" || std::string(argv[1]) == "--help") ? 0 : 2;
        }
        return bench.run();
    } catch (const std::exception& ex) {
        std::fprintf(stderr, "OptiBench_live: %s\n", ex.what());
        return 1;
    }
}
