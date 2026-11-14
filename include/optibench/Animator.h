#pragma once
#include <functional>
#include <string>
#include <vector>

#include "DoubleBuffer.h"
#include "Scene.h"
#include "Timeline.h"

namespace ob {
    // what one iteration of the animation loop produced
    struct FrameInfo {
        int index = 0;          // 0 for the first frame
        double time = 0.0;      // animation time of this frame, in seconds
        double dt = 0.0;        // time step that led to it
    };

    /**
     * The animation loop. Each step():
     *
     *      1. clock.advance(dt)                  delta time
     *      2. run every binding / update         tracks and paths sampled at clock.time(),
     *                                            written into the elements' transforms
     *      3. scene.render(buffers.back())       draw off-screen (beams re-traced from scratch)
     *      4. run every overlay on the back      optional extra drawing (labels, guides, graphs)
     *      5. buffers.swap()                     double buffering: the frame is now front()
     *
     * run() repeats this at a fixed 1/fps for offline rendering and hands each finished front
     * buffer to a callback (e.g. to save it as a .bmp). A live loop would call step(timer.lap())
     * instead and blit front() to a window.
     *
     *      Animator anim(scene, buffers);
     *      anim.animateRotation("mirrorTop", Track{}.add(0, a0).add(2, a1, easeInOut));
     *      anim.run(4.0, 15.0, [](const Framebuffer& f, const FrameInfo& i) { return f.saveBMP(...); });
     */
    class Animator {
    public:
        using Update = std::function<void(double time)>;
        using Overlay = std::function<void(Framebuffer& back, const FrameInfo& frame)>;
        using Present = std::function<bool(const Framebuffer& front, const FrameInfo& frame)>;   // false stops run()

        Animator(Scene& scene, DoubleBuffer& buffers) : scene_(scene), buffers_(buffers) {}

        /*
         * Bindings: drive one element's transform from a track or a path.
         * @throws std::invalid_argument if no element has that name, or the track is empty
         */
        void animateRotation(const std::string& element, Track track);
        void animatePosition(const std::string& element, Vec2Track track);
        void animateScale(const std::string& element, Vec2Track track);

        // moves the element along a curve; with orient, its rotation follows the direction of travel
        // (plus rotationOffset radians)
        void animatePath(const std::string& element, MotionPath path, bool orient = false,
                         double rotationOffset = 0.0);

        // any other per-frame change, run in the order added (after the bindings added before it)
        void addUpdate(Update update) { updates_.push_back(std::move(update)); }

        // extra drawing on the back buffer after the scene is rendered, before the swap
        void addOverlay(Overlay overlay) { overlays_.push_back(std::move(overlay)); }

        // one loop iteration (see above); returns what was drawn, which is now buffers.front()
        FrameInfo step(double dt);

        /**
         * offline loop: frames at t = 0, 1/fps, 2/fps, ... up to `duration` seconds from the clock's current time,
         * each passed to `present` (if given) once it is the front buffer.
         * @return the number of frames presented
         * @throws std::invalid_argument if fps <= 0 or duration < 0 (or either is not finite)
         */
        int run(double duration, double fps, const Present& present = {});

        Clock& clock() { return clock_; }
        const Clock& clock() const { return clock_; }
        int framesDrawn() const { return frames_; }

    private:
        Scene& scene_;
        DoubleBuffer& buffers_;
        Clock clock_;
        std::vector<Update> updates_;
        std::vector<Overlay> overlays_;
        int frames_ = 0;

        OpticalElement& require(const std::string& name);
    };
}
