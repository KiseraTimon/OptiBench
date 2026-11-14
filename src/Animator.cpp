#include "optibench/Animator.h"

#include <cmath>
#include <stdexcept>
#include <utility>

namespace ob {
    OpticalElement& Animator::require(const std::string& name) {
        OpticalElement* e = scene_.find(name);
        if (!e) throw std::invalid_argument("Animator: no element named '" + name + "'");
        return *e;
    }

    void Animator::animateRotation(const std::string& element, Track track) {
        if (track.empty()) throw std::invalid_argument("Animator::animateRotation: track has no keyframes");
        OpticalElement* e = &require(element);     // elements are owned by unique_ptr: the address is stable
        updates_.push_back([e, track = std::move(track)](double t) { e->transform.rotation = track.sample(t); });
    }

    void Animator::animatePosition(const std::string& element, Vec2Track track) {
        if (track.empty()) throw std::invalid_argument("Animator::animatePosition: track has no keyframes");
        OpticalElement* e = &require(element);
        updates_.push_back([e, track = std::move(track)](double t) { e->transform.position = track.sample(t); });
    }

    void Animator::animateScale(const std::string& element, Vec2Track track) {
        if (track.empty()) throw std::invalid_argument("Animator::animateScale: track has no keyframes");
        OpticalElement* e = &require(element);
        updates_.push_back([e, track = std::move(track)](double t) { e->transform.scale = track.sample(t); });
    }

    void Animator::animatePath(const std::string& element, MotionPath path, bool orient, double rotationOffset) {
        OpticalElement* e = &require(element);
        updates_.push_back([e, path = std::move(path), orient, rotationOffset](double t) {
            e->transform.position = path.sample(t);
            if (orient) e->transform.rotation = path.heading(t) + rotationOffset;
        });
    }

    FrameInfo Animator::step(double dt) {
        const double applied = clock_.advance(dt);
        const FrameInfo info{frames_, clock_.time(), applied};

        for (const Update& u : updates_) u(info.time);
        scene_.render(buffers_.back());
        for (const Overlay& o : overlays_) o(buffers_.back(), info);
        buffers_.swap();

        ++frames_;
        return info;
    }

    int Animator::run(double duration, double fps, const Present& present) {
        if (!std::isfinite(fps) || fps <= 0.0) throw std::invalid_argument("Animator::run: fps must be > 0");
        if (!std::isfinite(duration) || duration < 0.0) throw std::invalid_argument("Animator::run: duration must be >= 0");

        const double dt = 1.0 / fps;
        const int steps = static_cast<int>(std::lround(duration * fps));
        int presented = 0;
        for (int i = 0; i <= steps; ++i) {
            const FrameInfo info = step(i == 0 ? 0.0 : dt);     // first frame shows the starting pose
            ++presented;
            if (present && !present(buffers_.front(), info)) break;
        }
        return presented;
    }
}
