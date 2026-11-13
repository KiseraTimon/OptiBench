#pragma once
#include "Vec2.h"

namespace ob {
    /**
     * CONTRACT: animation helpers.                                OWNER: Sharon
     * (declarations only: implement in src/Timeline.cpp, signatures can be adjusted via PR)
     *
     * How animation plugs into the scene: each frame, look elements up by name and assign to
     * their `transform`, then call scene.render():
     *
     *      scene.findAs<FlatElement>("mirrorTop")->transform.rotation = rotationTrack.sample(t);
     */

    // linear interpolation: a + (b - a) * t,  t in [0, 1]
    double lerp(double a, double b, double t);
    Vec2 lerp(const Vec2& a, const Vec2& b, double t);

    // easing curves: map t in [0, 1] to a remapped t in [0, 1]
    double easeIn(double t);
    double easeOut(double t);
    double easeInOut(double t);

    // a value at a moment in time
    struct Keyframe {
        double time = 0.0;      // seconds
        double value = 0.0;
    };

    // a sequence of keyframes sampled with tweening
    class Track {
    public:
        void add(double time, double value);
        double sample(double time) const;       // before the first / after the last keyframe: clamp
        double duration() const;
    };
}
