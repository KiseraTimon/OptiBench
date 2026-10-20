#pragma once

/*
 * Special Notes: Ray Paths through Gradient-Index (GRIN) Media
 *
 * In a medium whose refractive index n(r) varies smoothly, light follows the ray equation
 *
 *      d/ds ( n dr/ds ) = grad n          (s = arc length)
 *
 * With the optical direction vector T = n dr/ds and the parameter sigma defined by
 * d sigma = ds / n, this becomes the first-order system
 *
 *      dr/dsigma = T,        dT/dsigma = n grad n = grad(n^2) / 2,
 *
 * which is integrated with classic 4th-order Runge-Kutta. Each step uses
 * d sigma = step / n(r), i.e. roughly `step` units of arc length. Useful properties:
 *
 *   - |T| = n along the exact path (a built-in accuracy check)
 *   - if n depends only on y, T.x = n cos(theta) is conserved (Snell's law in layered media)
 *   - if n^2 is linear in position, the exact path is a parabola and RK4 reproduces it exactly
 *
 * The sampled path converts to a C1 composite Bezier (cubic Hermite in sigma using T as the
 * derivative), so it can be drawn, measured or intersected like any other curve.
 */

#include <functional>
#include <limits>
#include <vector>

#include "optibench/curves/Composite.h"

namespace ob::curves {

    // refractive index at a point
    using IndexField = std::function<double(Vec2)>;

    // gradient of the refractive index at a point
    using IndexGradient = std::function<Vec2(Vec2)>;

    // integration settings
    struct GrinOptions {
        double step = 1.0;                                            // approximate arc length per step (> 0)
        int maxSteps = 10000;                                         // hard limit on steps
        double maxLength = std::numeric_limits<double>::infinity();   // stop after this arc length
        IndexGradient gradient;                                       // optional analytic grad n
        double gradientStep = 1e-4;                                   // central-difference step if gradient is empty
        std::function<bool(Vec2)> stop;                               // optional: stop when this returns true
    };

    // a traced ray path
    struct RayPath {
        std::vector<Vec2> points;       // samples along the ray, starting at the launch point
        std::vector<Vec2> directions;   // unit propagation direction at each sample
        std::vector<Vec2> opticalDirs;  // T = n dr/ds at each sample (|T| = n)
        std::vector<double> sigmas;     // optical parameter sigma at each sample
        double length = 0.0;            // arc length (sum of chord lengths of the samples)

        // the path as a C1 chain of cubic Beziers through every sample
        CompositeBezier toCurve() const;
    };

    /**
     * traces a ray through a GRIN medium
     * @param n refractive index field; tracing stops where it is not finite or <= 0
     * @param start launch point
     * @param direction launch direction (any length; a zero vector returns a single-point path)
     */
    RayPath traceGradientIndex(const IndexField& n, Vec2 start, Vec2 direction, const GrinOptions& options = {});
}
