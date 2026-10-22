#pragma once

/*
 * Special Notes: Curve Editor (data model)
 *
 * Toolkit-independent model for interactive curve editing. A UI layer forwards pointer
 * positions (in the same coordinates as the curve, e.g. framebuffer pixels) and redraws
 * whenever revision() changes; this class owns no window, event loop or renderer.
 *
 *   hit-testing     nearest control point within a pick radius, or nearest point on the curve
 *   editing         move / insert / insertNear / remove control points
 *   dragging        beginDrag -> dragTo* -> endDrag, preserving the grab offset
 *   recomputation   after every successful edit the curve is rebuilt by the CurveBuilder
 *
 * Every edit is transactional: if the builder rejects the new points (e.g. a drag merged
 * points so that too few distinct ones remain), the edit is rolled back and returns false.
 */

#include <cstdint>
#include <functional>
#include <optional>
#include <span>
#include <vector>

#include "optibench/curves/Composite.h"
#include "optibench/curves/CurveMath.h"

namespace ob::curves {

    // turns a list of control points into a curve
    struct CurveBuilder {
        // builds the curve; may throw std::invalid_argument for unusable input
        std::function<CompositeBezier(std::span<const Vec2>)> build;

        // fewest control points the curve type accepts (remove() will not go below it)
        std::size_t minPoints = 2;
    };

    // ready-made builders for every curve type in ob::curves
    namespace builders {
        // one Bezier curve whose degree follows the number of points
        CurveBuilder bezier();

        // interpolating Catmull-Rom spline
        CurveBuilder catmullRom(double alpha = 0.5, double tension = 0.0, bool closed = false);

        // interpolating natural cubic spline
        CurveBuilder naturalCubic(Parametrization parametrization = Parametrization::Chordal);

        // clamped B-spline of the given degree (uses a lower degree while there are too few points)
        CurveBuilder clampedBSpline(int degree = 3);
    }

    // result of hit-testing the curve itself
    struct CurvePick {
        double u = 0.0;         // global curve parameter of the nearest point
        Vec2 point;             // nearest point on the curve
        double distance = 0.0;  // distance from the query position
    };

    class CurveEditor {
    public:
        /**
         * @param points initial control points
         * @param builder how points become a curve
         * @throws std::invalid_argument if the builder rejects the initial points
         */
        CurveEditor(std::vector<Vec2> points, CurveBuilder builder);

        std::span<const Vec2> points() const { return points_; }

        // the current curve, always consistent with points()
        const CompositeBezier& curve() const { return curve_; }

        // increases by one after every successful change (use it to invalidate render caches)
        std::uint64_t revision() const { return revision_; }

        /*
         * Hit-testing
         */
        // index of the control point nearest to p within radius (ties: lowest index)
        std::optional<std::size_t> hitTest(Vec2 p, double radius) const;

        // nearest point on the curve, if it is within radius
        std::optional<CurvePick> hitTestCurve(Vec2 p, double radius) const;

        /*
         * Editing (each returns false and changes nothing if the edit is invalid)
         */
        // moves control point i to p
        bool move(std::size_t i, Vec2 p);

        // inserts p before index (index == size appends)
        bool insert(std::size_t index, Vec2 p);

        // inserts p into the nearest edge of the control polygon; returns its index
        std::optional<std::size_t> insertNear(Vec2 p);

        // removes control point i (refused below the builder's minimum)
        bool remove(std::size_t i);

        // switches the curve type, keeping the points
        bool setBuilder(CurveBuilder builder);

        /*
         * Dragging
         */
        // grabs the control point under p (within radius); false if there is none
        bool beginDrag(Vec2 p, double radius);

        // moves the grabbed point so it keeps its offset from the pointer
        bool dragTo(Vec2 p);

        // releases the grabbed point
        void endDrag() { dragged_.reset(); }

        // index of the grabbed point, if any
        std::optional<std::size_t> dragged() const { return dragged_; }

    private:
        std::vector<Vec2> points_;
        CurveBuilder builder_;
        CompositeBezier curve_;
        std::uint64_t revision_ = 0;
        std::optional<std::size_t> dragged_;
        Vec2 grabOffset_;

        // rebuilds from candidate points; commits them only if the builder accepts
        bool commit(std::vector<Vec2> candidate);
    };
}
