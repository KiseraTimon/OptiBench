#include "optibench/curves/RayPath.h"

#include "optibench/curves/Hermite.h"

namespace ob::curves {
    namespace {
        // derivative of the state (r, T) with respect to sigma
        struct StateRate {
            Vec2 dr;
            Vec2 dT;
        };

        bool usableIndex(double v) { return std::isfinite(v) && v > 0.0; }
    }

    CompositeBezier RayPath::toCurve() const {
        CompositeBezier out;
        for (std::size_t i = 0; i + 1 < points.size(); ++i) {
            const double dSigma = sigmas[i + 1] - sigmas[i];
            out.append(HermiteSegment{points[i], opticalDirs[i] * dSigma, points[i + 1], opticalDirs[i + 1] * dSigma}.toBezier());
        }
        return out;
    }

    RayPath traceGradientIndex(const IndexField& n, Vec2 start, Vec2 direction, const GrinOptions& options) {
        RayPath path;
        const Vec2 d0 = direction.normalized();
        const double n0 = n ? n(start) : 0.0;
        path.points.push_back(start);
        path.directions.push_back(d0);
        path.opticalDirs.push_back(d0 * (usableIndex(n0) ? n0 : 0.0));
        path.sigmas.push_back(0.0);
        if (d0.length() == 0.0 || !usableIndex(n0) || !(options.step > 0.0)) return path;

        // grad(n^2)/2 = n grad n, from the analytic gradient or central differences
        auto halfGradSq = [&](Vec2 p, double np) -> Vec2 {
            if (options.gradient) return options.gradient(p) * np;
            const double h = options.gradientStep;
            const Vec2 g{(n({p.x + h, p.y}) - n({p.x - h, p.y})) / (2 * h), (n({p.x, p.y + h}) - n({p.x, p.y - h})) / (2 * h)};
            return g * np;
        };
        bool valid = true;
        auto rate = [&](Vec2 r, Vec2 T) -> StateRate {
            const double nr = n(r);
            if (!usableIndex(nr)) { valid = false; return {Vec2{}, Vec2{}}; }
            return {T, halfGradSq(r, nr)};
        };

        Vec2 r = start;
        Vec2 T = d0 * n0;
        double sigma = 0.0;
        for (int step = 0; step < options.maxSteps && path.length < options.maxLength; ++step) {
            const double nr = n(r);
            if (!usableIndex(nr)) break;
            const double h = options.step / nr;

            valid = true;
            const StateRate k1 = rate(r, T);
            const StateRate k2 = rate(r + k1.dr * (h / 2), T + k1.dT * (h / 2));
            const StateRate k3 = rate(r + k2.dr * (h / 2), T + k2.dT * (h / 2));
            const StateRate k4 = rate(r + k3.dr * h, T + k3.dT * h);
            if (!valid) break;
            const Vec2 rNext = r + (k1.dr + k2.dr * 2.0 + k3.dr * 2.0 + k4.dr) * (h / 6);
            const Vec2 TNext = T + (k1.dT + k2.dT * 2.0 + k3.dT * 2.0 + k4.dT) * (h / 6);
            if (!std::isfinite(rNext.x) || !std::isfinite(rNext.y) || TNext.length() == 0.0) break;

            path.length += (rNext - r).length();
            r = rNext;
            T = TNext;
            sigma += h;
            path.points.push_back(r);
            path.directions.push_back(T.normalized());
            path.opticalDirs.push_back(T);
            path.sigmas.push_back(sigma);
            if (options.stop && options.stop(r)) break;
        }
        return path;
    }
}
