#pragma once

/*
 * Special Notes: Curves Test Harness
 *
 * A tiny, framework-free test harness used by the curves tests.
 *
 * Unlike the single-shot CHECK macro used by the other module tests, a failed check here is
 * recorded and the test keeps running, so one run reports every broken expectation.
 *
 * Usage:
 *      void testSomething(ob::test::Context& ctx) {
 *          CHECK(1 + 1 == 2);
 *          CHECK_NEAR(std::sqrt(2.0), 1.41421356, 1e-8);
 *      }
 *
 *      int main() { return ob::test::runTests("suite", {{"something", testSomething}}); }
 *
 * The macros expect a variable named `ctx` (the Context) in scope.
 */

#include <cmath>
#include <cstdio>
#include <initializer_list>
#include <source_location>
#include <string_view>

#include "optibench/Vec2.h"

namespace ob::test {

    // collects the results of the checks made by one test case
    class Context {
    public:
        /**
         * records a boolean expectation
         * @param ok bool: the result of the check
         * @param expr std::string_view: the source text of the check (for the failure message)
         * @param loc std::source_location: filled in automatically with the caller's location
         * @return bool: ok, so callers can bail out early if later checks depend on it
         */
        bool check(bool ok, std::string_view expr,
                   std::source_location loc = std::source_location::current()) {
            ++checks_;
            if (!ok) {
                ++failures_;
                std::printf("    FAIL %s:%u  %.*s\n", loc.file_name(), static_cast<unsigned>(loc.line()),
                            static_cast<int>(expr.size()), expr.data());
            }
            return ok;
        }

        // records that two numbers agree within an absolute tolerance
        bool near(double a, double b, double tol, std::string_view expr,
                  std::source_location loc = std::source_location::current()) {
            const bool ok = std::isfinite(a) && std::isfinite(b) && std::fabs(a - b) <= tol;
            if (!ok) std::printf("    values: %.12g vs %.12g (tol %.3g)\n", a, b, tol);
            return check(ok, expr, loc);
        }

        // records that two points agree within an absolute tolerance (euclidean distance)
        bool nearVec(Vec2 a, Vec2 b, double tol, std::string_view expr,
                     std::source_location loc = std::source_location::current()) {
            const double d = (a - b).length();
            const bool ok = std::isfinite(d) && d <= tol;
            if (!ok) std::printf("    values: (%.12g, %.12g) vs (%.12g, %.12g) (tol %.3g)\n", a.x, a.y, b.x, b.y, tol);
            return check(ok, expr, loc);
        }

        // number of checks that failed
        int failures() const { return failures_; }

        // number of checks made
        int checks() const { return checks_; }

    private:
        int checks_ = 0;
        int failures_ = 0;
    };

    // a named test function
    struct TestCase {
        std::string_view name;
        void (*fn)(Context&);
    };

    /**
     * runs every test case and prints a summary
     * @param suite std::string_view: name printed in the header
     * @param tests the test cases to run, in order
     * @return int: 0 if every check passed, 1 otherwise (use as the process exit code)
     */
    inline int runTests(std::string_view suite, std::initializer_list<TestCase> tests) {
        std::printf("== %.*s\n", static_cast<int>(suite.size()), suite.data());
        int failedCases = 0;
        int totalChecks = 0;
        for (const TestCase& tc : tests) {
            Context ctx;
            tc.fn(ctx);
            totalChecks += ctx.checks();
            const bool ok = ctx.failures() == 0;
            if (!ok) ++failedCases;
            std::printf("  [%s] %.*s (%d checks)\n", ok ? " OK " : "FAIL", static_cast<int>(tc.name.size()),
                        tc.name.data(), ctx.checks());
        }
        std::printf("== %zu cases, %d checks, %d failed cases\n", tests.size(), totalChecks, failedCases);
        return failedCases == 0 ? 0 : 1;
    }
}

// records a boolean expectation; needs `ctx` in scope
#define CHECK(cond) ctx.check(static_cast<bool>(cond), #cond)

// records that two numbers agree within tol; needs `ctx` in scope
#define CHECK_NEAR(a, b, tol) ctx.near((a), (b), (tol), #a " ~= " #b)

// records that two Vec2 points agree within tol; needs `ctx` in scope
#define CHECK_VEC_NEAR(a, b, tol) ctx.nearVec((a), (b), (tol), #a " ~= " #b)
