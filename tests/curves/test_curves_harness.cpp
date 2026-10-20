#include <cmath>

#include "TestHarness.h"

// self-test of the curves test harness: a failed check must be counted, not abort the test

namespace {
    void testPassingChecks(ob::test::Context& ctx) {
        CHECK(1 + 1 == 2);
        CHECK_NEAR(std::sqrt(2.0), 1.41421356237, 1e-10);
        CHECK_VEC_NEAR(ob::Vec2(1, 2), ob::Vec2(1, 2 + 1e-12), 1e-9);
    }

    void testFailuresAreCounted(ob::test::Context& ctx) {
        // the inner context intentionally fails; its messages are expected in the output
        ob::test::Context inner;
        inner.check(false, "intentional failure (expected)");
        inner.near(1.0, 2.0, 1e-3, "intentional mismatch (expected)");
        inner.near(NAN, NAN, 1.0, "NaN never compares near (expected)");
        CHECK(inner.failures() == 3);
        CHECK(inner.checks() == 3);
    }
}

int main() {
    return ob::test::runTests("curves harness", {
        {"passing checks", testPassingChecks},
        {"failures are counted", testFailuresAreCounted},
    });
}
