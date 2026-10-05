#include "sim_loop.h"

#include "assert_rel.h"
#include <cmath>
#include <cstdio>

static void test_offline_integration() {
    caliburn::SimLoop loop(0.01);

    int counter = 0;
    loop.run_offline(1.0, [&](double /*t*/, double /*dt*/) {
        counter++;
    });

    ASSERT_CHECK(counter == 100, "100 substeps for 1.0s at 0.01s step");
    // 1e-9: 100 additions of exact 0.01s; IEEE 754 drift ≤ 100×2⁻⁵³ ≪ 1e-9
    ASSERT_REL_NEAR(loop.sim_time(), 1.0, 1e-9);

    std::printf("  [PASS] offline integration (counter == 100)\n");
}

static void test_frame_substep_counting() {
    caliburn::SimLoop loop(0.01);

    int physics_calls = 0;
    loop.frame(0.025, [&](double /*t*/, double /*dt*/) {
        physics_calls++;
    }, [](double /*alpha*/) {});

    ASSERT_CHECK(loop.last_substep_count() == 2, "2 substeps for 0.025s frame at 0.01s step");
    ASSERT_CHECK(physics_calls == 2, "physics callback called twice for 2 substeps");

    std::printf("  [PASS] frame substep counting\n");
}

static void test_render_callback_alpha() {
    caliburn::SimLoop loop(0.01);

    int render_calls = 0;
    double captured_alpha = -1.0;

    loop.frame(0.025, [](double /*t*/, double /*dt*/) {}, [&](double alpha) {
        render_calls++;
        captured_alpha = alpha;
    });

    ASSERT_CHECK(render_calls == 1, "render callback called once per frame");
    ASSERT_CHECK(captured_alpha >= 0.0 && captured_alpha < 1.0, "alpha must lie in [0, 1)");
    // accumulator = 0.025 - 2*0.01 = 0.005, alpha = 0.005/0.01 = 0.5
    // 1e-9: alpha = 0.005/0.01; single division; guards IEEE 754 rounding
    ASSERT_REL_NEAR(captured_alpha, 0.5, 1e-9);

    std::printf("  [PASS] render callback receives valid alpha\n");
}

static void test_sim_time_after_offline() {
    caliburn::SimLoop loop(0.01);

    loop.run_offline(1.0, [](double /*t*/, double /*dt*/) {});

    // 1e-9: 100 additions of exact 0.01s; IEEE 754 drift ≤ 100×2⁻⁵³ ≪ 1e-9
    ASSERT_REL_NEAR(loop.sim_time(), 1.0, 1e-9);

    std::printf("  [PASS] sim_time progresses correctly after run_offline\n");
}

int main() {
    std::printf("sim_loop tests:\n");

    test_offline_integration();
    test_frame_substep_counting();
    test_render_callback_alpha();
    test_sim_time_after_offline();

    std::printf("All 4 tests passed.\n");
    return 0;
}
