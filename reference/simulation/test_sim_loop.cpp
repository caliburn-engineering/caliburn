#include "sim_loop.h"

#include <cassert>
#include <cmath>
#include <cstdio>

static void test_offline_integration() {
    caliburn::SimLoop loop(0.01);

    int counter = 0;
    loop.run_offline(1.0, [&](double /*t*/, double /*dt*/) {
        counter++;
    });

    assert(counter == 100);
    assert(std::abs(loop.sim_time() - 1.0) < 1e-9);

    std::printf("  [PASS] offline integration (counter == 100)\n");
}

static void test_frame_substep_counting() {
    caliburn::SimLoop loop(0.01);

    int physics_calls = 0;
    loop.frame(0.025, [&](double /*t*/, double /*dt*/) {
        physics_calls++;
    }, [](double /*alpha*/) {});

    assert(loop.last_substep_count() == 2);
    assert(physics_calls == 2);

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

    assert(render_calls == 1);
    assert(captured_alpha >= 0.0 && captured_alpha < 1.0);
    // accumulator = 0.025 - 2*0.01 = 0.005, alpha = 0.005/0.01 = 0.5
    assert(std::abs(captured_alpha - 0.5) < 1e-9);

    std::printf("  [PASS] render callback receives valid alpha\n");
}

static void test_sim_time_after_offline() {
    caliburn::SimLoop loop(0.01);

    loop.run_offline(1.0, [](double /*t*/, double /*dt*/) {});

    assert(std::abs(loop.sim_time() - 1.0) < 1e-9);

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
