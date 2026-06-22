#include "fixed_timestep.h"

#include <cassert>
#include <cmath>
#include <cstdio>

static void test_exact_substep_count() {
    caliburn::FixedTimestep ts(0.01);
    int substeps = ts.accumulate(0.035, []() {});

    assert(substeps == 3);
    assert(std::abs(ts.sim_time() - 0.03) < 1e-9);
    // accumulator should be ~0.005
    assert(std::abs(ts.alpha() - 0.5) < 1e-9);

    std::printf("  [PASS] exact substep count\n");
}

static void test_frame_time_clamping() {
    caliburn::FixedTimestep ts(0.01, 0.25);
    int substeps = ts.accumulate(1.0, []() {});

    // max_frame_dt=0.25 → accumulator clamped to 0.25
    // Due to IEEE 754 drift after repeated subtraction, we get 24 substeps
    // (residual ~0.00999... < dt). The key invariant: far fewer than 100 substeps.
    assert(substeps <= 25 && substeps >= 24);
    assert(substeps < 100);  // spiral of death prevented
    assert(ts.sim_time() < 0.26);

    std::printf("  [PASS] frame time clamping (spiral of death prevention)\n");
}

static void test_sim_time_accuracy() {
    caliburn::FixedTimestep ts(0.01);

    for (int i = 0; i < 100; i++) {
        ts.accumulate(0.016, []() {});
    }

    // Total input time: 100 * 0.016 = 1.6s
    // Each frame: 0.016 / 0.01 = 1 substep with 0.006 remainder
    // After 2 frames: accumulator = 0.012 → 1 substep, remainder 0.002
    // Pattern repeats. Total substeps should consume close to 1.6s minus residual accumulator.
    // sim_time + alpha*dt should reconstruct total elapsed time
    double reconstructed = ts.sim_time() + ts.alpha() * ts.dt();
    assert(std::abs(reconstructed - 1.6) < 0.001);

    std::printf("  [PASS] sim time accuracy over 100 frames\n");
}

static void test_alpha_interpolation() {
    caliburn::FixedTimestep ts(0.01);
    ts.accumulate(0.035, []() {});

    double a = ts.alpha();
    assert(a >= 0.0 && a < 1.0);
    // accumulator = 0.005, dt = 0.01 → alpha = 0.5
    assert(std::abs(a - 0.5) < 1e-9);

    std::printf("  [PASS] alpha interpolation\n");
}

static void test_zero_frame_dt() {
    caliburn::FixedTimestep ts(0.01);
    int substeps = ts.accumulate(0.0, []() {
        assert(false && "step_fn should not be called with zero frame_dt");
    });

    assert(substeps == 0);
    assert(ts.sim_time() == 0.0);

    std::printf("  [PASS] zero frame_dt\n");
}

int main() {
    std::printf("fixed_timestep tests:\n");

    test_exact_substep_count();
    test_frame_time_clamping();
    test_sim_time_accuracy();
    test_alpha_interpolation();
    test_zero_frame_dt();

    std::printf("All 5 tests passed.\n");
    return 0;
}
