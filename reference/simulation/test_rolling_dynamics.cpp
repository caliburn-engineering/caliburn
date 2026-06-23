#include "rolling_dynamics.h"

#include <cassert>
#include <cmath>
#include <cstdio>

using caliburn::RollingBallDynamics;
using caliburn::BallParams;
using caliburn::PlateParams;

static constexpr double TOL = 1e-10;
static constexpr double G = 9.81;

static RollingBallDynamics make_default() {
    BallParams ball{0.02, 0.05, 0.0};      // 20mm radius, 50g, no rolling friction
    PlateParams plate{0.15, G};             // 300mm square plate
    return RollingBallDynamics(ball, plate);
}

// ---------------------------------------------------------------------------
// 1. Level plate: no acceleration
// ---------------------------------------------------------------------------
static void test_level_plate() {
    auto dyn = make_default();
    Eigen::Vector4d state(0.0, 0.0, 0.0, 0.0);

    auto d = dyn.derivatives(state, 0.0, 0.0);

    assert(std::abs(d(2)) < TOL);  // ax = 0
    assert(std::abs(d(3)) < TOL);  // ay = 0
    std::printf("  [PASS] Level plate — zero acceleration\n");
}

// ---------------------------------------------------------------------------
// 2. Tilted plate: correct 5/7 factor
// ---------------------------------------------------------------------------
static void test_rolling_factor() {
    auto dyn = make_default();
    Eigen::Vector4d state(0.0, 0.0, 0.0, 0.0);
    double beta = 0.1;  // 0.1 rad tilt about x

    auto d = dyn.derivatives(state, 0.0, beta);

    double expected_ax = (5.0 / 7.0) * G * std::sin(beta);
    assert(std::abs(d(2) - expected_ax) < TOL);
    assert(std::abs(d(3)) < TOL);  // no y-acceleration
    std::printf("  [PASS] Rolling factor 5/7 correct (ax=%.6f)\n", d(2));
}

// ---------------------------------------------------------------------------
// 3. Both axes tilted
// ---------------------------------------------------------------------------
static void test_both_axes() {
    auto dyn = make_default();
    Eigen::Vector4d state(0.0, 0.0, 0.0, 0.0);
    double alpha = 0.05;
    double beta = 0.1;

    auto d = dyn.derivatives(state, alpha, beta);

    double expected_ax = (5.0 / 7.0) * G * std::sin(beta);
    double expected_ay = (5.0 / 7.0) * G * std::sin(alpha);
    assert(std::abs(d(2) - expected_ax) < TOL);
    assert(std::abs(d(3) - expected_ay) < TOL);
    std::printf("  [PASS] Both axes tilted\n");
}

// ---------------------------------------------------------------------------
// 4. Rolling friction decelerates
// ---------------------------------------------------------------------------
static void test_rolling_friction() {
    BallParams ball{0.02, 0.05, 0.005};  // C_rr = 0.005
    PlateParams plate{0.15, G};
    RollingBallDynamics dyn(ball, plate);

    // Ball rolling on level plate with positive velocity
    Eigen::Vector4d state(0.0, 0.0, 1.0, 0.0);
    auto d = dyn.derivatives(state, 0.0, 0.0);

    // Friction should decelerate (negative ax)
    assert(d(2) < 0.0);
    double expected = -(5.0 / 7.0) * 0.005 * G;
    assert(std::abs(d(2) - expected) < TOL);
    std::printf("  [PASS] Rolling friction decelerates (ax=%.6f)\n", d(2));
}

// ---------------------------------------------------------------------------
// 5. Boundary detection
// ---------------------------------------------------------------------------
static void test_on_plate() {
    auto dyn = make_default();

    Eigen::Vector4d inside(0.1, 0.1, 0.0, 0.0);
    assert(dyn.on_plate(inside));

    Eigen::Vector4d outside(0.2, 0.0, 0.0, 0.0);
    assert(!dyn.on_plate(outside));

    Eigen::Vector4d edge(0.15, 0.0, 0.0, 0.0);
    assert(!dyn.on_plate(edge));  // at boundary = off plate

    std::printf("  [PASS] Boundary detection\n");
}

// ---------------------------------------------------------------------------
// 6. Integration test: ball accelerates under tilt
// ---------------------------------------------------------------------------
static void test_integration() {
    auto dyn = make_default();
    Eigen::Vector4d state(0.0, 0.0, 0.0, 0.0);
    double beta = 0.05;
    double dt = 0.001;

    // Euler integration for 1 second
    for (int i = 0; i < 1000; ++i) {
        auto d = dyn.derivatives(state, 0.0, beta);
        state += d * dt;
    }

    // Ball should have moved in +x direction
    assert(state(0) > 0.0);
    assert(state(2) > 0.0);  // positive velocity

    // Analytical: x = 0.5 * a * t^2, a = 5/7 * g * sin(beta)
    double a = (5.0 / 7.0) * G * std::sin(beta);
    double expected_x = 0.5 * a * 1.0;
    assert(std::abs(state(0) - expected_x) < 0.01);
    std::printf("  [PASS] Integration test (x=%.4f, expected=%.4f)\n", state(0), expected_x);
}

// ---------------------------------------------------------------------------
int main() {
    std::printf("Rolling dynamics tests:\n");

    test_level_plate();
    test_rolling_factor();
    test_both_axes();
    test_rolling_friction();
    test_on_plate();
    test_integration();

    std::printf("All rolling dynamics tests passed.\n");
    return 0;
}
