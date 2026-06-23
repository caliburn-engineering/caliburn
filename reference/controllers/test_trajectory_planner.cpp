#include "trajectory_planner.h"

#include <cassert>
#include <cmath>
#include <cstdio>

using namespace caliburn;

static constexpr double TOL = 1e-10;

// ---------------------------------------------------------------------------
// 1. Cubic: boundary conditions satisfied
// ---------------------------------------------------------------------------
static void test_cubic_boundary_conditions() {
    CubicTrajectory traj(1.0, 0.5, 3.0, -0.5, 2.0);

    assert(std::abs(traj.position(0.0) - 1.0) < TOL);
    assert(std::abs(traj.velocity(0.0) - 0.5) < TOL);
    assert(std::abs(traj.position(2.0) - 3.0) < TOL);
    assert(std::abs(traj.velocity(2.0) - (-0.5)) < TOL);
    std::printf("  [PASS] Cubic boundary conditions\n");
}

// ---------------------------------------------------------------------------
// 2. Cubic: rest-to-rest has zero velocity at endpoints
// ---------------------------------------------------------------------------
static void test_cubic_rest_to_rest() {
    CubicTrajectory traj(0.0, 0.0, 1.0, 0.0, 1.0);

    assert(std::abs(traj.velocity(0.0)) < TOL);
    assert(std::abs(traj.velocity(1.0)) < TOL);
    assert(std::abs(traj.position(1.0) - 1.0) < TOL);
    std::printf("  [PASS] Cubic rest-to-rest\n");
}

// ---------------------------------------------------------------------------
// 3. MinJerk: boundary conditions (rest-to-rest)
// ---------------------------------------------------------------------------
static void test_minjerk_boundary_conditions() {
    MinJerkTrajectory traj(0.0, 2.0, 1.0);

    assert(std::abs(traj.position(0.0) - 0.0) < TOL);
    assert(std::abs(traj.position(1.0) - 2.0) < TOL);
    assert(std::abs(traj.velocity(0.0)) < TOL);
    assert(std::abs(traj.velocity(1.0)) < TOL);
    assert(std::abs(traj.acceleration(0.0)) < TOL);
    assert(std::abs(traj.acceleration(1.0)) < TOL);
    std::printf("  [PASS] MinJerk boundary conditions\n");
}

// ---------------------------------------------------------------------------
// 4. MinJerk: peak velocity at midpoint
// ---------------------------------------------------------------------------
static void test_minjerk_peak_velocity() {
    MinJerkTrajectory traj(0.0, 1.0, 1.0);

    // Velocity at t=0.5 should be the maximum (1.875 for unit displacement/time)
    double v_mid = traj.velocity(0.5);
    double v_quarter = traj.velocity(0.25);
    double v_three_quarter = traj.velocity(0.75);

    assert(v_mid > v_quarter);
    assert(v_mid > v_three_quarter);
    assert(std::abs(v_quarter - v_three_quarter) < TOL);  // symmetric
    std::printf("  [PASS] MinJerk peak velocity at midpoint (v=%.4f)\n", v_mid);
}

// ---------------------------------------------------------------------------
// 5. Trapezoidal: reaches target position
// ---------------------------------------------------------------------------
static void test_trapezoidal_reaches_target() {
    TrapezoidalTrajectory traj(0.0, 10.0, 2.0, 1.0);

    double T = traj.duration();
    assert(std::abs(traj.position(T) - 10.0) < 1e-8);
    assert(std::abs(traj.velocity(0.0)) < TOL);
    assert(std::abs(traj.velocity(T)) < TOL);
    std::printf("  [PASS] Trapezoidal reaches target (T=%.4f)\n", T);
}

// ---------------------------------------------------------------------------
// 6. Trapezoidal: triangular profile (short distance)
// ---------------------------------------------------------------------------
static void test_trapezoidal_triangular() {
    // v_max=10, a_max=1 => needs 100 units to reach cruise.  0.5 << 100 => triangular.
    TrapezoidalTrajectory traj(0.0, 0.5, 10.0, 1.0);

    double T = traj.duration();
    assert(std::abs(traj.position(T) - 0.5) < 1e-8);
    // Peak velocity should be less than v_max
    double v_peak = traj.velocity(T / 2.0);
    assert(v_peak < 10.0);
    assert(v_peak > 0.0);
    std::printf("  [PASS] Trapezoidal triangular profile (v_peak=%.4f)\n", v_peak);
}

// ---------------------------------------------------------------------------
// 7. Trapezoidal: reverse direction
// ---------------------------------------------------------------------------
static void test_trapezoidal_reverse() {
    TrapezoidalTrajectory traj(5.0, 0.0, 2.0, 1.0);

    double T = traj.duration();
    assert(std::abs(traj.position(T) - 0.0) < 1e-8);
    // Velocity should be negative
    double v_mid = traj.velocity(T / 2.0);
    assert(v_mid < 0.0);
    std::printf("  [PASS] Trapezoidal reverse direction\n");
}

// ---------------------------------------------------------------------------
int main() {
    std::printf("Trajectory planner tests:\n");

    test_cubic_boundary_conditions();
    test_cubic_rest_to_rest();
    test_minjerk_boundary_conditions();
    test_minjerk_peak_velocity();
    test_trapezoidal_reaches_target();
    test_trapezoidal_triangular();
    test_trapezoidal_reverse();

    std::printf("All trajectory planner tests passed.\n");
    return 0;
}
