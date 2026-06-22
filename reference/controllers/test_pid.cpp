#include "pid.h"

#include <cassert>
#include <cmath>
#include <cstdio>

using caliburn::PidController;
using caliburn::PidGains;

// ---------------------------------------------------------------------------
// 1. P-only step response
//    Plant: integrator  dx/dt = u
//    Setpoint = 1, Kp = 2, run 1000 steps at dt = 0.01
//    Final error should be < 0.01
// ---------------------------------------------------------------------------
static void test_p_only_step_response() {
    PidGains gains{2.0, 0.0, 0.0};
    PidController pid(gains, -100.0, 100.0);

    double x = 0.0;
    const double dt = 0.01;
    const double setpoint = 1.0;

    for (int i = 0; i < 1000; ++i) {
        double u = pid.compute(setpoint, x, dt);
        x += u * dt;  // integrator plant
    }

    double error = std::fabs(setpoint - x);
    assert(error < 0.01);
    std::printf("  [PASS] P-only step response (error=%.6f)\n", error);
}

// ---------------------------------------------------------------------------
// 2. PI eliminates steady-state error
//    Plant with drag: dx/dt = u - 0.5*x
//    Kp = 1, Ki = 2, setpoint = 1, 5000 steps
//    Final error should be < 0.01
// ---------------------------------------------------------------------------
static void test_pi_steady_state() {
    PidGains gains{1.0, 2.0, 0.0};
    PidController pid(gains, -100.0, 100.0);

    double x = 0.0;
    const double dt = 0.01;
    const double setpoint = 1.0;

    for (int i = 0; i < 5000; ++i) {
        double u = pid.compute(setpoint, x, dt);
        x += (u - 0.5 * x) * dt;  // plant with drag
    }

    double error = std::fabs(setpoint - x);
    assert(error < 0.01);
    std::printf("  [PASS] PI eliminates steady-state error (error=%.6f)\n", error);
}

// ---------------------------------------------------------------------------
// 3. Output clamping
//    Kp = 100, output range [-1, 1], large error => output clamped to +/-1
// ---------------------------------------------------------------------------
static void test_output_clamping() {
    PidGains gains{100.0, 0.0, 0.0};
    PidController pid(gains, -1.0, 1.0);

    // Large positive error
    double u = pid.compute(10.0, 0.0, 0.01);
    assert(u == 1.0);

    // Large negative error
    pid.reset();
    u = pid.compute(-10.0, 0.0, 0.01);
    assert(u == -1.0);

    std::printf("  [PASS] Output clamping\n");
}

// ---------------------------------------------------------------------------
// 4. Anti-windup
//    Kp = 1, Ki = 10, range [-1, 1].  Saturate for 1000 steps with
//    error = 100, then reverse to error = -2.  With anti-windup the integral
//    is clamped to 0.1 (= output_max / Ki), so after reversal the P-term
//    (-2.0) dominates the residual I-term and output goes negative within
//    a few steps.  Without clamping the integral would be ~10000 and the
//    controller would stay pegged at +1 for hundreds of steps.
// ---------------------------------------------------------------------------
static void test_anti_windup() {
    PidGains gains{1.0, 10.0, 0.0};
    PidController pid(gains, -1.0, 1.0);

    const double dt = 0.01;

    // Saturate: drive with huge error for 1000 steps
    for (int i = 0; i < 1000; ++i) {
        pid.compute(100.0, 0.0, dt);
    }

    // Integral should be clamped, not at the unclamped value of ~10000
    assert(pid.integral() < 1.0);

    // Reverse: setpoint=0, measurement=2 => error = -2
    // P-term = -2.0, I-term = Ki * ~0.08 = ~0.8  =>  output < 0
    double u = pid.compute(0.0, 2.0, dt);

    assert(u < 0.0);
    std::printf("  [PASS] Anti-windup (output after reversal=%.4f)\n", u);
}

// ---------------------------------------------------------------------------
// 5. Reset zeroes integral
// ---------------------------------------------------------------------------
static void test_reset() {
    PidGains gains{1.0, 5.0, 0.0};
    PidController pid(gains, -10.0, 10.0);

    pid.compute(1.0, 0.0, 0.01);
    pid.compute(1.0, 0.0, 0.01);
    assert(pid.integral() != 0.0);

    pid.reset();
    assert(pid.integral() == 0.0);

    std::printf("  [PASS] Reset zeroes integral\n");
}

// ---------------------------------------------------------------------------
int main() {
    std::printf("PID controller tests:\n");

    test_p_only_step_response();
    test_pi_steady_state();
    test_output_clamping();
    test_anti_windup();
    test_reset();

    std::printf("All PID tests passed.\n");
    return 0;
}
