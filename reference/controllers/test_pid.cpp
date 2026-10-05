#include "pid.h"
#include "assert_rel.h"

#include <cmath>
#include <cstdio>
#include <stdexcept>

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
    // TODO(#75): tolerance unjustified — error decays as (1 - Kp*dt)^n = 0.98^1000 ~ 1.7e-9,
    // so 0.01 is ~1e7x looser than the derivable error.
    ASSERT_CHECK(error < 0.01, "P-only convergence: error should be < 0.01 after 1000 steps");
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
    // TODO(#75): tolerance unjustified — measured error prints as 0.000000 in Release;
    // 0.01 is a loose convergence bound, not derived from the closed-loop decay rate.
    ASSERT_CHECK(error < 0.01, "PI steady-state error should be < 0.01 after 5000 steps");
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
    ASSERT_REL_NEAR(u, 1.0, 0.0);   // exact: clamp assigns u_max directly

    // Large negative error
    pid.reset();
    u = pid.compute(-10.0, 0.0, 0.01);
    ASSERT_REL_NEAR(u, -1.0, 0.0);  // exact: clamp assigns u_min directly

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
    // 1.0: the clamp bound is u_max/Ki = 0.1; unclamped the integral would reach ~1000.
    // Any threshold between the two separates clamped from unclamped.
    ASSERT_CHECK(pid.integral() < 1.0, "anti-windup: integral must be clamped near u_max/Ki = 0.1, not wound up to ~1000");

    // Reverse: setpoint=0, measurement=2 => error = -2
    // P-term = -2.0, I-term = Ki * ~0.08 = ~0.8  =>  output < 0
    double u = pid.compute(0.0, 2.0, dt);

    ASSERT_CHECK(u < 0.0, "anti-windup: P-term dominates clamped I-term after reversal, output must be negative");
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
    ASSERT_CHECK(pid.integral() != 0.0, "integral must accumulate after two valid compute() calls");

    pid.reset();
    ASSERT_REL_NEAR(pid.integral(), 0.0, 0.0);  // exact: reset() assigns 0.0 directly

    std::printf("  [PASS] Reset zeroes integral\n");
}

// ---------------------------------------------------------------------------
// Check 1: Constant-error sum
//   setpoint=1, measurement=0.5 => e=0.5 held fixed every call.
//   Kp=2, Ki=4, dt=0.25 (all dyadic rationals); limits ±1000 (no clamping).
//   D term is 0: first call explicitly; subsequent calls because
//     raw_derivative = -(m_k − m_{k-1})/dt = 0 (measurement is constant).
//   integral after call k = k·e·dt = k·0.125 (exact, no clamp).
//   expected_k = Kp·e + Ki·k·e·dt = 1.0 + 0.5·k.
//   All operands are dyadic rationals → every step is exact. Tolerance=0.0.
// ---------------------------------------------------------------------------
static void check_constant_error_sum() {
    const double setpoint = 1.0;
    const double meas = 0.5;
    const double e = setpoint - meas;  // 0.5
    const double Kp = 2.0, Ki = 4.0, dt = 0.25;
    const int N = 10;

    PidGains gains{Kp, Ki, 0.0};
    PidController pid(gains, -1000.0, 1000.0);

    for (int k = 1; k <= N; ++k) {
        double output = pid.compute(setpoint, meas, dt);
        // integral_k = k·e·dt; output_k = Kp·e + Ki·k·e·dt.
        // Exact: dyadic rational operands, no rounding. Tolerance=0.0.
        double expected = Kp * e + Ki * (double)k * e * dt;
        ASSERT_REL_NEAR(output, expected, 0.0);
    }
    std::printf("  [PASS] Check 1: constant-error sum\n");
}

// ---------------------------------------------------------------------------
// Check 2: Anti-windup
//   Kp=0, Ki=1, limits [−5,5], dt=0.25.
//   Saturation phase: e=100 for 200 steps.
//   After each step integral_ clamps to u_max/Ki = 5.0.
//   (a) every output is in [u_min, u_max].
//   (b) integral() never exceeds u_max/Ki = 5.0.
//   (d) the output clamp holds on its own when P alone saturates (see below).
//   Flip: setpoint=0, measurement=1 => e=−1.
//   (c) derivation: integral_clamped=5.0; step 1: integral=5.0−0.25=4.75,
//       output=4.75 < u_max. So output leaves u_max after exactly 1 step.
//       Without anti-windup: integral_unclamped = 100·0.25·200 = 5000;
//       would need ≈19980 steps to unwind (check (c) would fail).
// ---------------------------------------------------------------------------
static void check_anti_windup() {
    const double Kp = 0.0, Ki = 1.0;
    const double u_min = -5.0, u_max = 5.0, dt = 0.25;

    PidGains gains{Kp, Ki, 0.0};
    PidController pid(gains, u_min, u_max);

    // Saturation phase: checks (a) and (b).
    for (int i = 0; i < 200; ++i) {
        double out = pid.compute(100.0, 0.0, dt);
        ASSERT_CHECK(out >= u_min, "output below u_min during saturation");
        ASSERT_CHECK(out <= u_max, "output above u_max during saturation");
        ASSERT_CHECK(pid.integral() <= u_max / Ki,
                     "integral exceeds u_max/Ki during saturation");
    }

    // Check (c): flip to e=−1; output must leave u_max in 1 step, landing on
    // the hand-derived 5.0 − 0.25 = 4.75. Exact: dyadic operands. Tolerance=0.0.
    double out = pid.compute(0.0, 1.0, dt);
    ASSERT_CHECK(out < u_max,
                 "output did not leave u_max within 1 step (anti-windup)");
    ASSERT_REL_NEAR(out, 4.75, 0.0);

    // (d) Output clamp, independent of the integrator: with Ki=0 the integral
    //     clamp is skipped, and Kp·e = 10·(±1) = ±10 lies outside [−5, 5]. The
    //     output must be exactly u_max, then u_min. With Kp=0 above, the clamped
    //     integral alone keeps the output in range, so (a) can't catch a missing
    //     output clamp; this can.
    PidController p_only(PidGains{10.0, 0.0, 0.0}, u_min, u_max);
    ASSERT_REL_NEAR(p_only.compute(1.0, 0.0, dt), u_max, 0.0);
    ASSERT_REL_NEAR(p_only.compute(-1.0, 0.0, dt), u_min, 0.0);

    std::printf("  [PASS] Check 2: anti-windup\n");
}

// ---------------------------------------------------------------------------
// Check 3: Derivative filter
//   Kp=Ki=0, Kd=1, setpoint=0, measurement ramp m_j = r·j·dt, r=2, dt=0.25.
//   alpha = kDerivativeAlpha = 0.1.
//   Call j=1 (first): D=0 exactly (first_call_ guard).
//   Calls j=2..21 (k=j−1=1..20): raw = −(m_j − m_{j-1})/dt = −r exactly.
//   Closed form: d_k = −r·(1 − (1−alpha)^k).
//   Derivation by induction: d_0=0;
//     d_k = alpha·(−r) + (1−alpha)·d_{k-1}
//         = −r·alpha·(1 + (1−alpha) + … + (1−alpha)^{k-1})
//         = −r·alpha·(1 − (1−alpha)^k)/alpha = −r·(1 − (1−alpha)^k).
//   Tolerance 1e-12: alpha=0.1 and (1−alpha)=0.9 are not exact in IEEE 754;
//   per-step rounding ≤ 2^−52·max(|d|) ≈ 4.4e-16; 20 steps accumulate
//   ≤ 20·4.4e-16·2 ≈ 1.8e-14; 1e-12 adds a 55× margin.
// ---------------------------------------------------------------------------
static void check_derivative_filter() {
    const double r = 2.0, dt = 0.25;
    const double alpha = 0.1;  // kDerivativeAlpha
    const int N = 20;

    PidGains gains{0.0, 0.0, 1.0};
    PidController pid(gains, -1000.0, 1000.0);

    // Call j=1: first_call_ guard => D=0, output=0 exactly.
    double out0 = pid.compute(0.0, r * 1.0 * dt, dt);
    ASSERT_REL_NEAR(out0, 0.0, 0.0);  // exact: first_call_ forces D=0

    // Calls k=1..N (j=2..N+1): measurement advances along the ramp.
    for (int k = 1; k <= N; ++k) {
        double m = r * (double)(k + 1) * dt;
        double out = pid.compute(0.0, m, dt);
        // d_k = −r·(1−(1−alpha)^k); tolerance 1e-12 (see comment above).
        double d_k = -r * (1.0 - std::pow(1.0 - alpha, (double)k));
        ASSERT_REL_NEAR(out, d_k, 1e-12);
    }

    std::printf("  [PASS] Check 3: derivative filter\n");
}

// ---------------------------------------------------------------------------
// Check 4: dt<=0 guard
//   compute(…, dt) with dt=0 or dt<0 must return 0.0 and leave integral()
//   unchanged. Only dt<0 makes the integral check bite: at dt=0, e·dt=0 adds
//   nothing even without the guard.
//   Mutation that makes it fail: remove "if (dt <= 0.0) return 0.0;" —
//   without it, raw_derivative = Δm/0 = NaN, so output is NaN, not 0.
// ---------------------------------------------------------------------------
static void check_dt_zero_guard() {
    PidGains gains{1.0, 2.0, 1.0};
    PidController pid(gains, -100.0, 100.0);

    // Prime the integral with one valid step.
    pid.compute(1.0, 0.0, 0.1);
    double integral_before = pid.integral();

    double out = pid.compute(1.0, 0.0, 0.0);
    // ASSERT_REL_NEAR cannot catch NaN (NaN > 0 is false in IEEE 754); use ASSERT_CHECK.
    // Without the guard, raw_derivative = 0/0 = NaN, so output is NaN, not 0.0.
    ASSERT_CHECK(out == 0.0, "dt=0 must return exactly 0.0");
    ASSERT_CHECK(pid.integral() == integral_before, "dt=0 must not change integral()");

    // dt<0: without the guard, integral += e·dt = 1·(−0.1) would move it.
    out = pid.compute(1.0, 0.0, -0.1);
    ASSERT_CHECK(out == 0.0, "dt<0 must return exactly 0.0");
    ASSERT_CHECK(pid.integral() == integral_before, "dt<0 must not change integral()");

    std::printf("  [PASS] Check 4: dt<=0 guard\n");
}

// ---------------------------------------------------------------------------
// Check 5: Anti-windup for negative Ki
//   Ki = -2, Kp = Kd = 0, limits [-1, 1], dt = 0.25.
//   Ordered anti-windup bounds:
//     lo = min(u_min/Ki, u_max/Ki) = min(-1/-2, 1/-2) = min(0.5, -0.5) = -0.5
//     hi = max(u_min/Ki, u_max/Ki) = max(0.5, -0.5)                   =  0.5
//   Drive with e = 100 (setpoint=100, measurement=0): after step 1,
//     integral = 100*0.25 = 25 → clamped to hi = 0.5 (exact: dyadic).
//   Ki * integral = -2 * 0.5 = -1.0 = output_min.
//   Bug (today's main): std::clamp(25, 0.5, -0.5) is UB — with
//     -D_GLIBCXX_ASSERTIONS the process aborts at "Assertion '!(__hi < __lo)' failed";
//     without it, clamp returns hi = -0.5, so Ki*integral = +1. That still lies inside
//     [u_min, u_max], so the per-step range checks pass; the exact integral == 0.5
//     check below is the one that catches it.
// ---------------------------------------------------------------------------
static void check_negative_ki_anti_windup() {
    const double Ki = -2.0, u_min = -1.0, u_max = 1.0, dt = 0.25;
    PidGains gains{0.0, Ki, 0.0};
    PidController pid(gains, u_min, u_max);

    for (int i = 0; i < 10; ++i) {
        double out = pid.compute(100.0, 0.0, dt);
        double ki_int = Ki * pid.integral();
        ASSERT_CHECK(ki_int >= u_min, "Ki*integral must stay >= output_min for Ki < 0");
        ASSERT_CHECK(ki_int <= u_max, "Ki*integral must stay <= output_max for Ki < 0");
        ASSERT_CHECK(out >= u_min, "output must stay >= output_min");
        ASSERT_CHECK(out <= u_max, "output must stay <= output_max");
    }

    // Clamped integral equals hi = max(u_min/Ki, u_max/Ki) = 0.5 exactly (dyadic).
    ASSERT_REL_NEAR(pid.integral(), 0.5, 0.0);

    std::printf("  [PASS] Check 5: negative-Ki anti-windup\n");
}

// ---------------------------------------------------------------------------
// Check 6: Constructor rejects inverted output limits
//   output_min > output_max  → std::invalid_argument (not assert — vanishes in Release).
//   output_min == output_max → no throw (degenerate but valid).
// ---------------------------------------------------------------------------
static void check_invalid_output_limits() {
    bool threw = false;
    try {
        PidController bad(PidGains{1.0, 0.0, 0.0}, 1.0, -1.0);
    } catch (const std::invalid_argument&) {
        threw = true;
    }
    ASSERT_CHECK(threw, "output_min > output_max must throw std::invalid_argument");

    PidController equal(PidGains{1.0, 0.0, 0.0}, 0.0, 0.0);
    (void)equal;

    std::printf("  [PASS] Check 6: invalid output limits rejected\n");
}

// ---------------------------------------------------------------------------
int main() {
    std::printf("PID controller tests:\n");

    test_p_only_step_response();
    test_pi_steady_state();
    test_output_clamping();
    test_anti_windup();
    test_reset();

    check_constant_error_sum();
    check_anti_windup();
    check_derivative_filter();
    check_dt_zero_guard();
    check_negative_ki_anti_windup();
    check_invalid_output_limits();

    std::printf("All PID tests passed.\n");
    return 0;
}
