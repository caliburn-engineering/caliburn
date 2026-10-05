#include "trajectory_planner.h"
#include "assert_rel.h"

#include <cmath>
#include <cstdio>

using namespace caliburn;

static constexpr double TOL = 1e-10;

// ---------------------------------------------------------------------------
// 1. Cubic: boundary conditions satisfied
// ---------------------------------------------------------------------------
static void test_cubic_boundary_conditions() {
    CubicTrajectory traj(1.0, 0.5, 3.0, -0.5, 2.0);

    // 1e-10: cubic coefficients from 4×4 linear solve; rounding ≤ O(cond)×2⁻⁵³ ≈ 1e-14; 1e-10 adds ~10⁴× margin
    ASSERT_REL_NEAR(traj.position(0.0), 1.0,   TOL);   // |expected|=1 ≤ 1: relative scale = 1
    ASSERT_REL_NEAR(traj.velocity(0.0), 0.5,   TOL);   // |expected|=0.5 ≤ 1
    ASSERT_CHECK(std::abs(traj.position(2.0) - 3.0) < TOL,   // |expected|=3 > 1: ASSERT_REL_NEAR would be 3× looser
                 "cubic position at t=2 must equal qf=3.0 to within 1e-10");
    ASSERT_REL_NEAR(traj.velocity(2.0), -0.5,  TOL);   // |expected|=0.5 ≤ 1
    std::printf("  [PASS] Cubic boundary conditions\n");
}

// ---------------------------------------------------------------------------
// 2. Cubic: rest-to-rest has zero velocity at endpoints
// ---------------------------------------------------------------------------
static void test_cubic_rest_to_rest() {
    CubicTrajectory traj(0.0, 0.0, 1.0, 0.0, 1.0);

    // 1e-10: same 4×4 solve as above; boundary velocity = 0 so absolute and relative thresholds coincide
    ASSERT_REL_NEAR(traj.velocity(0.0), 0.0, TOL);
    ASSERT_REL_NEAR(traj.velocity(1.0), 0.0, TOL);
    ASSERT_REL_NEAR(traj.position(1.0), 1.0, TOL);  // |expected|=1 ≤ 1
    std::printf("  [PASS] Cubic rest-to-rest\n");
}

// ---------------------------------------------------------------------------
// 3. MinJerk: boundary conditions (rest-to-rest)
// ---------------------------------------------------------------------------
static void test_minjerk_boundary_conditions() {
    MinJerkTrajectory traj(0.0, 2.0, 1.0);

    // 1e-10: min-jerk coefficients from 6-condition linear solve (T=1, all params O(1));
    //        rounding ≤ O(cond)×2⁻⁵³ ≈ 1e-13; 1e-10 adds ~1000× margin
    ASSERT_REL_NEAR(traj.position(0.0), 0.0, TOL);   // compare to 0: absolute ≡ relative
    ASSERT_CHECK(std::abs(traj.position(1.0) - 2.0) < TOL,   // |expected|=2 > 1: ASSERT_REL_NEAR would be 2× looser
                 "min-jerk position at t=T must equal qf=2.0 to within 1e-10");
    ASSERT_REL_NEAR(traj.velocity(0.0),     0.0, TOL);
    ASSERT_REL_NEAR(traj.velocity(1.0),     0.0, TOL);
    ASSERT_REL_NEAR(traj.acceleration(0.0), 0.0, TOL);
    ASSERT_REL_NEAR(traj.acceleration(1.0), 0.0, TOL);
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

    ASSERT_CHECK(v_mid > v_quarter, "min-jerk peak velocity must be at midpoint, greater than v(T/4)");
    ASSERT_CHECK(v_mid > v_three_quarter, "min-jerk peak velocity must be at midpoint, greater than v(3T/4)");
    // v_quarter ≈ v_three_quarter ≈ 0.94 ≤ 1, so relative scale = 1 and absolute threshold equals ASSERT_REL_NEAR
    ASSERT_REL_NEAR(v_quarter, v_three_quarter, TOL);  // symmetric about midpoint; |values| < 1
    std::printf("  [PASS] MinJerk peak velocity at midpoint (v=%.4f)\n", v_mid);
}

// ---------------------------------------------------------------------------
// 5. Trapezoidal: reaches target position
// ---------------------------------------------------------------------------
static void test_trapezoidal_reaches_target() {
    TrapezoidalTrajectory traj(0.0, 10.0, 2.0, 1.0);

    double T = traj.duration();
    // |expected|=10 > 1: ASSERT_REL_NEAR would allow 10× larger error; keep absolute bound
    // 1e-8: piecewise quadratic; O(1) floating-point operations; rounding ≈ 1e-16; 1e-8 is a loose sanity check
    ASSERT_CHECK(std::abs(traj.position(T) - 10.0) < 1e-8,
                 "trapezoidal position at T must equal qf=10.0 to within 1e-8");
    ASSERT_REL_NEAR(traj.velocity(0.0), 0.0, TOL);  // rest start: absolute ≡ relative
    ASSERT_REL_NEAR(traj.velocity(T),   0.0, TOL);  // rest end
    std::printf("  [PASS] Trapezoidal reaches target (T=%.4f)\n", T);
}

// ---------------------------------------------------------------------------
// 6. Trapezoidal: triangular profile (short distance)
// ---------------------------------------------------------------------------
static void test_trapezoidal_triangular() {
    // v_max=10, a_max=1 => needs 100 units to reach cruise.  0.5 << 100 => triangular.
    TrapezoidalTrajectory traj(0.0, 0.5, 10.0, 1.0);

    double T = traj.duration();
    // |expected|=0.5 ≤ 1: relative scale = max(0.5, 0.5, 1) = 1; 1e-8 threshold same as absolute
    ASSERT_REL_NEAR(traj.position(T), 0.5, 1e-8);  // 1e-8: piecewise quadratic with few operations
    // Peak velocity should be less than v_max
    double v_peak = traj.velocity(T / 2.0);
    ASSERT_CHECK(v_peak < 10.0, "triangular: peak velocity must be less than v_max=10");
    ASSERT_CHECK(v_peak > 0.0,  "triangular: peak velocity must be positive (forward motion)");
    std::printf("  [PASS] Trapezoidal triangular profile (v_peak=%.4f)\n", v_peak);
}

// ---------------------------------------------------------------------------
// 7. Trapezoidal: reverse direction
// ---------------------------------------------------------------------------
static void test_trapezoidal_reverse() {
    TrapezoidalTrajectory traj(5.0, 0.0, 2.0, 1.0);

    double T = traj.duration();
    // comparing to 0: absolute and relative thresholds coincide (scale floor = 1)
    ASSERT_REL_NEAR(traj.position(T), 0.0, 1e-8);  // 1e-8: piecewise quadratic with few operations
    // Velocity should be negative
    double v_mid = traj.velocity(T / 2.0);
    ASSERT_CHECK(v_mid < 0.0, "trapezoidal reverse: mid-trajectory velocity must be negative");
    std::printf("  [PASS] Trapezoidal reverse direction\n");
}

// ---------------------------------------------------------------------------
// 8. Trapezoidal: check 1 — closed-form timing (oracle-free)
// ---------------------------------------------------------------------------
// Trapezoidal case: q0=0, qf=8, v_max=2, a_max=1.
//   d=8, dist_to_cruise = v_max²/a_max = 4.  8 ≥ 4 → trapezoidal branch.
//   t_a = v_max/a_max = 2.  T = 2·t_a + (d−4)/v_max = 4+2 = 6.
//
// Triangular case: q0=0, qf=1, v_max=2, a_max=1.
//   d=1 < 4 → triangular branch.  v_p = √(d·a_max) = 1.  t_a = 1.  T = 2.
//
// All expected values are small integers — exact in IEEE 754, tolerance = 0.0.
//
// ε = 1e-6: t_a ∈ {1.0, 2.0} are exact doubles; float spacing there is
// ~4×10⁻¹⁶, so t_a ± ε land firmly inside the intended phase, 10¹⁰ × away
// from the boundary.
static void test_trapezoidal_check1_timing() {
    const double eps = 1e-6;

    // --- Trapezoidal, forward ---
    {
        TrapezoidalTrajectory traj(0.0, 8.0, 2.0, 1.0);
        // T = 4 + 2 = 6 (integers, exact).
        ASSERT_REL_NEAR(traj.duration(), 6.0, 0.0);
        // Peak velocity at T/2 = 3 (cruise phase): |v| = v_max = 2.
        ASSERT_REL_NEAR(std::fabs(traj.velocity(3.0)), 2.0, 0.0);
        // Accel→cruise switch at t_a = 2: left → +a_max, right → 0.
        ASSERT_REL_NEAR(traj.acceleration(2.0 - eps),  1.0, 0.0);
        ASSERT_REL_NEAR(traj.acceleration(2.0 + eps),  0.0, 0.0);
        // Cruise→decel switch at T−t_a = 4: left → 0, right → −a_max.
        ASSERT_REL_NEAR(traj.acceleration(4.0 - eps),  0.0, 0.0);
        ASSERT_REL_NEAR(traj.acceleration(4.0 + eps), -1.0, 0.0);
    }
    // --- Trapezoidal, backward — signs flip ---
    {
        TrapezoidalTrajectory traj(8.0, 0.0, 2.0, 1.0);
        ASSERT_REL_NEAR(traj.duration(), 6.0, 0.0);
        ASSERT_REL_NEAR(std::fabs(traj.velocity(3.0)), 2.0, 0.0);
        ASSERT_REL_NEAR(traj.acceleration(2.0 - eps), -1.0, 0.0);
        ASSERT_REL_NEAR(traj.acceleration(2.0 + eps),  0.0, 0.0);
        ASSERT_REL_NEAR(traj.acceleration(4.0 - eps),  0.0, 0.0);
        ASSERT_REL_NEAR(traj.acceleration(4.0 + eps),  1.0, 0.0);
    }
    // --- Triangular, forward ---
    // t_a = T−t_a = T/2 = 1: no cruise phase, single switch.
    // velocity() uses <=, so velocity(1.0) → accel formula: a·t = 1·1 = v_p.
    {
        TrapezoidalTrajectory traj(0.0, 1.0, 2.0, 1.0);
        ASSERT_REL_NEAR(traj.duration(), 2.0, 0.0);
        ASSERT_REL_NEAR(std::fabs(traj.velocity(1.0)), 1.0, 0.0);
        // Single accel→decel switch at t_a = 1: left → +a_max, right → −a_max.
        ASSERT_REL_NEAR(traj.acceleration(1.0 - eps),  1.0, 0.0);
        ASSERT_REL_NEAR(traj.acceleration(1.0 + eps), -1.0, 0.0);
    }
    // --- Triangular, backward ---
    {
        TrapezoidalTrajectory traj(1.0, 0.0, 2.0, 1.0);
        ASSERT_REL_NEAR(traj.duration(), 2.0, 0.0);
        ASSERT_REL_NEAR(std::fabs(traj.velocity(1.0)), 1.0, 0.0);
        ASSERT_REL_NEAR(traj.acceleration(1.0 - eps), -1.0, 0.0);
        ASSERT_REL_NEAR(traj.acceleration(1.0 + eps),  1.0, 0.0);
    }
    std::printf("  [PASS] Trapezoidal check 1: closed-form timing\n");
}

// ---------------------------------------------------------------------------
// 9. Trapezoidal: check 2 — boundary conditions
// ---------------------------------------------------------------------------
// position(0) = q0, position(T) = qf, velocity(0) = velocity(T) = 0.
// At t=0 accel formula gives q0_ + 0.  At t=T decel formula gives qf_ − 0.
// Velocity at endpoints: a·0 = 0 and a·(T−T) = 0.  All integers, exact.
static void test_trapezoidal_check2_boundary_conditions() {
    auto check = [](double q0, double qf) {
        TrapezoidalTrajectory traj(q0, qf, 2.0, 1.0);
        double T = traj.duration();
        ASSERT_REL_NEAR(traj.position(0.0), q0,  0.0);
        ASSERT_REL_NEAR(traj.position(T),   qf,  0.0);
        ASSERT_REL_NEAR(traj.velocity(0.0), 0.0, 0.0);
        ASSERT_REL_NEAR(traj.velocity(T),   0.0, 0.0);
    };
    check(0.0, 8.0);  // trapezoidal fwd
    check(8.0, 0.0);  // trapezoidal bwd
    check(0.0, 1.0);  // triangular fwd
    check(1.0, 0.0);  // triangular bwd

    // The endpoints hold by construction (the decel branch returns
    // qf − ½·a·(T−t)², which is qf at t=T), so they can't catch a wrong phase
    // formula. Interior positions, hand-derived from q = q0 ± ½·a·t² (accel),
    // q_a ± v·(t−t_a) (cruise), qf ∓ ½·a·(T−t)² (decel), can. All operands are
    // dyadic, so tolerance 0.0.
    //   Trapezoidal 0→8 (t_a=2, T=6): q(1)=0.5, q(2)=2, q(3)=2+2·1=4, q(5)=8−0.5=7.5
    //   Triangular  0→1 (t_a=1, T=2): q(0.5)=0.125, q(1)=0.5, q(1.5)=1−0.125=0.875
    // Reverse runs mirror them: q_rev(t) = q0 + qf − q_fwd(t).
    {
        TrapezoidalTrajectory fwd(0.0, 8.0, 2.0, 1.0), rev(8.0, 0.0, 2.0, 1.0);
        const double pts[][2] = {{1.0, 0.5}, {2.0, 2.0}, {3.0, 4.0}, {5.0, 7.5}};
        for (const auto& p : pts) {
            ASSERT_REL_NEAR(fwd.position(p[0]), p[1], 0.0);
            ASSERT_REL_NEAR(rev.position(p[0]), 8.0 - p[1], 0.0);
        }
    }
    {
        TrapezoidalTrajectory fwd(0.0, 1.0, 2.0, 1.0), rev(1.0, 0.0, 2.0, 1.0);
        const double pts[][2] = {{0.5, 0.125}, {1.0, 0.5}, {1.5, 0.875}};
        for (const auto& p : pts) {
            ASSERT_REL_NEAR(fwd.position(p[0]), p[1], 0.0);
            ASSERT_REL_NEAR(rev.position(p[0]), 1.0 - p[1], 0.0);
        }
    }

    // Outside [0, T] the profile holds its end state: t is clamped.
    {
        TrapezoidalTrajectory traj(0.0, 8.0, 2.0, 1.0);
        ASSERT_REL_NEAR(traj.position(-1.0), 0.0, 0.0);
        ASSERT_REL_NEAR(traj.position(7.0),  8.0, 0.0);
        ASSERT_REL_NEAR(traj.velocity(-1.0), 0.0, 0.0);
        ASSERT_REL_NEAR(traj.velocity(7.0),  0.0, 0.0);
    }
    std::printf("  [PASS] Trapezoidal check 2: boundary conditions\n");
}

// ---------------------------------------------------------------------------
// 10. Trapezoidal: check 3 — bounds and continuity
// ---------------------------------------------------------------------------
// ε = 1e-6 (see check 1 for why it is safe).
// Continuity bound: for a C⁰ function |f(t_sw−ε) − f(t_sw+ε)| ≤ 2·rate·ε
// where rate = v_max for position and a_max for velocity.  3× safety margin.
static void test_trapezoidal_check3_bounds_and_continuity() {
    const double eps = 1e-6;

    auto bounds_grid = [&](TrapezoidalTrajectory& traj, double a_max, double peak_v,
                           double T) {
        for (int i = 0; i <= 100; ++i) {
            double t = T * i / 100.0;
            ASSERT_CHECK(std::fabs(traj.acceleration(t)) <= a_max + 1e-12,
                         "|accel| <= a_max on grid");
            ASSERT_CHECK(std::fabs(traj.velocity(t)) <= peak_v + 1e-12,
                         "|velocity| <= peak_v on grid");
        }
    };
    auto bounds_pts = [&](TrapezoidalTrajectory& traj, double a_max, double peak_v,
                          double t1, double t2, double t3) {
        for (double t : {t1, t2, t3}) {
            ASSERT_CHECK(std::fabs(traj.acceleration(t)) <= a_max + 1e-12,
                         "|accel| <= a_max at explicit point");
            ASSERT_CHECK(std::fabs(traj.velocity(t)) <= peak_v + 1e-12,
                         "|velocity| <= peak_v at explicit point");
        }
    };
    auto continuity = [&](TrapezoidalTrajectory& traj, double a_max, double peak_v,
                          double t_sw) {
        // position and velocity are C⁰: difference across ±ε ≤ 2·rate·ε.
        ASSERT_CHECK(
            std::fabs(traj.position(t_sw - eps) - traj.position(t_sw + eps))
                <= 3.0 * peak_v * eps,
            "position continuous at switch time");
        ASSERT_CHECK(
            std::fabs(traj.velocity(t_sw - eps) - traj.velocity(t_sw + eps))
                <= 3.0 * a_max * eps,
            "velocity continuous at switch time");
    };

    // --- Trapezoidal, forward: t_a=2, T=6, switch times {2, 4}, T/2=3 ---
    {
        TrapezoidalTrajectory traj(0.0, 8.0, 2.0, 1.0);
        bounds_grid(traj, 1.0, 2.0, 6.0);
        bounds_pts(traj, 1.0, 2.0, /*t_a*/2.0, /*T-t_a*/4.0, /*T/2*/3.0);
        continuity(traj, 1.0, 2.0, 2.0);
        continuity(traj, 1.0, 2.0, 4.0);
    }
    // --- Trapezoidal, backward ---
    {
        TrapezoidalTrajectory traj(8.0, 0.0, 2.0, 1.0);
        bounds_grid(traj, 1.0, 2.0, 6.0);
        bounds_pts(traj, 1.0, 2.0, 2.0, 4.0, 3.0);
        continuity(traj, 1.0, 2.0, 2.0);
        continuity(traj, 1.0, 2.0, 4.0);
    }
    // --- Triangular, forward: t_a=T-t_a=T/2=1, T=2 ---
    {
        TrapezoidalTrajectory traj(0.0, 1.0, 2.0, 1.0);
        bounds_grid(traj, 1.0, 1.0, 2.0);
        bounds_pts(traj, 1.0, 1.0, /*t_a*/1.0, /*T-t_a*/1.0, /*T/2*/1.0);
        continuity(traj, 1.0, 1.0, 1.0);
    }
    // --- Triangular, backward ---
    {
        TrapezoidalTrajectory traj(1.0, 0.0, 2.0, 1.0);
        bounds_grid(traj, 1.0, 1.0, 2.0);
        bounds_pts(traj, 1.0, 1.0, 1.0, 1.0, 1.0);
        continuity(traj, 1.0, 1.0, 1.0);
    }
    std::printf("  [PASS] Trapezoidal check 3: bounds and continuity\n");
}

// ---------------------------------------------------------------------------
// 11. Trapezoidal: check 4 — boundary case d = v_max²/a_max
// ---------------------------------------------------------------------------
// q0=0, qf=4, v_max=2, a_max=1 → d = 4 = v_max²/a_max.
// Implementation takes the trapezoidal branch (4 < 4 is false):
//   cruise_dist = 0, cruise_time = 0, T = 2·t_a = 2·(v_max/a_max) = 4.
// Triangular branch would give: v_p = √(4·1) = 2 = v_max, T = 2·(2/1) = 4.
// Both branches yield T = 2·v_max/a_max = 4. Exact (integers).
static void test_trapezoidal_check4_boundary_case() {
    TrapezoidalTrajectory traj(0.0, 4.0, 2.0, 1.0);
    ASSERT_REL_NEAR(traj.duration(), 4.0, 0.0);
    std::printf("  [PASS] Trapezoidal check 4: boundary case d = v_max^2/a_max\n");
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
    test_trapezoidal_check1_timing();
    test_trapezoidal_check2_boundary_conditions();
    test_trapezoidal_check3_bounds_and_continuity();
    test_trapezoidal_check4_boundary_case();

    std::printf("All trajectory planner tests passed.\n");
    return 0;
}
