#include "tyre_model.h"

#include "../test/assert_rel.h"
#include <cmath>
#include <cstdio>

using namespace caliburn;

// TODO(#75): tolerance unjustified — every use measures exactly 0 in Release (linear
// proportionality, zero-slip forces, saturated magnitude and direction ratio), so 1e-3 N
// is far looser than the arithmetic needs. Kept per #75: no tolerance values change.
static constexpr double kTol = 1e-3;

static PacejkaParams default_lateral_params() {
    return PacejkaParams{
        .B = 10.0,
        .C = 1.5,
        .D = 5000.0,   // mu * Fz = 0.9 * ~5500 N (one corner of 1500 kg car)
        .E = -0.5,
    };
}

static PacejkaParams default_longitudinal_params() {
    return PacejkaParams{
        .B = 12.0,
        .C = 1.6,
        .D = 5000.0,
        .E = -0.3,
    };
}

void test_linear_tyre() {
    LinearTyre tyre(80000.0);  // 80 kN/rad

    // Force proportional to slip angle
    double F1 = tyre.lateral_force(0.01);   // ~1 degree
    double F2 = tyre.lateral_force(0.02);   // ~2 degrees
    // 1e-3 abs: linear tyre is exactly proportional; F1≈800 N, F2≈1600 N — absolute is tighter than relative
    ASSERT_CHECK(std::abs(F2 - 2.0 * F1) < kTol, "linear tyre force is exactly proportional to slip angle");

    // Zero slip = zero force
    // 1e-3: stiffness * 0 = 0 exactly; floor-1 scale makes this an absolute check on a near-zero value
    ASSERT_REL_NEAR(tyre.lateral_force(0.0), 0.0, kTol);

    printf("  linear tyre: F(0.01) = %.1f N — PASS\n", F1);
}

void test_pacejka_shape() {
    PacejkaTyre tyre(default_lateral_params());

    // Zero slip = zero force
    // 1e-3: sin(C*atan(0)) = sin(0) = 0 exactly; floor-1 scale makes this an absolute check
    ASSERT_REL_NEAR(tyre.force(0.0), 0.0, kTol);

    // Force increases initially
    double F_small = tyre.force(0.02);
    double F_larger = tyre.force(0.05);
    ASSERT_CHECK(F_larger > F_small, "force increases from 0.02 to 0.05 rad on the initial slope");

    // Peak exists and is positive
    double peak = tyre.peak_slip();
    double peak_force = tyre.peak_force();
    ASSERT_CHECK(peak > 0.0 && peak < 0.5, "peak slip is in (0, 0.5) rad for B=10, C=1.5");
    ASSERT_CHECK(peak_force > 0.0, "peak force is positive because D = 5000 N > 0");

    // Force at peak is greater than force at 2x peak (post-peak drop)
    double F_post = tyre.force(2.0 * peak);
    ASSERT_CHECK(peak_force > F_post, "force drops past the peak (post-peak regime of the Magic Formula)");

    printf("  pacejka shape: peak at slip=%.3f, F_peak=%.1f N — PASS\n", peak, peak_force);
}

void test_pacejka_peak_bounded_by_D() {
    PacejkaTyre tyre(default_lateral_params());

    // Peak force should not exceed D (the peak parameter)
    double peak_force = tyre.peak_force();
    // TODO(#75): tolerance unjustified — sin() <= 1 bounds the peak by D exactly, and the
    // measured peak/D is 1.0 (Release); the 1% margin is not needed for rounding.
    ASSERT_CHECK(peak_force <= default_lateral_params().D * 1.01,
                 "Pacejka peak force is bounded by D (the peak scale factor)");

    printf("  peak force (%.1f) <= D (%.1f) — PASS\n",
           peak_force, default_lateral_params().D);
}

void test_traction_circle_within() {
    TractionCircle circle(0.9);
    double Fz = 5000.0;

    // Inside the circle
    ASSERT_CHECK(circle.is_within(1000.0, 1000.0, Fz), "sqrt(1000^2+1000^2)=1414 N is well inside the 4500 N circle");
    // On the boundary
    double F_max = 0.9 * 5000.0;  // 4500 N
    ASSERT_CHECK(circle.is_within(F_max, 0.0, Fz), "F_max on one axis lies on the circle boundary (F_max^2 <= F_max^2)");
    ASSERT_CHECK(circle.is_within(0.0, F_max, Fz), "F_max on the other axis lies on the circle boundary");
    // Outside
    ASSERT_CHECK(!circle.is_within(4000.0, 3000.0, Fz), "sqrt(4000^2+3000^2)=5000 N exceeds the 4500 N circle");

    printf("  traction circle bounds check — PASS\n");
}

void test_traction_circle_saturate() {
    TractionCircle circle(0.9);
    double Fz = 5000.0;
    double F_max = 0.9 * 5000.0;

    // Force outside circle gets scaled back
    double Fx = 4000.0;
    double Fy = 3000.0;
    circle.saturate(Fx, Fy, Fz);

    double F_mag = std::sqrt(Fx * Fx + Fy * Fy);
    // 1e-3 abs: saturate must scale exactly to F_max=4500 N; absolute is tighter than relative at this magnitude
    ASSERT_CHECK(std::abs(F_mag - F_max) < kTol, "saturated magnitude equals F_max exactly");

    // Direction preserved
    double ratio = Fx / Fy;
    // 1e-3 abs: ratio≈1.33 > 1, so relative-1 floor gives 1e-3*1.33; kept absolute for a stricter bound
    ASSERT_CHECK(std::abs(ratio - 4000.0 / 3000.0) < kTol, "direction ratio Fx/Fy is preserved by saturation");

    printf("  traction circle saturate: F_mag=%.1f, ratio preserved — PASS\n", F_mag);
}

void test_combined_slip() {
    CombinedSlipTyre tyre(default_longitudinal_params(), default_lateral_params(), 0.9);

    double Fz = 5000.0;
    double Fx, Fy;

    // Pure longitudinal
    tyre.forces(0.05, 0.0, Fz, Fx, Fy);
    // 1e-3: alpha=0 gives exactly zero lateral force; floor-1 scale makes this an absolute check
    ASSERT_REL_NEAR(Fy, 0.0, kTol);
    ASSERT_CHECK(Fx > 0.0, "longitudinal force is positive for positive longitudinal slip");
    printf("  combined (pure long): Fx=%.1f, Fy=%.3f\n", Fx, Fy);

    // Pure lateral
    tyre.forces(0.0, 0.05, Fz, Fx, Fy);
    // 1e-3: kappa=0 gives exactly zero longitudinal force; floor-1 scale makes this an absolute check
    ASSERT_REL_NEAR(Fx, 0.0, kTol);
    ASSERT_CHECK(Fy > 0.0, "lateral force is positive for positive lateral slip");
    printf("  combined (pure lat):  Fx=%.3f, Fy=%.1f\n", Fx, Fy);

    // Combined — both non-zero, magnitude <= mu*Fz
    tyre.forces(0.05, 0.05, Fz, Fx, Fy);
    double F_mag = std::sqrt(Fx * Fx + Fy * Fy);
    // kTol guards the floating-point boundary at exactly F_max; this is a magnitude bound, not a near-equality
    ASSERT_CHECK(F_mag <= 0.9 * Fz + kTol, "combined force magnitude stays within the traction circle (1e-3 N float guard at boundary)");
    ASSERT_CHECK(Fx > 0.0 && Fy > 0.0, "both force components are positive for positive slip inputs");
    printf("  combined (both): Fx=%.1f, Fy=%.1f, |F|=%.1f — PASS\n", Fx, Fy, F_mag);
}

void test_force_slip_curve_initial_slope() {
    PacejkaTyre tyre(default_lateral_params());

    // At very small slip, the curve should be approximately linear
    // Initial slope = B * C * D (for the Magic Formula at slip=0)
    double slip_small = 0.001;
    double F = tyre.force(slip_small);
    double slope = F / slip_small;

    double B = default_lateral_params().B;
    double C = default_lateral_params().C;
    double D = default_lateral_params().D;
    double expected_slope = B * C * D;

    // Should be within 5% (approximation valid only at very small slip)
    double error = std::abs(slope - expected_slope) / expected_slope;
    // TODO(#75): tolerance unjustified — measured relative slope error at slip=0.001 is 5.4e-5
    // (Release), so 5% is ~1000x looser than the small-slip approximation error.
    ASSERT_CHECK(error < 0.05, "initial slope is within 5% of B*C*D (small-angle approximation at slip=0.001)");

    printf("  initial slope: %.0f vs expected %.0f (error %.1f%%) — PASS\n",
           slope, expected_slope, error * 100.0);
}

int main() {
    printf("=== Tyre Model Tests ===\n");
    test_linear_tyre();
    test_pacejka_shape();
    test_pacejka_peak_bounded_by_D();
    test_traction_circle_within();
    test_traction_circle_saturate();
    test_combined_slip();
    test_force_slip_curve_initial_slope();
    printf("All tests passed.\n");
    return 0;
}
