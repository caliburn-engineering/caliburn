#include "tyre_model.h"

#include "../test/assert_rel.h"
#include <cmath>
#include <cstdio>

using namespace caliburn;

// 0: each use is a zero-slip force that is exactly 0 by construction (x·0 = 0, sin(0) = 0,
// a zero component scaled by anything is 0). Exact comparisons elsewhere state their own reason.
static constexpr double kTol = 0.0;

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
    // Exact: the double nearest 0.02 is exactly 2× the double nearest 0.01 (same mantissa,
    // exponent + 1), and multiplying by 2 commutes with rounding, so C_alpha·0.02 == 2·(C_alpha·0.01).
    ASSERT_CHECK(F2 == 2.0 * F1, "linear tyre force is exactly proportional to slip angle");

    // Exact: C_alpha * 0 = 0; scale = max(0,0,1) = 1, threshold = kTol*1 = 0.
    ASSERT_REL_NEAR(tyre.lateral_force(0.0), 0.0, kTol);

    printf("  linear tyre: F(0.01) = %.1f N — PASS\n", F1);
}

void test_pacejka_shape() {
    PacejkaTyre tyre(default_lateral_params());

    // Exact: sin(C*atan(0)) = sin(0) = 0; scale = 1 (floor), threshold = 0.
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

    // Interior point of the curve at slip = 0.05: Bs = 0.5, inner = Bs + 0.5·(Bs - atan(Bs)),
    // F = D·sin(C·atan(inner)). Reference value from the Magic Formula evaluated at 40 digits
    // (mpmath): 3286.098430929226943…. The double evaluation is ~7 operations, each
    // correctly rounded or within ~1 ulp for atan/sin, so relative error ≲ 1e-15. 1e-13 is ~100×.
    ASSERT_REL_NEAR(tyre.force(0.05), 3286.098430929226943, 1e-13);

    printf("  pacejka shape: peak at slip=%.3f, F_peak=%.1f N — PASS\n", peak, peak_force);
}

void test_pacejka_peak_bounded_by_D() {
    PacejkaTyre tyre(default_lateral_params());

    // Peak force should not exceed D (the peak parameter)
    double peak_force = tyre.peak_force();
    // Exact bound: force(slip) = D·sin(·), libm sin never returns more than 1, and rounding
    // D·s with s ≤ 1 is monotonic, so the product cannot exceed D.
    ASSERT_CHECK(peak_force <= default_lateral_params().D,
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
    // Exact for these inputs, not in general: fl(0.9·5000) = 4500, |F| = 5000 exactly, scale =
    // fl(4500/5000) = fl(0.9), and fl(4000·fl(0.9)) = 3600, fl(3000·fl(0.9)) = 2700 (the
    // representation error of 0.9 is below half an ulp of the products). Then 3600² + 2700² =
    // 20250000 = 4500² exactly, so the recomputed magnitude is exactly F_max.
    ASSERT_CHECK(F_mag == F_max, "saturated magnitude equals F_max exactly");

    // Direction preserved
    double ratio = Fx / Fy;
    // Exact for these inputs: Fx = 3600 and Fy = 2700 exactly (above), and 3600/2700 and
    // 4000/3000 are both 4/3, so both divisions round to the same double.
    ASSERT_CHECK(ratio == 4000.0 / 3000.0, "direction ratio Fx/Fy is preserved by saturation");

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
    // Magnitude bound, not a near-equality. Here |F| ≈ 4385.6 N, about 114 N inside the
    // 4500 N circle, so saturation does not engage and no rounding guard is needed.
    ASSERT_CHECK(F_mag <= 0.9 * Fz, "combined force magnitude stays within the traction circle");
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

    // Not "within 5%": the small-slip deviation is known. Expanding the Magic Formula,
    // F = B·C·D·x·[1 - (Bx)²·(E/3 + 1/3 + C²/6) + O((Bx)^4)], so the relative slope error is
    // (Bx)²·(E/3 + 1/3 + C²/6) = 1e-4·(-1/6 + 1/3 + 3/8) = 5.41667e-5 at x = 0.001. The
    // O((Bx)^4) remainder is 1.6e-9 (40-digit evaluation: 5.416504e-5); round-off in F/x is
    // ~1e-16. 1e-8 is ~6× over the remainder.
    double error = std::abs(slope - expected_slope) / expected_slope;
    const double Bx = B * slip_small;
    const double second_order = Bx * Bx * (default_lateral_params().E / 3.0 + 1.0 / 3.0 + C * C / 6.0);
    ASSERT_CHECK(std::abs(error - second_order) < 1e-8,
                 "initial-slope deviation must match the second-order expansion of the Magic Formula");

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
