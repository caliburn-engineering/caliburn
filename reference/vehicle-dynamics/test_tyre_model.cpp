#include "tyre_model.h"

#include <cassert>
#include <cmath>
#include <cstdio>

using namespace caliburn;

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
    assert(std::abs(F2 - 2.0 * F1) < kTol);

    // Zero slip = zero force
    assert(std::abs(tyre.lateral_force(0.0)) < kTol);

    printf("  linear tyre: F(0.01) = %.1f N — PASS\n", F1);
}

void test_pacejka_shape() {
    PacejkaTyre tyre(default_lateral_params());

    // Zero slip = zero force
    assert(std::abs(tyre.force(0.0)) < kTol);

    // Force increases initially
    double F_small = tyre.force(0.02);
    double F_larger = tyre.force(0.05);
    assert(F_larger > F_small);

    // Peak exists and is positive
    double peak = tyre.peak_slip();
    double peak_force = tyre.peak_force();
    assert(peak > 0.0 && peak < 0.5);
    assert(peak_force > 0.0);

    // Force at peak is greater than force at 2x peak (post-peak drop)
    double F_post = tyre.force(2.0 * peak);
    assert(peak_force > F_post);

    printf("  pacejka shape: peak at slip=%.3f, F_peak=%.1f N — PASS\n", peak, peak_force);
}

void test_pacejka_peak_bounded_by_D() {
    PacejkaTyre tyre(default_lateral_params());

    // Peak force should not exceed D (the peak parameter)
    double peak_force = tyre.peak_force();
    assert(peak_force <= default_lateral_params().D * 1.01);  // small tolerance

    printf("  peak force (%.1f) <= D (%.1f) — PASS\n",
           peak_force, default_lateral_params().D);
}

void test_traction_circle_within() {
    TractionCircle circle(0.9);
    double Fz = 5000.0;

    // Inside the circle
    assert(circle.is_within(1000.0, 1000.0, Fz));
    // On the boundary
    double F_max = 0.9 * 5000.0;  // 4500 N
    assert(circle.is_within(F_max, 0.0, Fz));
    assert(circle.is_within(0.0, F_max, Fz));
    // Outside
    assert(!circle.is_within(4000.0, 3000.0, Fz));  // sqrt(4000^2+3000^2)=5000 > 4500

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
    assert(std::abs(F_mag - F_max) < kTol);

    // Direction preserved
    double ratio = Fx / Fy;
    assert(std::abs(ratio - 4000.0 / 3000.0) < kTol);

    printf("  traction circle saturate: F_mag=%.1f, ratio preserved — PASS\n", F_mag);
}

void test_combined_slip() {
    CombinedSlipTyre tyre(default_longitudinal_params(), default_lateral_params(), 0.9);

    double Fz = 5000.0;
    double Fx, Fy;

    // Pure longitudinal
    tyre.forces(0.05, 0.0, Fz, Fx, Fy);
    assert(std::abs(Fy) < kTol);
    assert(Fx > 0.0);
    printf("  combined (pure long): Fx=%.1f, Fy=%.3f\n", Fx, Fy);

    // Pure lateral
    tyre.forces(0.0, 0.05, Fz, Fx, Fy);
    assert(std::abs(Fx) < kTol);
    assert(Fy > 0.0);
    printf("  combined (pure lat):  Fx=%.3f, Fy=%.1f\n", Fx, Fy);

    // Combined — both non-zero, magnitude <= mu*Fz
    tyre.forces(0.05, 0.05, Fz, Fx, Fy);
    double F_mag = std::sqrt(Fx * Fx + Fy * Fy);
    assert(F_mag <= 0.9 * Fz + kTol);
    assert(Fx > 0.0 && Fy > 0.0);
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
    assert(error < 0.05);

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
