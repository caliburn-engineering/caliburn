#include "bicycle_model.h"

#include "../test/assert_rel.h"
#include <cmath>
#include <cstdio>

using namespace caliburn;

// 0: its one use (steady-state yaw rate) evaluates V·delta/(L + K_us·V·V) with the same
// operands in the same order as the implementation, so both sides round identically.
static constexpr double kTol = 0.0;

static VehicleParams default_car() {
    return VehicleParams{
        .m = 1500.0,
        .Iz = 2500.0,
        .Lf = 1.2,
        .Lr = 1.4,
        .Cf = 80000.0,
        .Cr = 80000.0,
    };
}

static VehicleParams oversteer_car() {
    // Rear-biased CG: Lf > Lr with equal stiffness → K_us < 0
    return VehicleParams{
        .m = 1500.0,
        .Iz = 2500.0,
        .Lf = 1.5,
        .Lr = 1.1,
        .Cf = 80000.0,
        .Cr = 80000.0,
    };
}

void test_understeer_gradient() {
    auto car = default_car();
    double K_us = car.understeer_gradient();
    // Lf < Lr and Cf == Cr → Lr/Cf > Lf/Cr → K_us > 0 (understeer)
    ASSERT_CHECK(K_us > 0.0, "Lf=1.2 < Lr=1.4 with equal stiffness gives Lr/Cf > Lf/Cr, so K_us > 0 (understeer)");
    printf("  understeer gradient: %.6f rad/(m/s^2) — PASS\n", K_us);
}

void test_steady_state_yaw_rate() {
    auto car = default_car();
    BicycleModel model(car);

    double V = 20.0;       // 20 m/s
    double delta = 0.02;   // ~1.1 degrees

    double r_ss = model.steady_state_yaw_rate(delta, V);
    double L = car.wheelbase();
    double K_us = car.understeer_gradient();
    double r_expected = V * delta / (L + K_us * V * V);

    // Yaw rate ~0.15 rad/s < 1, so the floor-1 scale makes this the original absolute check.
    ASSERT_REL_NEAR(r_ss, r_expected, kTol);
    printf("  steady-state yaw rate at V=20: %.4f rad/s — PASS\n", r_ss);
}

void test_simulation_converges_to_steady_state() {
    auto car = default_car();
    BicycleModel model(car);

    double V = 20.0;
    double delta = 0.02;
    double dt = 0.001;

    BicycleModel::State x = BicycleModel::State::Zero();

    // Simulate for 5 seconds (should reach steady state)
    for (int i = 0; i < 5000; ++i) {
        model.step_rk4(x, delta, V, dt);
    }

    double r_sim = x(1);
    double r_ss = model.steady_state_yaw_rate(delta, V);

    double error = std::abs(r_sim - r_ss);
    // The error is the decaying transient, not round-off. At V = 20 the lateral/yaw modes
    // are -5.387 ± 2.495j and A is diagonalisable with κ(V) = 7.80, so from x0 = 0:
    // |r - r_ss| ≤ ‖x - x_ss‖ ≤ κ(V)·‖x_ss‖·e^{-5.387·5} = 7.80·0.2885·e^{-26.93} ≈ 4.5e-12.
    // RK4 at h·|λ| ≈ 0.006 tracks e^{hλ} to O((hλ)^5) per step; round-off is ~1e-16 per step,
    // contracted. 1e-11 is ~2× over the bound.
    ASSERT_CHECK(error < 1e-11, "RK4 simulation converges to steady-state yaw rate within 1e-11 rad/s after 5 s");
    printf("  simulation converges to r_ss: error = %.6f — PASS\n", error);
}

void test_stability_understeer() {
    auto car = default_car();
    BicycleModel model(car);

    // Understeer vehicle should be stable at all speeds
    ASSERT_CHECK(model.is_stable(10.0), "understeer car is stable at 10 m/s (all eigenvalues have negative real parts)");
    ASSERT_CHECK(model.is_stable(30.0), "understeer car is stable at 30 m/s");
    ASSERT_CHECK(model.is_stable(50.0), "understeer car is stable at 50 m/s");
    ASSERT_CHECK(model.critical_speed() == std::numeric_limits<double>::infinity(),
                 "understeer car (K_us > 0) returns infinity for critical speed");
    printf("  understeer vehicle stable at all speeds — PASS\n");
}

void test_stability_oversteer() {
    auto car = oversteer_car();
    BicycleModel model(car);

    double V_crit = model.critical_speed();
    ASSERT_CHECK(std::isfinite(V_crit), "oversteer car has a finite critical speed");
    ASSERT_CHECK(V_crit > 0.0, "critical speed is positive");

    // Should be stable below critical speed
    ASSERT_CHECK(model.is_stable(V_crit * 0.8), "oversteer car is stable below V_crit (at 0.8 * V_crit)");
    // Should be unstable above critical speed
    ASSERT_CHECK(!model.is_stable(V_crit * 1.2), "oversteer car is unstable above V_crit (at 1.2 * V_crit)");

    printf("  oversteer vehicle: V_crit = %.1f m/s — PASS\n", V_crit);
}

void test_eigenvalues_negative_real_parts() {
    auto car = default_car();
    BicycleModel model(car);

    auto eigs = model.eigenvalues(20.0);
    ASSERT_CHECK(eigs(0).real() < 0.0, "first eigenvalue has negative real part for stable understeer car at 20 m/s");
    ASSERT_CHECK(eigs(1).real() < 0.0, "second eigenvalue has negative real part for stable understeer car at 20 m/s");
    printf("  eigenvalues at V=20: (%.2f + %.2fj), (%.2f + %.2fj) — PASS\n",
           eigs(0).real(), eigs(0).imag(), eigs(1).real(), eigs(1).imag());
}

int main() {
    printf("=== Bicycle Model Tests ===\n");
    test_understeer_gradient();
    test_steady_state_yaw_rate();
    test_simulation_converges_to_steady_state();
    test_stability_understeer();
    test_stability_oversteer();
    test_eigenvalues_negative_real_parts();
    printf("All tests passed.\n");
    return 0;
}
