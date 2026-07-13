#include "bicycle_model.h"

#include <cassert>
#include <cmath>
#include <cstdio>

using namespace caliburn;

static constexpr double kTol = 1e-4;

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
    assert(K_us > 0.0);
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

    assert(std::abs(r_ss - r_expected) < kTol);
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
    assert(error < 0.001);
    printf("  simulation converges to r_ss: error = %.6f — PASS\n", error);
}

void test_stability_understeer() {
    auto car = default_car();
    BicycleModel model(car);

    // Understeer vehicle should be stable at all speeds
    assert(model.is_stable(10.0));
    assert(model.is_stable(30.0));
    assert(model.is_stable(50.0));
    assert(model.critical_speed() == std::numeric_limits<double>::infinity());
    printf("  understeer vehicle stable at all speeds — PASS\n");
}

void test_stability_oversteer() {
    auto car = oversteer_car();
    BicycleModel model(car);

    double V_crit = model.critical_speed();
    assert(std::isfinite(V_crit));
    assert(V_crit > 0.0);

    // Should be stable below critical speed
    assert(model.is_stable(V_crit * 0.8));
    // Should be unstable above critical speed
    assert(!model.is_stable(V_crit * 1.2));

    printf("  oversteer vehicle: V_crit = %.1f m/s — PASS\n", V_crit);
}

void test_eigenvalues_negative_real_parts() {
    auto car = default_car();
    BicycleModel model(car);

    auto eigs = model.eigenvalues(20.0);
    assert(eigs(0).real() < 0.0);
    assert(eigs(1).real() < 0.0);
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
