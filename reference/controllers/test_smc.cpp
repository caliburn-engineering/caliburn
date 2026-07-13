#include "smc.h"
#include <cassert>
#include <cmath>
#include <iostream>
#include <vector>

using namespace caliburn;
using Eigen::VectorXd;

// --- Test plant: double integrator with disturbance ---
// x = [position, velocity], x_dot = [velocity, u + d]
// Sliding surface: s = x_dot + lambda * x (lambda = 2.0)
// Equivalent control: u_eq = -lambda * x_dot (from setting s_dot = 0 with d=0)

static constexpr double LAMBDA = 2.0;
static constexpr double DT = 0.001;
static constexpr int SIM_STEPS = 5000;  // 5 seconds

struct SimResult {
    std::vector<double> position;
    std::vector<double> velocity;
    std::vector<double> control;
    std::vector<double> sliding_var;
};

SimResult simulate(SlidingModeController& ctrl, double d_const, double x0, double v0) {
    SimResult res;
    VectorXd x(2);
    x << x0, v0;

    for (int i = 0; i < SIM_STEPS; ++i) {
        double u = ctrl.compute(x, DT);
        res.position.push_back(x(0));
        res.velocity.push_back(x(1));
        res.control.push_back(u);
        res.sliding_var.push_back(ctrl.getSlidingVariable());

        // Euler integration: x_dot = [v, u + d]
        double x_new = x(0) + x(1) * DT;
        double v_new = x(1) + (u + d_const) * DT;
        x(0) = x_new;
        x(1) = v_new;
    }
    return res;
}

// Sliding surface: s = v + lambda * x
auto surface = [](const VectorXd& x) -> double {
    return x(1) + LAMBDA * x(0);
};

// Equivalent control: u_eq = -lambda * v (sets s_dot = 0 for d=0)
auto eq_control = [](const VectorXd& x) -> double {
    return -LAMBDA * x(1);
};

// Test 1: Double integrator regulation with constant disturbance (Sign mode)
void test_sign_convergence() {
    std::cout << "Test 1: Sign mode convergence with disturbance... ";

    SMCParams params{};
    params.K = 5.0;     // disturbance is 2.0, so K > d_max
    params.phi = 0.0;

    SlidingModeController ctrl(params, SMCMode::Sign, surface, eq_control);
    auto res = simulate(ctrl, 2.0, 1.0, 0.5);

    // After 5 seconds with lambda=2, state should be near origin
    double final_pos = std::abs(res.position.back());
    double final_vel = std::abs(res.velocity.back());
    assert(final_pos < 0.05 && "Position should converge near zero");
    assert(final_vel < 0.05 && "Velocity should converge near zero");

    std::cout << "PASSED (final pos=" << final_pos << ", vel=" << final_vel << ")\n";
}

// Test 2: Boundary layer vs sign — compare chattering
void test_boundary_layer_chattering() {
    std::cout << "Test 2: Boundary layer reduces chattering... ";

    // Sign mode
    SMCParams sign_params{};
    sign_params.K = 5.0;
    sign_params.phi = 0.0;
    SlidingModeController sign_ctrl(sign_params, SMCMode::Sign, surface, eq_control);
    auto sign_res = simulate(sign_ctrl, 2.0, 1.0, 0.5);

    // Boundary layer mode
    SMCParams bl_params{};
    bl_params.K = 5.0;
    bl_params.phi = 0.5;
    SlidingModeController bl_ctrl(bl_params, SMCMode::BoundaryLayer, surface, eq_control);
    auto bl_res = simulate(bl_ctrl, 2.0, 1.0, 0.5);

    // Count sign changes in control signal (measure of chattering)
    int sign_changes_sign = 0;
    int sign_changes_bl = 0;
    for (size_t i = 1; i < sign_res.control.size(); ++i) {
        if (sign_res.control[i] * sign_res.control[i-1] < 0) sign_changes_sign++;
        if (bl_res.control[i] * bl_res.control[i-1] < 0) sign_changes_bl++;
    }

    assert(sign_changes_bl < sign_changes_sign &&
           "Boundary layer should have fewer sign changes than pure sign");

    std::cout << "PASSED (sign changes: sign=" << sign_changes_sign
              << ", boundary=" << sign_changes_bl << ")\n";
}

// Test 3: Super-twisting — continuous control and convergence
void test_super_twisting() {
    std::cout << "Test 3: Super-twisting convergence and continuity... ";

    SMCParams params{};
    params.K = 0.0;     // not used in super-twisting
    params.phi = 0.0;
    params.k1 = 10.0;
    params.k2 = 15.0;

    SlidingModeController ctrl(params, SMCMode::SuperTwisting, surface, eq_control);
    auto res = simulate(ctrl, 2.0, 1.0, 0.5);

    // Check convergence
    double final_pos = std::abs(res.position.back());
    double final_vel = std::abs(res.velocity.back());
    assert(final_pos < 0.1 && "Super-twisting should converge");
    assert(final_vel < 0.1 && "Super-twisting velocity should converge");

    // Check control continuity: max |u[i] - u[i-1]| should be small
    double max_jump = 0.0;
    for (size_t i = 1; i < res.control.size(); ++i) {
        double jump = std::abs(res.control[i] - res.control[i-1]);
        if (jump > max_jump) max_jump = jump;
    }
    // For sign mode with K=5, jumps would be ~10. Super-twisting should be much less.
    assert(max_jump < 2.0 && "Super-twisting control should be approximately continuous");

    std::cout << "PASSED (final pos=" << final_pos << ", max_jump=" << max_jump << ")\n";
}

// Test 4: Robustness — vary disturbance up to K, verify invariance on surface
void test_robustness() {
    std::cout << "Test 4: Robustness to disturbances within bound... ";

    SMCParams params{};
    params.K = 10.0;    // Use larger K to ensure robust margin across all test disturbances
    params.phi = 0.0;

    // Test with multiple disturbance levels well below K
    double disturbances[] = {0.0, 1.0, 2.0, 3.0, 5.0, 8.0};
    for (double d : disturbances) {
        SlidingModeController ctrl(params, SMCMode::Sign, surface, eq_control);
        auto res = simulate(ctrl, d, 1.0, 0.5);

        double final_pos = std::abs(res.position.back());
        assert(final_pos < 0.15 &&
               "System should converge for any disturbance < K");
    }

    std::cout << "PASSED (converges for d in {0, 1, 2, 3, 5, 8} with K=10)\n";
}

// Test 5: Failure — disturbance exceeds K, system does not reach sliding surface
void test_failure_exceeds_bound() {
    std::cout << "Test 5: Failure when disturbance exceeds K... ";

    SMCParams params{};
    params.K = 5.0;
    params.phi = 0.0;

    // Disturbance = 10 >> K = 5
    SlidingModeController ctrl(params, SMCMode::Sign, surface, eq_control);
    auto res = simulate(ctrl, 10.0, 0.0, 0.0);

    // State should drift away from origin (disturbance dominates)
    double final_pos = std::abs(res.position.back());
    assert(final_pos > 1.0 &&
           "System should diverge when disturbance exceeds switching gain");

    std::cout << "PASSED (final pos=" << final_pos << " — diverged as expected)\n";
}

int main() {
    std::cout << "=== Sliding Mode Controller Tests ===\n";
    test_sign_convergence();
    test_boundary_layer_chattering();
    test_super_twisting();
    test_robustness();
    test_failure_exceeds_bound();
    std::cout << "=== All SMC tests passed ===\n";
    return 0;
}
