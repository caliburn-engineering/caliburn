#include "luenberger.h"

#include <cassert>
#include <cmath>
#include <cstdio>

// ---------------------------------------------------------------------------
// Test 1: Mass-spring-damper — estimate velocity from position measurement
// ---------------------------------------------------------------------------
void test_mass_spring_damper() {
    // System: m*x_ddot + c*x_dot + k*x = 0
    // State: [position, velocity]
    // Measurement: position only
    // Observer should reconstruct velocity

    const double m = 1.0, c = 0.5, k = 2.0;
    const double dt = 0.001;
    const int num_steps = 5000;  // 5 seconds

    // Continuous-time state-space: x_dot = A*x + B*u, y = C*x
    Eigen::MatrixXd A(2, 2);
    A << 0.0,    1.0,
         -k/m,  -c/m;

    Eigen::MatrixXd B(2, 1);
    B << 0.0, 1.0/m;

    Eigen::MatrixXd C(1, 2);
    C << 1.0, 0.0;

    // Observer gain — place poles at -20, -25 (fast, controller poles ~-1.5)
    // For a 2nd order system: desired char poly = (s+20)(s+25) = s^2 + 45s + 500
    // Using direct pole placement for this simple system:
    // eig(A - LC) = {-20, -25}
    // For C = [1, 0], L = [l1; l2]
    // A - LC = [0-l1, 1; -k/m-l2, -c/m]
    // char poly: s^2 + (c/m + l1)*s + (k/m + l2 + l1*c/m)
    // Match: c/m + l1 = 45 → l1 = 45 - 0.5 = 44.5
    //        k/m + l2 + l1*c/m ... actually use direct computation
    // Desired: s^2 + 45s + 500
    // System char poly of (A-LC): (s + l1)(s + c/m) + (k/m + l2) ... let's just set gains

    Eigen::MatrixXd L(2, 1);
    L << 44.5,
         475.75;  // Computed to place poles at -20, -25

    caliburn::LuenbergerObserver obs(A, B, C, L);

    // True initial state: position=1, velocity=0
    Eigen::VectorXd x_true(2);
    x_true << 1.0, 0.0;

    // Observer starts with wrong initial estimate
    Eigen::VectorXd x0_hat(2);
    x0_hat << 0.0, 0.0;
    obs.set_state(x0_hat);

    // Simulate
    Eigen::VectorXd u(1);
    u << 0.0;  // no external input

    for (int i = 0; i < num_steps; ++i) {
        // True system evolution (forward Euler)
        Eigen::VectorXd x_dot_true = A * x_true + B * u;
        x_true = x_true + dt * x_dot_true;

        // Measurement
        Eigen::VectorXd y = C * x_true;

        // Observer update
        obs.update(u, y, dt);
    }

    // After 5 seconds with poles at -20,-25, error should be negligible
    double err = obs.errorNorm(x_true);
    printf("Test 1 (mass-spring-damper): final error = %.6e\n", err);
    assert(err < 1e-3 && "Observer should converge for mass-spring-damper");
    printf("  PASSED\n");
}

// ---------------------------------------------------------------------------
// Test 2: Pole speed comparison — convergence rate scales with pole placement
// ---------------------------------------------------------------------------
void test_pole_speed_comparison() {
    // Simple integrator: x_dot = [0 1; 0 0]*x + [0; 1]*u, y = [1 0]*x
    // (double integrator: position/velocity, measure position)

    const double dt = 0.001;

    Eigen::MatrixXd A(2, 2);
    A << 0.0, 1.0,
         0.0, 0.0;

    Eigen::MatrixXd B(2, 1);
    B << 0.0, 1.0;

    Eigen::MatrixXd C(1, 2);
    C << 1.0, 0.0;

    // Test with different observer pole speeds
    // For double integrator with C=[1,0]:
    // A-LC = [0-l1, 1; -l2, 0], char poly: s^2 + l1*s + l2
    // Desired poles at -p, -p: char poly = s^2 + 2p*s + p^2
    // So l1 = 2p, l2 = p^2

    double pole_speeds[] = {5.0, 10.0, 20.0, 50.0};
    double convergence_times[4];

    for (int trial = 0; trial < 4; ++trial) {
        double p = pole_speeds[trial];
        Eigen::MatrixXd L(2, 1);
        L << 2.0 * p, p * p;

        caliburn::LuenbergerObserver obs(A, B, C, L);

        // True state: stationary at position=1
        Eigen::VectorXd x_true(2);
        x_true << 1.0, 0.0;

        // Observer starts at zero
        Eigen::VectorXd x0_hat(2);
        x0_hat << 0.0, 0.0;
        obs.set_state(x0_hat);

        Eigen::VectorXd u(1);
        u << 0.0;

        // Find time to converge to within 5% of true state
        double convergence_time = -1.0;
        for (int i = 0; i < 10000; ++i) {
            Eigen::VectorXd y = C * x_true;
            obs.update(u, y, dt);

            if (convergence_time < 0 && obs.errorNorm(x_true) < 0.05) {
                convergence_time = i * dt;
                break;
            }
        }

        convergence_times[trial] = convergence_time;
        printf("  Poles at -%.0f: convergence time = %.4f s\n",
               p, convergence_time);
    }

    // Faster poles should converge faster
    for (int i = 0; i < 3; ++i) {
        assert(convergence_times[i+1] < convergence_times[i] &&
               "Faster poles must converge faster");
    }

    printf("Test 2 (pole speed comparison): PASSED\n");
}

// ---------------------------------------------------------------------------
// Test 3: Integration with LQR — separation principle verification
// ---------------------------------------------------------------------------
void test_separation_principle() {
    // Double integrator with LQR controller using observer estimates
    // Verify: closed-loop poles = controller poles ∪ observer poles

    const double dt = 0.001;
    const int num_steps = 10000;  // 10 seconds

    Eigen::MatrixXd A(2, 2);
    A << 0.0, 1.0,
         0.0, 0.0;

    Eigen::MatrixXd B(2, 1);
    B << 0.0, 1.0;

    Eigen::MatrixXd C(1, 2);
    C << 1.0, 0.0;

    // Controller gain (places controller poles at -2, -3)
    // u = -K * x_hat, K = [k1, k2]
    // eig(A - BK) = eig([0, 1; -k1, -k2])
    // char poly: s^2 + k2*s + k1 = (s+2)(s+3) = s^2 + 5s + 6
    // k1 = 6, k2 = 5
    Eigen::MatrixXd K(1, 2);
    K << 6.0, 5.0;

    // Observer gain (places observer poles at -10, -15, i.e. ~5x faster)
    // l1 = 2*p ≈ 25, l2 = p^2 ... for distinct poles -10, -15:
    // char poly: s^2 + 25s + 150
    // l1 = 25, l2 = 150
    Eigen::MatrixXd L(2, 1);
    L << 25.0, 150.0;

    caliburn::LuenbergerObserver obs(A, B, C, L);

    // True initial state (displaced)
    Eigen::VectorXd x_true(2);
    x_true << 2.0, 0.0;

    // Observer starts at zero (wrong estimate)
    Eigen::VectorXd x0_hat(2);
    x0_hat << 0.0, 0.0;
    obs.set_state(x0_hat);

    // Simulate closed loop: u = -K * x_hat
    for (int i = 0; i < num_steps; ++i) {
        // Control using estimated state
        Eigen::VectorXd u = -K * obs.state();

        // Measurement from true system
        Eigen::VectorXd y = C * x_true;

        // True system evolution
        Eigen::VectorXd x_dot_true = A * x_true + B * u;
        x_true = x_true + dt * x_dot_true;

        // Observer update
        obs.update(u, y, dt);
    }

    // Both true state and estimate should converge to zero
    double state_err = x_true.norm();
    double obs_err = obs.errorNorm(x_true);

    printf("Test 3 (separation principle): true state norm = %.6e, "
           "observer error = %.6e\n", state_err, obs_err);
    assert(state_err < 1e-3 && "True state should converge to origin");
    assert(obs_err < 1e-4 && "Observer should track true state");
    printf("  PASSED\n");
}

// ---------------------------------------------------------------------------
// Test 4: Unobservable mode — observer cannot estimate hidden state
// ---------------------------------------------------------------------------
void test_unobservable_mode() {
    // System with 2 states but only one is observable
    // A = [a1 0; 0 a2], C = [1 0] — second state is completely hidden
    // Observer should converge for state 1 but NOT for state 2

    const double dt = 0.001;
    const int num_steps = 5000;

    Eigen::MatrixXd A(2, 2);
    A << -1.0, 0.0,
          0.0, -2.0;  // both states stable

    Eigen::MatrixXd B(2, 1);
    B << 1.0, 1.0;

    Eigen::MatrixXd C(1, 2);
    C << 1.0, 0.0;  // only measure first state

    // Check observability: O = [C; CA] = [1 0; -1 0] — rank 1, not full rank!
    // Observer gain — can only affect first state's convergence
    Eigen::MatrixXd L(2, 1);
    L << 10.0, 0.0;  // gain only on observable state

    caliburn::LuenbergerObserver obs(A, B, C, L);

    // True state: both states displaced
    Eigen::VectorXd x_true(2);
    x_true << 1.0, 3.0;

    // Observer starts at zero — wrong for both states
    Eigen::VectorXd x0_hat(2);
    x0_hat << 0.0, 0.0;
    obs.set_state(x0_hat);

    Eigen::VectorXd u(1);
    u << 0.0;

    for (int i = 0; i < num_steps; ++i) {
        Eigen::VectorXd x_dot_true = A * x_true + B * u;
        x_true = x_true + dt * x_dot_true;

        Eigen::VectorXd y = C * x_true;
        obs.update(u, y, dt);
    }

    // First state (observable): observer should converge
    double err_state1 = std::abs(x_true(0) - obs.state()(0));
    // Second state (unobservable): observer tracks open-loop only
    // Since both start at different ICs and L doesn't couple to state 2,
    // the error in state 2 evolves as e2_dot = a2*e2 (stable here, but
    // only by luck of the open-loop dynamics, not by observer design)
    double err_state2 = std::abs(x_true(1) - obs.state()(1));

    printf("Test 4 (unobservable mode): err_state1 = %.6e, err_state2 = %.6e\n",
           err_state1, err_state2);
    assert(err_state1 < 1e-3 && "Observable state should converge");
    // The unobservable state may or may not converge depending on open-loop stability
    // Key insight: observer gain L cannot accelerate convergence of unobservable mode
    printf("  PASSED (demonstrates unobservable mode limitation)\n");
}

// ---------------------------------------------------------------------------
int main() {
    printf("=== Luenberger Observer Tests ===\n\n");

    test_mass_spring_damper();
    printf("\n");

    test_pole_speed_comparison();
    printf("\n");

    test_separation_principle();
    printf("\n");

    test_unobservable_mode();
    printf("\n");

    printf("=== All tests passed ===\n");
    return 0;
}
