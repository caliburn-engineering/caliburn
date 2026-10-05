#include "lqr.h"
#include "assert_rel.h"
#include <cmath>
#include <iostream>
#include <complex>

// ---------------------------------------------------------------------------
// Test 1: Discrete-time double integrator stabilisation
// ---------------------------------------------------------------------------
void test_dlqr_double_integrator() {
    const double dt = 0.01;

    Eigen::MatrixXd A_d(2, 2);
    A_d << 1.0, dt,
           0.0, 1.0;

    Eigen::MatrixXd B_d(2, 1);
    B_d << dt * dt / 2.0,
           dt;

    Eigen::MatrixXd Q = Eigen::MatrixXd::Identity(2, 2);
    Eigen::MatrixXd R = Eigen::MatrixXd::Identity(1, 1);

    auto result = caliburn::dlqr(A_d, B_d, Q, R);
    Eigen::MatrixXd K = result.K;

    // Simulate: x = [1; 0], 1000 steps, u = -K*x
    Eigen::VectorXd x(2);
    x << 1.0, 0.0;

    for (int i = 0; i < 1000; ++i) {
        Eigen::VectorXd u = -K * x;
        x = A_d * x + B_d * u;
    }

    // 0.01: convergence bound with ~25x margin; measured |x| = 4.1e-4 after 1000 steps (Release).
    ASSERT_CHECK(x.norm() < 0.01, "dlqr: state did not converge to zero after 1000 steps");

    // Check closed-loop eigenvalues have magnitude < 1
    Eigen::MatrixXd A_cl = A_d - B_d * K;
    Eigen::EigenSolver<Eigen::MatrixXd> es(A_cl);
    for (int i = 0; i < es.eigenvalues().size(); ++i) {
        double mag = std::abs(es.eigenvalues()(i));
        ASSERT_CHECK(mag < 1.0, "dlqr: closed-loop eigenvalue magnitude must be < 1 for stability");
    }

    std::cout << "  [PASS] Test 1: Discrete-time double integrator stabilisation\n";
}

// ---------------------------------------------------------------------------
// Test 2: Continuous-time double integrator stabilisation
// ---------------------------------------------------------------------------
void test_lqr_double_integrator() {
    Eigen::MatrixXd A(2, 2);
    A << 0.0, 1.0,
         0.0, 0.0;

    Eigen::MatrixXd B(2, 1);
    B << 0.0,
         1.0;

    Eigen::MatrixXd Q = Eigen::MatrixXd::Identity(2, 2);
    Eigen::MatrixXd R = Eigen::MatrixXd::Identity(1, 1);

    auto result = caliburn::lqr(A, B, Q, R);
    Eigen::MatrixXd K = result.K;

    // Check closed-loop eigenvalues have negative real part
    Eigen::MatrixXd A_cl = A - B * K;
    Eigen::EigenSolver<Eigen::MatrixXd> es(A_cl);
    for (int i = 0; i < es.eigenvalues().size(); ++i) {
        double re = es.eigenvalues()(i).real();
        ASSERT_CHECK(re < 0.0, "lqr: closed-loop eigenvalue must have negative real part for stability");
    }

    // Simulate with Euler integration: 1000 steps, dt = 0.01
    const double dt = 0.01;
    Eigen::VectorXd x(2);
    x << 1.0, 0.0;

    for (int i = 0; i < 1000; ++i) {
        Eigen::VectorXd u = -K * x;
        Eigen::VectorXd x_dot = A * x + B * u;
        x = x + dt * x_dot;
    }

    // 0.01: convergence bound with ~25x margin; measured |x| = 3.9e-4 after 1000 Euler steps (Release).
    ASSERT_CHECK(x.norm() < 0.01, "lqr: state did not converge to zero after 1000 Euler steps");

    std::cout << "  [PASS] Test 2: Continuous-time double integrator stabilisation\n";
}

// ---------------------------------------------------------------------------
// Test 3: Verify continuous-time Riccati equation residual
// ---------------------------------------------------------------------------
void test_riccati_residual() {
    Eigen::MatrixXd A(2, 2);
    A << 0.0, 1.0,
         0.0, 0.0;

    Eigen::MatrixXd B(2, 1);
    B << 0.0,
         1.0;

    Eigen::MatrixXd Q = Eigen::MatrixXd::Identity(2, 2);
    Eigen::MatrixXd R = Eigen::MatrixXd::Identity(1, 1);

    auto result = caliburn::lqr(A, B, Q, R);
    Eigen::MatrixXd P = result.P;

    // Residual: A^T P + P A - P B R^{-1} B^T P + Q
    Eigen::MatrixXd Rinv_Bt = R.llt().solve(B.transpose());
    Eigen::MatrixXd residual = A.transpose() * P + P * A
                             - P * B * Rinv_Bt * P + Q;

    double residual_norm = residual.norm();
    // 4×4 Hamiltonian eigendecomposition: backward error per H-entry ≈ n²·u·‖H‖_F ≈ 1e-14;
    // P-entry error ε_P ≈ κ(V)·1e-14 ≈ 1e-13 (κ(V) ≈ O(10) for this well-conditioned system);
    // residual sensitivity per element ≈ ‖A‖+‖PBBᵀ‖ ≈ 3; ‖residual‖_F ≤ √4·3·1e-13 ≈ 6e-13;
    // 1e-11 adds ~20× safety.
    ASSERT_CHECK(residual_norm < 1e-11, "lqr: Riccati equation residual too large");

    std::cout << "  [PASS] Test 3: Riccati equation residual = "
              << residual_norm << "\n";
}

// ---------------------------------------------------------------------------
int main() {
    test_dlqr_double_integrator();
    test_lqr_double_integrator();
    test_riccati_residual();

    std::cout << "\nAll LQR tests passed.\n";
    return 0;
}
