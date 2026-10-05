#include "inverted_pendulum.h"
#include "../controllers/lqr.h"
#include "../integrators/rk4.h"
#include "../test/assert_rel.h"
#include <cmath>
#include <iostream>
#include <complex>

// ---------------------------------------------------------------------------
// Test 1: Open-loop unstable (at least one eigenvalue with positive real part)
// ---------------------------------------------------------------------------
void test_open_loop_unstable() {
    caliburn::InvertedPendulumParams p;
    auto model = caliburn::build_inverted_pendulum(p);

    Eigen::EigenSolver<Eigen::Matrix4d> es(model.A);

    bool has_unstable = false;
    for (int i = 0; i < 4; ++i) {
        if (es.eigenvalues()(i).real() > 0.0) {
            has_unstable = true;
        }
    }
    ASSERT_CHECK(has_unstable, "inverted pendulum should be open-loop unstable");

    std::cout << "  [PASS] Test 1: Open-loop unstable (has positive eigenvalue)\n";
    std::cout << "         Eigenvalues: ";
    for (int i = 0; i < 4; ++i) {
        auto ev = es.eigenvalues()(i);
        std::cout << ev.real();
        if (std::abs(ev.imag()) > 1e-10)
            std::cout << "+" << ev.imag() << "j";
        std::cout << "  ";
    }
    std::cout << "\n";
}

// ---------------------------------------------------------------------------
// Test 2: A matrix structure — gravity enters at correct positions
// ---------------------------------------------------------------------------
void test_matrix_structure() {
    caliburn::InvertedPendulumParams p;
    p.M = 2.0; p.m = 0.5; p.L = 1.0; p.g = 9.81;

    auto model = caliburn::build_inverted_pendulum(p);

    // 1e-10: A and B entries are 2-3 floating-point operations on O(10) values
    // including g=9.81 (not exactly representable); error bounded by a few ULPs of
    // result (< 1e-13 for these magnitudes), well within 1e-10.
    double tol = 1e-10;

    // Trivial rows
    ASSERT_CHECK(model.A(0, 1) == 1.0, "A(0,1) must be 1 (kinematic row)");
    ASSERT_CHECK(model.A(2, 3) == 1.0, "A(2,3) must be 1 (kinematic row)");

    // A(1,2): -m*g/M = -0.5*9.81/2 = -2.4525 — magnitude > 1, so ASSERT_REL_NEAR would be looser.
    double expected_A12 = -p.m * p.g / p.M;
    ASSERT_CHECK(std::abs(model.A(1, 2) - expected_A12) < tol, "A(1,2) = -m*g/M");

    // A(3,2): (M+m)*g/(M*L) = 2.5*9.81/2 = 12.26 — magnitude >> 1, so ASSERT_REL_NEAR would be looser.
    double expected_A32 = (p.M + p.m) * p.g / (p.M * p.L);
    ASSERT_CHECK(std::abs(model.A(3, 2) - expected_A32) < tol, "A(3,2) = (M+m)*g/(M*L)");

    // B(1): 1/M = 0.5; B(3): -1/(M*L) = -0.5 — both < 1; scale = 1.
    ASSERT_REL_NEAR(model.B(1), 1.0 / p.M, tol);
    ASSERT_REL_NEAR(model.B(3), -1.0 / (p.M * p.L), tol);

    std::cout << "  [PASS] Test 2: A matrix structure verified\n";
}

// ---------------------------------------------------------------------------
// Test 3: LQR stabilises the inverted pendulum (closed-loop stable)
// ---------------------------------------------------------------------------
void test_lqr_stabilisation() {
    caliburn::InvertedPendulumParams p;
    auto model = caliburn::build_inverted_pendulum(p);

    // Continuous-time LQR
    Eigen::MatrixXd A = model.A;
    Eigen::MatrixXd B = model.B;

    Eigen::MatrixXd Q = Eigen::MatrixXd::Identity(4, 4);
    Q(2, 2) = 10.0;  // penalise angle heavily

    Eigen::MatrixXd R = Eigen::MatrixXd::Identity(1, 1);

    auto result = caliburn::lqr(A, B, Q, R);
    Eigen::MatrixXd K = result.K;

    // Check closed-loop eigenvalues: all should have negative real parts
    Eigen::MatrixXd A_cl = A - B * K;
    Eigen::EigenSolver<Eigen::MatrixXd> es(A_cl);

    for (int i = 0; i < 4; ++i) {
        double re = es.eigenvalues()(i).real();
        ASSERT_CHECK(re < 0.0, "LQR closed-loop eigenvalue has non-negative real part");
    }

    std::cout << "  [PASS] Test 3: LQR stabilises inverted pendulum (all closed-loop poles stable)\n";
}

// ---------------------------------------------------------------------------
// Test 4: LQR simulation — pendulum recovers from small perturbation
// ---------------------------------------------------------------------------
void test_lqr_simulation() {
    caliburn::InvertedPendulumParams p;
    auto model = caliburn::build_inverted_pendulum(p);

    Eigen::MatrixXd A = model.A;
    Eigen::MatrixXd B_mat = model.B;

    Eigen::MatrixXd Q = Eigen::MatrixXd::Identity(4, 4);
    Q(2, 2) = 10.0;

    Eigen::MatrixXd R = Eigen::MatrixXd::Identity(1, 1);

    auto result = caliburn::lqr(A, B_mat, Q, R);
    Eigen::MatrixXd K = result.K;

    // Simulate: initial perturbation theta = 0.1 rad (~5.7 degrees)
    auto deriv = [&](double /*t*/, const Eigen::VectorXd& x) -> Eigen::VectorXd {
        Eigen::VectorXd u = -K * x;
        return A * x + B_mat * u;
    };

    Eigen::VectorXd x0(4);
    x0 << 0.0, 0.0, 0.1, 0.0;  // small angle perturbation

    double dt = 0.001;
    int steps = 10000;  // 10 seconds

    Eigen::VectorXd x_final = caliburn::rk4_integrate(x0, 0.0, dt, steps, deriv);

    // 0.01: convergence bound with ~15x margin; measured |x_final| = 6.7e-4 (Release).
    ASSERT_CHECK(x_final.norm() < 0.01,
                 "LQR did not stabilise the pendulum from small perturbation");

    std::cout << "  [PASS] Test 4: LQR simulation recovers from theta=0.1 rad "
              << "(final state norm=" << x_final.norm() << ")\n";
}

// ---------------------------------------------------------------------------
int main() {
    test_open_loop_unstable();
    test_matrix_structure();
    test_lqr_stabilisation();
    test_lqr_simulation();

    std::cout << "\nAll inverted pendulum tests passed.\n";
    return 0;
}
