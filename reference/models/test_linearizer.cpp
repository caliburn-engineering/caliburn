#include "linearizer.h"

#include <cassert>
#include <cmath>
#include <cstdio>

using namespace caliburn;

static constexpr double TOL = 1e-8;
static constexpr double G = 9.81;
static constexpr double K = 5.0 / 7.0;

// ---- Test 1: Mass-spring-damper (already linear — should recover exact A, B) ----
//
// mx'' + bx' + kx = u
// State: [x, v], Input: [u]
// A = [0, 1; -k/m, -b/m], B = [0; 1/m]
static void test_mass_spring_damper() {
    double m = 2.0, b = 0.5, k = 10.0;

    NonlinearFn f = [=](const Eigen::VectorXd& x, const Eigen::VectorXd& u) {
        Eigen::VectorXd dx(2);
        dx(0) = x(1);
        dx(1) = (-k * x(0) - b * x(1) + u(0)) / m;
        return dx;
    };

    Eigen::VectorXd x0 = Eigen::VectorXd::Zero(2);
    Eigen::VectorXd u0 = Eigen::VectorXd::Zero(1);

    auto sys = linearize(f, x0, u0);

    assert(std::abs(sys.A(0, 0) - 0.0) < TOL);
    assert(std::abs(sys.A(0, 1) - 1.0) < TOL);
    assert(std::abs(sys.A(1, 0) - (-k / m)) < TOL);
    assert(std::abs(sys.A(1, 1) - (-b / m)) < TOL);
    assert(std::abs(sys.B(0, 0) - 0.0) < TOL);
    assert(std::abs(sys.B(1, 0) - (1.0 / m)) < TOL);

    // C should be identity, D should be zero
    assert(sys.C.rows() == 2 && sys.C.cols() == 2);
    assert(sys.D.rows() == 2 && sys.D.cols() == 1);
    assert((sys.C - Eigen::MatrixXd::Identity(2, 2)).norm() < TOL);
    assert(sys.D.norm() < TOL);

    std::printf("  [PASS] Mass-spring-damper — exact A, B recovered\n");
}

// ---- Test 2: Ball-balancer analytical vs numerical ----
static void test_ball_balancer_linearization() {
    // Nonlinear ball-on-plate: f(x, u) with sin(alpha), sin(beta)
    // No friction for clean comparison
    NonlinearFn f = [](const Eigen::VectorXd& x, const Eigen::VectorXd& u) {
        Eigen::VectorXd dx(4);
        dx(0) = x(2);  // vx
        dx(1) = x(3);  // vy
        dx(2) = K * G * std::sin(u(1));  // ax from beta (Y-axis tilt)
        dx(3) = K * G * std::sin(u(0));  // ay from alpha (X-axis tilt)
        return dx;
    };

    // Hand-derived analytical model
    LinearSystem analytical;
    analytical.A = Eigen::MatrixXd::Zero(4, 4);
    analytical.A(0, 2) = 1.0;
    analytical.A(1, 3) = 1.0;

    analytical.B = Eigen::MatrixXd::Zero(4, 2);
    analytical.B(2, 1) = K * G;  // d(ax)/d(beta)
    analytical.B(3, 0) = K * G;  // d(ay)/d(alpha)

    analytical.C = Eigen::MatrixXd::Identity(4, 4);
    analytical.D = Eigen::MatrixXd::Zero(4, 2);

    Eigen::VectorXd x0 = Eigen::VectorXd::Zero(4);
    Eigen::VectorXd u0 = Eigen::VectorXd::Zero(2);

    auto result = validate(analytical, f, x0, u0, 1e-6);

    assert(result.pass);
    assert(result.max_A_error < 1e-8);
    assert(result.max_B_error < 1e-6);  // sin linearization via central diff

    std::printf("  [PASS] Ball-balancer analytical vs numerical (max_A=%.2e, max_B=%.2e)\n",
                result.max_A_error, result.max_B_error);
}

// ---- Test 3: Custom C and D ----
static void test_custom_output_matrices() {
    // Simple 2-state system, measure only first state
    NonlinearFn f = [](const Eigen::VectorXd& x, const Eigen::VectorXd& u) {
        Eigen::VectorXd dx(2);
        dx(0) = x(1);
        dx(1) = -x(0) + u(0);
        return dx;
    };

    Eigen::VectorXd x0 = Eigen::VectorXd::Zero(2);
    Eigen::VectorXd u0 = Eigen::VectorXd::Zero(1);

    Eigen::MatrixXd C(1, 2);
    C << 1.0, 0.0;
    Eigen::MatrixXd D = Eigen::MatrixXd::Zero(1, 1);

    auto sys = linearize(f, x0, u0, C, D);

    assert(sys.C.rows() == 1 && sys.C.cols() == 2);
    assert(std::abs(sys.C(0, 0) - 1.0) < TOL);
    assert(std::abs(sys.C(0, 1) - 0.0) < TOL);
    assert(sys.D.rows() == 1 && sys.D.cols() == 1);
    assert(sys.D.norm() < TOL);

    std::printf("  [PASS] Custom C and D passed through correctly\n");
}

// ---- Test 4: Nonlinear system — linearization at non-zero operating point ----
static void test_nonzero_operating_point() {
    // f(x, u) = [-x^2 + u], linearize at x0=2, u0=4 (equilibrium: -4+4=0)
    // A = df/dx = -2*x0 = -4, B = df/du = 1
    NonlinearFn f = [](const Eigen::VectorXd& x, const Eigen::VectorXd& u) {
        Eigen::VectorXd dx(1);
        dx(0) = -x(0) * x(0) + u(0);
        return dx;
    };

    Eigen::VectorXd x0(1);
    x0 << 2.0;
    Eigen::VectorXd u0(1);
    u0 << 4.0;

    auto sys = linearize(f, x0, u0);

    assert(std::abs(sys.A(0, 0) - (-4.0)) < 1e-6);
    assert(std::abs(sys.B(0, 0) - 1.0) < TOL);

    std::printf("  [PASS] Nonzero operating point (A=%.4f, B=%.4f)\n",
                sys.A(0, 0), sys.B(0, 0));
}

// ---- Test 5: Zero state/input — no division by zero ----
static void test_zero_operating_point() {
    NonlinearFn f = [](const Eigen::VectorXd& x, const Eigen::VectorXd& u) {
        Eigen::VectorXd dx(2);
        dx(0) = x(1) + u(0);
        dx(1) = -x(0);
        return dx;
    };

    Eigen::VectorXd x0 = Eigen::VectorXd::Zero(2);
    Eigen::VectorXd u0 = Eigen::VectorXd::Zero(1);

    auto sys = linearize(f, x0, u0);

    assert(std::abs(sys.A(0, 1) - 1.0) < TOL);
    assert(std::abs(sys.A(1, 0) - (-1.0)) < TOL);
    assert(std::abs(sys.B(0, 0) - 1.0) < TOL);

    std::printf("  [PASS] Zero operating point — no NaN or division by zero\n");
}

int main() {
    std::printf("Linearizer tests:\n");

    test_mass_spring_damper();
    test_ball_balancer_linearization();
    test_custom_output_matrices();
    test_nonzero_operating_point();
    test_zero_operating_point();

    std::printf("All linearizer tests passed.\n");
    return 0;
}
