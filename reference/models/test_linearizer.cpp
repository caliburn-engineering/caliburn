#include "linearizer.h"

#include "../test/assert_rel.h"
#include <cmath>
#include <cstdio>

using namespace caliburn;

// 1e-8: central-difference Jacobian; truncation and round-off both ~1e-10 for these
// well-conditioned test cases; also bounds exact C, D construction from these tests.
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

    ASSERT_REL_NEAR(sys.A(0, 0), 0.0, TOL);    // expected 0; scale = 1
    ASSERT_REL_NEAR(sys.A(0, 1), 1.0, TOL);    // expected 1; scale = 1
    // A(1,0) = -k/m = -5.0 — magnitude > 1, so ASSERT_REL_NEAR would be looser.
    ASSERT_CHECK(std::abs(sys.A(1, 0) - (-k / m)) < TOL, "A(1,0) = -k/m");
    ASSERT_REL_NEAR(sys.A(1, 1), -b / m, TOL); // expected -0.25; scale = 1
    ASSERT_REL_NEAR(sys.B(0, 0), 0.0, TOL);    // expected 0; scale = 1
    ASSERT_REL_NEAR(sys.B(1, 0), 1.0 / m, TOL); // expected 0.5; scale = 1

    // C should be identity, D should be zero
    ASSERT_CHECK(sys.C.rows() == 2 && sys.C.cols() == 2, "C must be 2x2");
    ASSERT_CHECK(sys.D.rows() == 2 && sys.D.cols() == 1, "D must be 2x1");
    // ||I_2||_F = sqrt(2) > 1; ASSERT_MATRIX_REL_NEAR would be looser than 1e-8 absolute.
    ASSERT_CHECK((sys.C - Eigen::MatrixXd::Identity(2, 2)).norm() < TOL, "C should equal identity");
    ASSERT_CHECK(sys.D.norm() < TOL, "D should be zero");

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

    ASSERT_CHECK(result.pass, "ball-balancer validation must pass");
    // A: f is linear in the state and x0 = 0, so x0 ± h are exactly ±h, f(x0 ± h) are exactly
    // ±h (or 0 for rows that depend only on u), and (f+ - f-)/(2h) is exactly 1 or 0. The
    // central difference is exact, so the error must be exactly 0.
    ASSERT_CHECK(result.max_A_error == 0.0, "max A error must be exactly 0 (linear f at x0 = 0: central diff exact)");
    // B: sin linearisation at u0=0 via central diff with epsilon=1e-6. Truncation dominates:
    // K*G * (1 - sin(h)/h) = K*G*h^2/6 = 0.714*9.81*(1e-6)^2/6 = 1.17e-12; measured 1.17e-12.
    // Use 1e-11 (10x margin).
    ASSERT_CHECK(result.max_B_error < 1e-11, "max B error must be < 1e-11 (sin via central diff, K*G*h^2/6)");

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

    ASSERT_CHECK(sys.C.rows() == 1 && sys.C.cols() == 2, "C must be 1x2");
    ASSERT_REL_NEAR(sys.C(0, 0), 1.0, TOL);  // expected 1; scale = 1
    ASSERT_REL_NEAR(sys.C(0, 1), 0.0, TOL);  // expected 0; scale = 1
    ASSERT_CHECK(sys.D.rows() == 1 && sys.D.cols() == 1, "D must be 1x1");
    ASSERT_CHECK(sys.D.norm() < TOL, "D should be zero");

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

    // A(0,0) = -4.0 — magnitude > 1, so ASSERT_REL_NEAR would be looser.
    // Central diff of -x² has no truncation error (third derivative is 0), so the error is
    // pure round-off. h = 1e-6, u = 2^-53 ≈ 1.1e-16. Rounding x0 ± h to a double moves each
    // point by ≤ u·2, which moves f by ≤ 4·2.2e-16 = 8.8e-16; each square (≈ 4) rounds by
    // ≤ u·4 = 4.4e-16; the subtraction from u0 = 4 is exact (Sterbenz). Numerator error
    // ≤ 2·(8.8 + 4.4)e-16 ≈ 2.6e-15, divided by 2h: ≤ 1.3e-9. 1e-8 is ~7.5× over.
    ASSERT_CHECK(std::abs(sys.A(0, 0) - (-4.0)) < 1e-8, "A(0,0) = df/dx = -2*x0 = -4");
    ASSERT_REL_NEAR(sys.B(0, 0), 1.0, TOL);  // expected 1; scale = 1

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

    ASSERT_REL_NEAR(sys.A(0, 1), 1.0, TOL);   // expected 1; scale = 1
    ASSERT_REL_NEAR(sys.A(1, 0), -1.0, TOL);  // expected -1; scale = 1
    ASSERT_REL_NEAR(sys.B(0, 0), 1.0, TOL);   // expected 1; scale = 1

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
