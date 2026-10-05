#include "rk4.h"

#include "assert_rel.h"
#include <cmath>
#include <iostream>

// ---------------------------------------------------------------------------
// Test 1: Exponential decay  y' = -y,  y(0) = 1
//         Exact solution: y(t) = e^{-t}
// ---------------------------------------------------------------------------
void test_exponential_decay() {
    caliburn::DerivativeFn f = [](double /*t*/, const Eigen::VectorXd& y) {
        return -y;
    };

    Eigen::VectorXd y0(1);
    y0 << 1.0;

    const double h = 0.01;
    const int steps = 100;  // t_final = 1.0

    Eigen::VectorXd y = caliburn::rk4_integrate(y0, 0.0, h, steps, f);
    const double error = std::abs(y(0) - std::exp(-1.0));

    // 1e-8: RK4 global error O(h^4·T) with h=0.01, T=1; leading error constant ~1/30 → ~3e-9
    ASSERT_CHECK(error < 1e-8, "exponential decay: RK4 global error should be < 1e-8 at t=1 with h=0.01");
    std::cout << "  exponential decay   error = " << error << "  PASS\n";
}

// ---------------------------------------------------------------------------
// Test 2: Harmonic oscillator  x'' = -x  as first-order system
//         State: [x, v],  derivatives: [v, -x]
//         IC: x(0) = 1, v(0) = 0   =>   x(t) = cos(t), v(t) = -sin(t)
// ---------------------------------------------------------------------------
void test_harmonic_oscillator() {
    caliburn::DerivativeFn f = [](double /*t*/, const Eigen::VectorXd& y) {
        Eigen::VectorXd dy(2);
        dy(0) = y(1);     // dx/dt = v
        dy(1) = -y(0);    // dv/dt = -x
        return dy;
    };

    Eigen::VectorXd y0(2);
    y0 << 1.0, 0.0;

    const double period = 2.0 * M_PI;
    const double h = 0.001;
    const int steps = static_cast<int>(std::round(period / h));

    Eigen::VectorXd y = caliburn::rk4_integrate(y0, 0.0, h, steps, f);
    // Compare at the time actually reached: steps * h = 6.283 stops 1.85e-4 short
    // of 2*pi, so v(t_end) = -sin(t_end) ~ 1.85e-4, not 0. RK4 global error is
    // O(h^4 * t_end) ~ 1e-11 here, far inside the 1e-6 tolerance.
    const double t_end = steps * h;
    const double x_error = std::abs(y(0) - std::cos(t_end));
    const double v_error = std::abs(y(1) + std::sin(t_end));

    // TODO(#75): tolerance unjustified — actual error ~5e-14 in Release; 1e-6 is ~1e8× loose, order-lowering mutations give ~5e-7 and still pass
    ASSERT_CHECK(x_error < 1e-6, "harmonic oscillator: x error should be < 1e-6 at t_end");
    // TODO(#75): tolerance unjustified — actual error ~5e-14 in Release; 1e-6 is ~1e8× loose, order-lowering mutations give ~5e-7 and still pass
    ASSERT_CHECK(v_error < 1e-6, "harmonic oscillator: v error should be < 1e-6 at t_end");
    std::cout << "  harmonic oscillator x_err = " << x_error
              << "  v_err = " << v_error << "  PASS\n";
}

// ---------------------------------------------------------------------------
// Test 3: Order-of-accuracy check
//         Exponential decay with h = 0.1 vs h = 0.05
//         Error ratio should be ~2^4 = 16 (4th-order method)
// ---------------------------------------------------------------------------
void test_order_of_accuracy() {
    caliburn::DerivativeFn f = [](double /*t*/, const Eigen::VectorXd& y) {
        return -y;
    };

    Eigen::VectorXd y0(1);
    y0 << 1.0;

    const double exact = std::exp(-1.0);

    // Coarse: h = 0.1, 10 steps
    Eigen::VectorXd y_coarse = caliburn::rk4_integrate(y0, 0.0, 0.1, 10, f);
    const double err_coarse = std::abs(y_coarse(0) - exact);

    // Fine: h = 0.05, 20 steps
    Eigen::VectorXd y_fine = caliburn::rk4_integrate(y0, 0.0, 0.05, 20, f);
    const double err_fine = std::abs(y_fine(0) - exact);

    const double ratio = err_coarse / err_fine;

    // 14.0–18.0: 4th-order method predicts ratio = (0.1/0.05)^4 = 16; ±2 allows sub-leading-order terms
    ASSERT_CHECK(ratio > 14.0 && ratio < 18.0, "order check: error ratio h=0.1 vs h=0.05 should be near 16 for 4th-order");
    std::cout << "  order of accuracy   ratio = " << ratio
              << " (expect ~16)  PASS\n";
}

// ---------------------------------------------------------------------------
int main() {
    std::cout << "RK4 integrator tests:\n";

    test_exponential_decay();
    test_harmonic_oscillator();
    test_order_of_accuracy();

    std::cout << "All RK4 tests passed.\n";
    return 0;
}
