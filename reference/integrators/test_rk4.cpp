#include "rk4.h"

#include <cassert>
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

    assert(error < 1e-8);
    std::cout << "  exponential decay   error = " << error << "  PASS\n";
}

// ---------------------------------------------------------------------------
// Test 2: Harmonic oscillator  x'' = -x  as first-order system
//         State: [x, v],  derivatives: [v, -x]
//         IC: x(0) = 1, v(0) = 0   =>   x(2*pi) = 1, v(2*pi) = 0
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
    const double x_error = std::abs(y(0) - 1.0);
    const double v_error = std::abs(y(1) - 0.0);

    assert(x_error < 1e-6);
    assert(v_error < 1e-6);
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

    assert(ratio > 14.0 && ratio < 18.0);
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
