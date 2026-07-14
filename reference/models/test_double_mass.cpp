#include "double_mass_spring_damper.h"
#include "../integrators/rk4.h"
#include <cassert>
#include <cmath>
#include <iostream>
#include <complex>

// ---------------------------------------------------------------------------
// Test 1: A matrix structure verification
// ---------------------------------------------------------------------------
void test_matrix_structure() {
    caliburn::DoubleMassSpringDamperParams p;
    p.m1 = 2.0; p.m2 = 3.0;
    p.k1 = 15.0; p.k2 = 20.0;
    p.c1 = 1.0; p.c2 = 2.0;

    auto model = caliburn::build_double_msd(p);

    // Row 0: trivial kinematic row [0, 1, 0, 0]
    assert(model.A(0, 0) == 0.0);
    assert(model.A(0, 1) == 1.0);
    assert(model.A(0, 2) == 0.0);
    assert(model.A(0, 3) == 0.0);

    // Row 2: trivial kinematic row [0, 0, 0, 1]
    assert(model.A(2, 0) == 0.0);
    assert(model.A(2, 1) == 0.0);
    assert(model.A(2, 2) == 0.0);
    assert(model.A(2, 3) == 1.0);

    // Row 1: -(k1+k2)/m1
    double tol = 1e-12;
    assert(std::abs(model.A(1, 0) - (-(p.k1 + p.k2) / p.m1)) < tol);
    assert(std::abs(model.A(1, 1) - (-(p.c1 + p.c2) / p.m1)) < tol);
    assert(std::abs(model.A(1, 2) - (p.k2 / p.m1)) < tol);
    assert(std::abs(model.A(1, 3) - (p.c2 / p.m1)) < tol);

    // Row 3: coupling with opposite signs (Newton's 3rd law)
    assert(std::abs(model.A(3, 0) - (p.k2 / p.m2)) < tol);
    assert(std::abs(model.A(3, 1) - (p.c2 / p.m2)) < tol);
    assert(std::abs(model.A(3, 2) - (-p.k2 / p.m2)) < tol);
    assert(std::abs(model.A(3, 3) - (-p.c2 / p.m2)) < tol);

    // B: only 1/m2 at last entry
    assert(model.B(0) == 0.0);
    assert(model.B(1) == 0.0);
    assert(model.B(2) == 0.0);
    assert(std::abs(model.B(3) - 1.0 / p.m2) < tol);

    std::cout << "  [PASS] Test 1: A matrix structure verification\n";
}

// ---------------------------------------------------------------------------
// Test 2: Eigenvalues have negative real parts (stable with damping)
// ---------------------------------------------------------------------------
void test_eigenvalues_stable() {
    caliburn::DoubleMassSpringDamperParams p;
    auto model = caliburn::build_double_msd(p);

    Eigen::EigenSolver<Eigen::Matrix4d> es(model.A);
    for (int i = 0; i < 4; ++i) {
        double re = es.eigenvalues()(i).real();
        assert(re < 0.0 && "eigenvalue has non-negative real part for damped system");
    }

    std::cout << "  [PASS] Test 2: All eigenvalues have negative real parts\n";
}

// ---------------------------------------------------------------------------
// Test 3: Analytical natural frequencies (undamped case)
// ---------------------------------------------------------------------------
void test_natural_frequencies() {
    // For equal masses and springs with no damping:
    // m1 = m2 = m, k1 = k2 = k, c1 = c2 = 0
    // Natural frequencies: omega_1 = sqrt(k/m), omega_2 = sqrt(3k/m)
    caliburn::DoubleMassSpringDamperParams p;
    p.m1 = 1.0; p.m2 = 1.0;
    p.k1 = 10.0; p.k2 = 10.0;
    p.c1 = 0.0; p.c2 = 0.0;

    auto model = caliburn::build_double_msd(p);

    Eigen::EigenSolver<Eigen::Matrix4d> es(model.A);

    // Collect imaginary parts of eigenvalues (natural frequencies)
    double freqs[4];
    for (int i = 0; i < 4; ++i) {
        freqs[i] = std::abs(es.eigenvalues()(i).imag());
    }

    // Sort frequencies
    std::sort(freqs, freqs + 4);

    // Expected: omega_1 = sqrt(k/m) = sqrt(10), omega_2 = sqrt(3k/m) = sqrt(30)
    // Each appears twice (conjugate pairs)
    double omega_1 = std::sqrt(10.0);
    double omega_2 = std::sqrt(30.0);
    double tol = 1e-8;

    assert(std::abs(freqs[0] - omega_1) < tol);
    assert(std::abs(freqs[1] - omega_1) < tol);
    assert(std::abs(freqs[2] - omega_2) < tol);
    assert(std::abs(freqs[3] - omega_2) < tol);

    std::cout << "  [PASS] Test 3: Natural frequencies match analytical (omega_1="
              << omega_1 << ", omega_2=" << omega_2 << ")\n";
}

// ---------------------------------------------------------------------------
// Test 4: Step response simulation with RK4 decays to static equilibrium
// ---------------------------------------------------------------------------
void test_step_response_rk4() {
    caliburn::DoubleMassSpringDamperParams p;
    p.m1 = 1.0; p.m2 = 1.0;
    p.k1 = 10.0; p.k2 = 10.0;
    p.c1 = 2.0; p.c2 = 2.0;

    auto model = caliburn::build_double_msd(p);
    const double F = 5.0;  // constant force on m2

    // Derivative function: x_dot = A*x + B*u
    auto deriv = [&](double /*t*/, const Eigen::VectorXd& x) -> Eigen::VectorXd {
        return model.A * x + model.B * F;
    };

    // Initial condition: all zero
    Eigen::VectorXd x0 = Eigen::VectorXd::Zero(4);
    double dt = 0.001;
    int steps = 50000;  // 50 seconds — long enough for damped system to settle

    Eigen::VectorXd x_final = caliburn::rk4_integrate(x0, 0.0, dt, steps, deriv);

    // Steady-state: A*x_ss + B*F = 0 => x_ss = -A^{-1} * B * F
    Eigen::Vector4d x_ss = -model.A.inverse() * model.B * F;

    double err = (x_final - x_ss).norm();
    assert(err < 0.01 && "step response did not converge to steady state");

    std::cout << "  [PASS] Test 4: Step response converges to analytical steady-state (err="
              << err << ")\n";
}

// ---------------------------------------------------------------------------
int main() {
    test_matrix_structure();
    test_eigenvalues_stable();
    test_natural_frequencies();
    test_step_response_rk4();

    std::cout << "\nAll double mass-spring-damper tests passed.\n";
    return 0;
}
