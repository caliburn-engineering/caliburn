#include "double_mass_spring_damper.h"
#include "../integrators/rk4.h"
#include "../test/assert_rel.h"
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
    ASSERT_CHECK(model.A(0, 0) == 0.0, "A(0,0) must be 0 (kinematic row)");
    ASSERT_CHECK(model.A(0, 1) == 1.0, "A(0,1) must be 1 (kinematic row)");
    ASSERT_CHECK(model.A(0, 2) == 0.0, "A(0,2) must be 0 (kinematic row)");
    ASSERT_CHECK(model.A(0, 3) == 0.0, "A(0,3) must be 0 (kinematic row)");

    // Row 2: trivial kinematic row [0, 0, 0, 1]
    ASSERT_CHECK(model.A(2, 0) == 0.0, "A(2,0) must be 0 (kinematic row)");
    ASSERT_CHECK(model.A(2, 1) == 0.0, "A(2,1) must be 0 (kinematic row)");
    ASSERT_CHECK(model.A(2, 2) == 0.0, "A(2,2) must be 0 (kinematic row)");
    ASSERT_CHECK(model.A(2, 3) == 1.0, "A(2,3) must be 1 (kinematic row)");

    // Row 1: -(k1+k2)/m1
    // 1e-12: A and B entries are simple ratios of exactly-representable integer inputs; IEEE arithmetic gives exact or near-exact results.
    double tol = 1e-12;
    // Magnitudes of A(1,0..2) are 17.5, 1.5, 10 — all > 1, so ASSERT_REL_NEAR would be looser.
    ASSERT_CHECK(std::abs(model.A(1, 0) - (-(p.k1 + p.k2) / p.m1)) < tol, "A(1,0) = -(k1+k2)/m1");
    ASSERT_CHECK(std::abs(model.A(1, 1) - (-(p.c1 + p.c2) / p.m1)) < tol, "A(1,1) = -(c1+c2)/m1");
    ASSERT_CHECK(std::abs(model.A(1, 2) - (p.k2 / p.m1)) < tol, "A(1,2) = k2/m1");
    ASSERT_REL_NEAR(model.A(1, 3), p.c2 / p.m1, tol);  // expected 1.0; scale = max(1, 1, 1) = 1

    // Row 3: coupling with opposite signs (Newton's 3rd law)
    // A(3,0) = 6.67 and A(3,2) = -6.67 — magnitudes > 1, so ASSERT_REL_NEAR would be looser.
    ASSERT_CHECK(std::abs(model.A(3, 0) - (p.k2 / p.m2)) < tol, "A(3,0) = k2/m2");
    ASSERT_REL_NEAR(model.A(3, 1), p.c2 / p.m2, tol);   // expected 0.67; scale = 1
    ASSERT_CHECK(std::abs(model.A(3, 2) - (-p.k2 / p.m2)) < tol, "A(3,2) = -k2/m2");
    ASSERT_REL_NEAR(model.A(3, 3), -p.c2 / p.m2, tol);  // expected -0.67; scale = 1

    // B: only 1/m2 at last entry
    ASSERT_CHECK(model.B(0) == 0.0, "B(0) must be 0");
    ASSERT_CHECK(model.B(1) == 0.0, "B(1) must be 0");
    ASSERT_CHECK(model.B(2) == 0.0, "B(2) must be 0");
    ASSERT_REL_NEAR(model.B(3), 1.0 / p.m2, tol);  // expected 0.33; scale = 1

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
        ASSERT_CHECK(re < 0.0, "eigenvalue has non-negative real part for damped system");
    }

    std::cout << "  [PASS] Test 2: All eigenvalues have negative real parts\n";
}

// ---------------------------------------------------------------------------
// Test 3: Analytical natural frequencies (undamped case)
// ---------------------------------------------------------------------------
void test_natural_frequencies() {
    // Wall -- k1 -- m1 -- k2 -- m2 (free end), no damping, m = 1, k = 10:
    //   K = [[k1+k2, -k2], [-k2, k2]] = [[20, -10], [-10, 10]], M = I
    //   det(K - w^2 I) = 0  =>  w^4 - 30 w^2 + 100 = 0  =>  w^2 = 15 -/+ sqrt(125)
    //   omega_1 = 1.9544 rad/s, omega_2 = 5.1167 rad/s
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

    // Each frequency appears twice (conjugate pairs).
    // tol 1e-8: a 4x4 nonsymmetric eigensolve with O(10) entries is accurate to
    // ~1e-14 for these well-separated, non-defective eigenvalues.
    double omega_1 = std::sqrt(15.0 - std::sqrt(125.0));
    double omega_2 = std::sqrt(15.0 + std::sqrt(125.0));
    double tol = 1e-8;

    // omega_1 ≈ 1.95 rad/s, omega_2 ≈ 5.12 rad/s — both > 1, so ASSERT_REL_NEAR would be looser.
    ASSERT_CHECK(std::abs(freqs[0] - omega_1) < tol, "first natural frequency must match omega_1");
    ASSERT_CHECK(std::abs(freqs[1] - omega_1) < tol, "conjugate pair of omega_1 must match");
    ASSERT_CHECK(std::abs(freqs[2] - omega_2) < tol, "second natural frequency must match omega_2");
    ASSERT_CHECK(std::abs(freqs[3] - omega_2) < tol, "conjugate pair of omega_2 must match");

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
    // Transient decay: A is diagonalisable, so ‖e^{At}‖ ≤ κ(V)·e^{σ t} with σ = -0.382 rad/s
    // (the softer mode, -0.382 ± 1.92j) and κ(V) = 6.0 for these parameters. From x0 = 0, with
    // |x_ss| = |[0.5, 0, 1, 0]| = 1.118: err ≤ 6.0·1.118·e^{-0.382·50} ≈ 3.4e-8. RK4 at
    // h·|λ|max ≈ 0.005 tracks e^{hλ} to O((hλ)^5) per step, negligible. 1e-7 is ~3× over the bound.
    ASSERT_CHECK(err < 1e-7, "step response did not converge to steady state");

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
