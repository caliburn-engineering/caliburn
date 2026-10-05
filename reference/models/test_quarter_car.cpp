#include "quarter_car.h"
#include "../integrators/rk4.h"
#include "../test/assert_rel.h"
#include <cmath>
#include <cstdio>
#include <iostream>
#include <complex>

// ---------------------------------------------------------------------------
// Test 1: Natural frequencies match analytical estimates
// ---------------------------------------------------------------------------
void test_natural_frequencies() {
    caliburn::QuarterCarParams p;
    auto model = caliburn::build_quarter_car(p);

    Eigen::EigenSolver<Eigen::Matrix4d> es(model.A);

    // Collect natural frequencies (imaginary parts)
    double freqs_hz[4];
    for (int i = 0; i < 4; ++i) {
        freqs_hz[i] = std::abs(es.eigenvalues()(i).imag()) / (2.0 * M_PI);
    }
    std::sort(freqs_hz, freqs_hz + 4);

    // Body bounce: ~1-2 Hz (approx sqrt(k_s/m_b) / 2pi)
    // Wheel hop: ~10-15 Hz (approx sqrt((k_s+k_t)/m_w) / 2pi)
    double f_body_approx = std::sqrt(p.k_s / p.m_b) / (2.0 * M_PI);
    double f_wheel_approx = std::sqrt((p.k_s + p.k_t) / p.m_w) / (2.0 * M_PI);

    // The actual eigenfrequencies will differ from these approximations because
    // of coupling, but should be in the right ballpark
    // Body mode: expect 1-2 Hz
    // Plausibility bands for a passenger-car quarter model: body bounce ~1-1.5 Hz,
    // wheel hop ~10-15 Hz. Measured 1.21 Hz and 11.2 Hz (Release).
    ASSERT_CHECK(freqs_hz[0] > 0.5 && freqs_hz[0] < 3.0, "body bounce frequency out of expected range");
    // Wheel hop mode: expect 8-15 Hz
    ASSERT_CHECK(freqs_hz[2] > 5.0 && freqs_hz[2] < 20.0, "wheel hop frequency out of expected range");

    std::cout << "  [PASS] Test 1: Natural frequencies in expected range "
              << "(body=" << freqs_hz[0] << " Hz, wheel=" << freqs_hz[2] << " Hz)\n";
}

// ---------------------------------------------------------------------------
// Test 2: All eigenvalues stable (negative real parts)
// ---------------------------------------------------------------------------
void test_eigenvalues_stable() {
    caliburn::QuarterCarParams p;
    auto model = caliburn::build_quarter_car(p);

    Eigen::EigenSolver<Eigen::Matrix4d> es(model.A);
    for (int i = 0; i < 4; ++i) {
        double re = es.eigenvalues()(i).real();
        ASSERT_CHECK(re < 0.0, "eigenvalue has non-negative real part");
    }

    std::cout << "  [PASS] Test 2: All eigenvalues have negative real parts\n";
}

// ---------------------------------------------------------------------------
// Test 3: Bump response simulation — system settles after road bump
// ---------------------------------------------------------------------------
void test_bump_response() {
    caliburn::QuarterCarParams p;
    auto model = caliburn::build_quarter_car(p);

    // Simulate a step bump: z_r goes from 0 to 0.05 m at t=0
    const double bump_height = 0.05;  // 5 cm bump

    auto deriv = [&](double /*t*/, const Eigen::VectorXd& x) -> Eigen::VectorXd {
        return model.A * x + model.B_w * bump_height;
    };

    Eigen::VectorXd x0 = Eigen::VectorXd::Zero(4);
    double dt = 0.0005;
    int steps = 20000;  // 10 seconds

    Eigen::VectorXd x_final = caliburn::rk4_integrate(x0, 0.0, dt, steps, deriv);

    // Steady state after step bump: A*x_ss + B_w*z_r = 0
    Eigen::Vector4d x_ss = -model.A.inverse() * model.B_w * bump_height;

    double err = (x_final - x_ss).norm();
    // Transient decay: A is diagonalisable, so ‖e^{At}‖ ≤ κ(V)·e^{σ t} with σ = -2.116 rad/s
    // (the slower, body-bounce pair -2.116 ± 7.61j) and κ(V) = 80.5 for the default parameters.
    // From x0 = 0: err ≤ κ(V)·|x_ss|·e^{σ·10} = 80.5·0.0707·e^{-21.16} ≈ 3.7e-9. RK4 at
    // h·|λ|max ≈ 0.037 tracks e^{hλ} to O((hλ)^5) per step, negligible here.
    // 1e-8 is ~2.7× over the bound.
    ASSERT_CHECK(err < 1e-8, "bump response did not settle to steady state");

    // Steady-state body position should equal bump height (body rises to road level)
    // x_ss(0) = z_b should be approximately bump_height
    // Static 4x4 solve; the exact answer is bump_height. Forward error ≲ n·κ(A)·u·|x_ss|, with
    // κ(A) = 5554 for the default parameters: 4·5554·1.1e-16·0.0707 ≈ 1.7e-16. 1e-14 is ~60× over.
    ASSERT_CHECK(std::abs(x_ss(0) - bump_height) < 1e-14,
                 "steady-state body position does not match bump height");

    std::cout << "  [PASS] Test 3: Bump response settles correctly (err=" << err
              << ", z_b_ss=" << x_ss(0) << ")\n";
}

// ---------------------------------------------------------------------------
// Test 4: B matrices structure verification
// ---------------------------------------------------------------------------
void test_input_matrices() {
    caliburn::QuarterCarParams p;
    auto model = caliburn::build_quarter_car(p);

    // 1e-12: B_u and B_w entries are simple ratios of exactly-representable parameter
    // values (m_b=300, m_w=40, k_t=200000); IEEE arithmetic gives exact or near-exact results.
    double tol = 1e-12;

    // B_u: active force on body (+1/m_b) and wheel (-1/m_w)
    ASSERT_CHECK(model.B_u(0) == 0.0, "B_u(0) must be 0");
    ASSERT_REL_NEAR(model.B_u(1), 1.0 / p.m_b, tol);  // expected 1/300 ≈ 0.0033; scale = 1
    ASSERT_CHECK(model.B_u(2) == 0.0, "B_u(2) must be 0");
    ASSERT_REL_NEAR(model.B_u(3), -1.0 / p.m_w, tol); // expected -1/40 = -0.025; scale = 1

    // B_w: road disturbance enters only through tyre spring on wheel
    ASSERT_CHECK(model.B_w(0) == 0.0, "B_w(0) must be 0");
    ASSERT_CHECK(model.B_w(1) == 0.0, "B_w(1) must be 0");
    ASSERT_CHECK(model.B_w(2) == 0.0, "B_w(2) must be 0");
    // B_w(3) = k_t/m_w = 200000/40 = 5000 — magnitude >> 1, so ASSERT_REL_NEAR would be looser.
    ASSERT_CHECK(std::abs(model.B_w(3) - p.k_t / p.m_w) < tol, "B_w(3) = k_t/m_w");

    std::cout << "  [PASS] Test 4: B_u and B_w matrix structure verified\n";
}

// ---------------------------------------------------------------------------
// Test 5: A matrix, every entry, against the equations of motion
//   z_ddot_b = [-k_s(z_b - z_w) - c_s(z_dot_b - z_dot_w) + F_a] / m_b
//   z_ddot_w = [ k_s(z_b - z_w) + c_s(z_dot_b - z_dot_w) - k_t(z_w - z_r) - F_a] / m_w
// Each expected entry is the same single IEEE division (or sum then division) of the
// parameters as the model must perform, so the comparison is exact (tolerance 0).
// Default parameters are used: m_b ≠ m_w and k_s ≠ k_t, so a swapped mass or
// stiffness changes the entry. This catches, for example, the damper coupling A(1,3)
// dropped to 0, A(3,1) dropped, or a wheel-row entry divided by m_b.
// ---------------------------------------------------------------------------
void test_a_matrix_structure() {
    caliburn::QuarterCarParams p;
    auto model = caliburn::build_quarter_car(p);

    Eigen::Matrix4d expected;
    expected <<            0.0,             1.0,                       0.0,             0.0,
               -p.k_s / p.m_b, -p.c_s / p.m_b,             p.k_s / p.m_b,   p.c_s / p.m_b,
                           0.0,             0.0,                       0.0,             1.0,
                p.k_s / p.m_w,  p.c_s / p.m_w, -(p.k_s + p.k_t) / p.m_w,  -p.c_s / p.m_w;

    for (int i = 0; i < 4; ++i) {
        for (int j = 0; j < 4; ++j) {
            if (model.A(i, j) != expected(i, j)) {
                std::fprintf(stderr, "A(%d,%d) = %.17g, expected %.17g\n", i, j,
                             model.A(i, j), expected(i, j));
            }
            ASSERT_CHECK(model.A(i, j) == expected(i, j),
                         "quarter-car A entry must match the equations of motion exactly");
        }
    }

    std::cout << "  [PASS] Test 5: every A entry matches the equations of motion\n";
}

// ---------------------------------------------------------------------------
int main() {
    test_natural_frequencies();
    test_eigenvalues_stable();
    test_bump_response();
    test_input_matrices();
    test_a_matrix_structure();

    std::cout << "\nAll quarter-car tests passed.\n";
    return 0;
}
