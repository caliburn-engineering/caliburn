#include "quarter_car.h"
#include "../integrators/rk4.h"
#include "../test/assert_rel.h"
#include <cmath>
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
    ASSERT_CHECK(err < 0.01, "bump response did not settle to steady state");

    // Steady-state body position should equal bump height (body rises to road level)
    // x_ss(0) = z_b should be approximately bump_height
    // 0.001: x_ss from -A⁻¹B_w*z_r; 4×4 well-conditioned system, A.inverse() accurate
    // to ~1e-12; generous bound for this static body-road equality.
    ASSERT_CHECK(std::abs(x_ss(0) - bump_height) < 0.001,
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
int main() {
    test_natural_frequencies();
    test_eigenvalues_stable();
    test_bump_response();
    test_input_matrices();

    std::cout << "\nAll quarter-car tests passed.\n";
    return 0;
}
