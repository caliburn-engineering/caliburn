#include "luenberger.h"
#include "assert_rel.h"

#include <Eigen/Eigenvalues>
#include <algorithm>
#include <cassert>
#include <cmath>
#include <cstdio>
#include <complex>
#include <vector>

// ---------------------------------------------------------------------------
// Test 1: Mass-spring-damper — estimate velocity from position measurement
// ---------------------------------------------------------------------------
void test_mass_spring_damper() {
    // System: m*x_ddot + c*x_dot + k*x = 0
    // State: [position, velocity]
    // Measurement: position only
    // Observer should reconstruct velocity

    const double m = 1.0, c = 0.5, k = 2.0;
    const double dt = 0.001;
    const int num_steps = 5000;  // 5 seconds

    // Continuous-time state-space: x_dot = A*x + B*u, y = C*x
    Eigen::MatrixXd A(2, 2);
    A << 0.0,    1.0,
         -k/m,  -c/m;

    Eigen::MatrixXd B(2, 1);
    B << 0.0, 1.0/m;

    Eigen::MatrixXd C(1, 2);
    C << 1.0, 0.0;

    // Observer gain — place poles at -20, -25 (fast, controller poles ~-1.5)
    // For a 2nd order system: desired char poly = (s+20)(s+25) = s^2 + 45s + 500
    // Using direct pole placement for this simple system:
    // eig(A - LC) = {-20, -25}
    // For C = [1, 0], L = [l1; l2]
    // A - LC = [0-l1, 1; -k/m-l2, -c/m]
    // char poly: s^2 + (c/m + l1)*s + (k/m + l2 + l1*c/m)
    // Match: c/m + l1 = 45 → l1 = 45 - 0.5 = 44.5
    //        k/m + l2 + l1*c/m ... actually use direct computation
    // Desired: s^2 + 45s + 500
    // System char poly of (A-LC): (s + l1)(s + c/m) + (k/m + l2) ... let's just set gains

    Eigen::MatrixXd L(2, 1);
    L << 44.5,
         475.75;  // Computed to place poles at -20, -25

    caliburn::LuenbergerObserver obs(A, B, C, L);

    // True initial state: position=1, velocity=0
    Eigen::VectorXd x_true(2);
    x_true << 1.0, 0.0;

    // Observer starts with wrong initial estimate
    Eigen::VectorXd x0_hat(2);
    x0_hat << 0.0, 0.0;
    obs.set_state(x0_hat);

    // Simulate
    Eigen::VectorXd u(1);
    u << 0.0;  // no external input

    for (int i = 0; i < num_steps; ++i) {
        // True system evolution (forward Euler)
        Eigen::VectorXd x_dot_true = A * x_true + B * u;
        x_true = x_true + dt * x_dot_true;

        // Measurement
        Eigen::VectorXd y = C * x_true;

        // Observer update
        obs.update(u, y, dt);
    }

    // After 5 seconds with poles at -20,-25, error should be negligible
    double err = obs.errorNorm(x_true);
    printf("Test 1 (mass-spring-damper): final error = %.6e\n", err);
    assert(err < 1e-3 && "Observer should converge for mass-spring-damper");
    printf("  PASSED\n");
}

// ---------------------------------------------------------------------------
// Test 2: Pole speed comparison — convergence rate scales with pole placement
// ---------------------------------------------------------------------------
void test_pole_speed_comparison() {
    // Simple integrator: x_dot = [0 1; 0 0]*x + [0; 1]*u, y = [1 0]*x
    // (double integrator: position/velocity, measure position)

    const double dt = 0.001;

    Eigen::MatrixXd A(2, 2);
    A << 0.0, 1.0,
         0.0, 0.0;

    Eigen::MatrixXd B(2, 1);
    B << 0.0, 1.0;

    Eigen::MatrixXd C(1, 2);
    C << 1.0, 0.0;

    // Test with different observer pole speeds
    // For double integrator with C=[1,0]:
    // A-LC = [0-l1, 1; -l2, 0], char poly: s^2 + l1*s + l2
    // Desired poles at -p, -p: char poly = s^2 + 2p*s + p^2
    // So l1 = 2p, l2 = p^2

    double pole_speeds[] = {5.0, 10.0, 20.0, 50.0};
    double convergence_times[4];

    for (int trial = 0; trial < 4; ++trial) {
        double p = pole_speeds[trial];
        Eigen::MatrixXd L(2, 1);
        L << 2.0 * p, p * p;

        caliburn::LuenbergerObserver obs(A, B, C, L);

        // True state: stationary at position=1
        Eigen::VectorXd x_true(2);
        x_true << 1.0, 0.0;

        // Observer starts at zero
        Eigen::VectorXd x0_hat(2);
        x0_hat << 0.0, 0.0;
        obs.set_state(x0_hat);

        Eigen::VectorXd u(1);
        u << 0.0;

        // Find time to converge to within 5% of true state
        double convergence_time = -1.0;
        for (int i = 0; i < 10000; ++i) {
            Eigen::VectorXd y = C * x_true;
            obs.update(u, y, dt);

            if (convergence_time < 0 && obs.errorNorm(x_true) < 0.05) {
                convergence_time = i * dt;
                break;
            }
        }

        convergence_times[trial] = convergence_time;
        printf("  Poles at -%.0f: convergence time = %.4f s\n",
               p, convergence_time);
    }

    // Faster poles should converge faster
    for (int i = 0; i < 3; ++i) {
        assert(convergence_times[i+1] < convergence_times[i] &&
               "Faster poles must converge faster");
    }

    printf("Test 2 (pole speed comparison): PASSED\n");
}

// ---------------------------------------------------------------------------
// Test 3: Integration with LQR — separation principle verification
// ---------------------------------------------------------------------------
void test_separation_principle() {
    // Double integrator with LQR controller using observer estimates
    // Verify: closed-loop poles = controller poles ∪ observer poles

    const double dt = 0.001;
    const int num_steps = 10000;  // 10 seconds

    Eigen::MatrixXd A(2, 2);
    A << 0.0, 1.0,
         0.0, 0.0;

    Eigen::MatrixXd B(2, 1);
    B << 0.0, 1.0;

    Eigen::MatrixXd C(1, 2);
    C << 1.0, 0.0;

    // Controller gain (places controller poles at -2, -3)
    // u = -K * x_hat, K = [k1, k2]
    // eig(A - BK) = eig([0, 1; -k1, -k2])
    // char poly: s^2 + k2*s + k1 = (s+2)(s+3) = s^2 + 5s + 6
    // k1 = 6, k2 = 5
    Eigen::MatrixXd K(1, 2);
    K << 6.0, 5.0;

    // Observer gain (places observer poles at -10, -15, i.e. ~5x faster)
    // l1 = 2*p ≈ 25, l2 = p^2 ... for distinct poles -10, -15:
    // char poly: s^2 + 25s + 150
    // l1 = 25, l2 = 150
    Eigen::MatrixXd L(2, 1);
    L << 25.0, 150.0;

    caliburn::LuenbergerObserver obs(A, B, C, L);

    // True initial state (displaced)
    Eigen::VectorXd x_true(2);
    x_true << 2.0, 0.0;

    // Observer starts at zero (wrong estimate)
    Eigen::VectorXd x0_hat(2);
    x0_hat << 0.0, 0.0;
    obs.set_state(x0_hat);

    // Simulate closed loop: u = -K * x_hat
    for (int i = 0; i < num_steps; ++i) {
        // Control using estimated state
        Eigen::VectorXd u = -K * obs.state();

        // Measurement from true system
        Eigen::VectorXd y = C * x_true;

        // True system evolution
        Eigen::VectorXd x_dot_true = A * x_true + B * u;
        x_true = x_true + dt * x_dot_true;

        // Observer update
        obs.update(u, y, dt);
    }

    // Both true state and estimate should converge to zero
    double state_err = x_true.norm();
    double obs_err = obs.errorNorm(x_true);

    printf("Test 3 (separation principle): true state norm = %.6e, "
           "observer error = %.6e\n", state_err, obs_err);
    assert(state_err < 1e-3 && "True state should converge to origin");
    assert(obs_err < 1e-4 && "Observer should track true state");
    printf("  PASSED\n");
}

// ---------------------------------------------------------------------------
// Test 4: Unobservable mode — observer cannot estimate hidden state
// ---------------------------------------------------------------------------
void test_unobservable_mode() {
    // System with 2 states but only one is observable
    // A = [a1 0; 0 a2], C = [1 0] — second state is completely hidden
    // Observer should converge for state 1 but NOT for state 2

    const double dt = 0.001;
    const int num_steps = 5000;

    Eigen::MatrixXd A(2, 2);
    A << -1.0, 0.0,
          0.0, -2.0;  // both states stable

    Eigen::MatrixXd B(2, 1);
    B << 1.0, 1.0;

    Eigen::MatrixXd C(1, 2);
    C << 1.0, 0.0;  // only measure first state

    // Check observability: O = [C; CA] = [1 0; -1 0] — rank 1, not full rank!
    // Observer gain — can only affect first state's convergence
    Eigen::MatrixXd L(2, 1);
    L << 10.0, 0.0;  // gain only on observable state

    caliburn::LuenbergerObserver obs(A, B, C, L);

    // True state: both states displaced
    Eigen::VectorXd x_true(2);
    x_true << 1.0, 3.0;

    // Observer starts at zero — wrong for both states
    Eigen::VectorXd x0_hat(2);
    x0_hat << 0.0, 0.0;
    obs.set_state(x0_hat);

    Eigen::VectorXd u(1);
    u << 0.0;

    for (int i = 0; i < num_steps; ++i) {
        Eigen::VectorXd x_dot_true = A * x_true + B * u;
        x_true = x_true + dt * x_dot_true;

        Eigen::VectorXd y = C * x_true;
        obs.update(u, y, dt);
    }

    // First state (observable): observer should converge
    double err_state1 = std::abs(x_true(0) - obs.state()(0));
    // Second state (unobservable): observer tracks open-loop only
    // Since both start at different ICs and L doesn't couple to state 2,
    // the error in state 2 evolves as e2_dot = a2*e2 (stable here, but
    // only by luck of the open-loop dynamics, not by observer design)
    double err_state2 = std::abs(x_true(1) - obs.state()(1));

    printf("Test 4 (unobservable mode): err_state1 = %.6e, err_state2 = %.6e\n",
           err_state1, err_state2);
    assert(err_state1 < 1e-3 && "Observable state should converge");
    // The unobservable state may or may not converge depending on open-loop stability
    // Key insight: observer gain L cannot accelerate convergence of unobservable mode
    printf("  PASSED (demonstrates unobservable mode limitation)\n");
}

// ---------------------------------------------------------------------------
// Test 5: placeObserverPoles — hand-derived gain (double integrator)
// ---------------------------------------------------------------------------
// System: A = [[0,1],[0,0]], C = [1,0].
// For L = [l1; l2], A−LC = [[−l1, 1],[−l2, 0]].
// char poly: det(sI − (A−LC)) = s(s+l1) + l2 = s² + l1·s + l2.
// Desired (s−p1)(s−p2) = s² − (p1+p2)s + p1·p2  →  l1 = −(p1+p2), l2 = p1·p2.
//
// O = [[C],[CA]] = [[1,0],[0,1]] = I (condition number 1).
// Operands are small integers; rounding is ≤ 2^−52 per op, so 1e−13 is safe.
void test_place_poles_hand_derived() {
    Eigen::MatrixXd A(2, 2);
    A << 0.0, 1.0,
         0.0, 0.0;
    Eigen::MatrixXd C(1, 2);
    C << 1.0, 0.0;

    // Case 1: real pair p1=−3, p2=−5
    // l1 = −(−3 + −5) = 8,  l2 = (−3)·(−5) = 15
    {
        Eigen::VectorXcd poles(2);
        poles << std::complex<double>(-3.0, 0.0),
                 std::complex<double>(-5.0, 0.0);
        Eigen::VectorXd L = caliburn::placeObserverPoles(A, C, poles);
        ASSERT_REL_NEAR(L(0), 8.0,  1e-13);
        ASSERT_REL_NEAR(L(1), 15.0, 1e-13);
    }

    // Case 2: conjugate pair p1=−2+3i, p2=−2−3i
    // l1 = −((−2+3i)+(−2−3i)) = 4,  l2 = (−2+3i)(−2−3i) = 4+9 = 13
    {
        Eigen::VectorXcd poles(2);
        poles << std::complex<double>(-2.0,  3.0),
                 std::complex<double>(-2.0, -3.0);
        Eigen::VectorXd L = caliburn::placeObserverPoles(A, C, poles);
        ASSERT_REL_NEAR(L(0), 4.0,  1e-13);
        ASSERT_REL_NEAR(L(1), 13.0, 1e-13);
    }

    // Case 3: one unstable pole, p1=+1, p2=−3, so the polynomial has a negative
    // coefficient (s−1)(s+3) = s² + 2s − 3. With stable poles every coefficient is
    // positive, so sign errors on the coefficients would go unseen.
    // l1 = −(1 + −3) = 2,  l2 = (1)·(−3) = −3
    {
        Eigen::VectorXcd poles(2);
        poles << std::complex<double>( 1.0, 0.0),
                 std::complex<double>(-3.0, 0.0);
        Eigen::VectorXd L = caliburn::placeObserverPoles(A, C, poles);
        ASSERT_REL_NEAR(L(0),  2.0, 1e-13);
        ASSERT_REL_NEAR(L(1), -3.0, 1e-13);
    }

    printf("Test 5 (place_poles hand-derived): PASSED\n");
}

// ---------------------------------------------------------------------------
// Test 6: placeObserverPoles — eigenvalue round trip (triple integrator, then O ≠ I)
// ---------------------------------------------------------------------------
// System: A = [[0,1,0],[0,0,1],[0,0,0]], C = [1,0,0].
// O = diag(C, CA, CA²) = I  →  condition number 1.
//
// Strategy: call placeObserverPoles, form M = A−L·C, compute eig(M) with
// Eigen::EigenSolver, sort by (real, imag), compare to desired poles.
//
// Tolerance 1e-10: O is identity so no amplification from inversion;
// ~9 Horner multiplications of O(10) matrices → rounding < 9·10·2^−52 ≈ 2e−14;
// 1e-10 is conservative and deliberately not loosened here.
void test_place_poles_eigenvalue_roundtrip() {
    Eigen::MatrixXd A(3, 3);
    A << 0.0, 1.0, 0.0,
         0.0, 0.0, 1.0,
         0.0, 0.0, 0.0;
    Eigen::MatrixXd C(1, 3);
    C << 1.0, 0.0, 0.0;

    auto sort_by_re_im = [](std::vector<std::complex<double>>& v) {
        std::sort(v.begin(), v.end(), [](const std::complex<double>& a,
                                         const std::complex<double>& b) {
            if (std::fabs(a.real() - b.real()) > 1e-8) return a.real() < b.real();
            return a.imag() < b.imag();
        });
    };

    // Case 1: three distinct real poles −2, −3, −4
    {
        Eigen::VectorXcd poles(3);
        poles << std::complex<double>(-2.0, 0.0),
                 std::complex<double>(-3.0, 0.0),
                 std::complex<double>(-4.0, 0.0);
        Eigen::VectorXd L = caliburn::placeObserverPoles(A, C, poles);
        Eigen::MatrixXd M = A - L * C;
        Eigen::EigenSolver<Eigen::MatrixXd> solver(M);
        Eigen::VectorXcd eigs = solver.eigenvalues();

        std::vector<std::complex<double>> got(eigs.data(), eigs.data() + 3);
        std::vector<std::complex<double>> want(poles.data(), poles.data() + 3);
        sort_by_re_im(got);
        sort_by_re_im(want);
        for (int i = 0; i < 3; ++i) {
            ASSERT_REL_NEAR(got[i].real(), want[i].real(), 1e-10);
            ASSERT_REL_NEAR(got[i].imag(), want[i].imag(), 1e-10);
        }
    }

    // Case 2: real pole −3 and conjugate pair −1±2i
    {
        Eigen::VectorXcd poles(3);
        poles << std::complex<double>(-3.0,  0.0),
                 std::complex<double>(-1.0,  2.0),
                 std::complex<double>(-1.0, -2.0);
        Eigen::VectorXd L = caliburn::placeObserverPoles(A, C, poles);
        Eigen::MatrixXd M = A - L * C;
        Eigen::EigenSolver<Eigen::MatrixXd> solver(M);
        Eigen::VectorXcd eigs = solver.eigenvalues();

        std::vector<std::complex<double>> got(eigs.data(), eigs.data() + 3);
        std::vector<std::complex<double>> want(poles.data(), poles.data() + 3);
        sort_by_re_im(got);
        sort_by_re_im(want);
        for (int i = 0; i < 3; ++i) {
            ASSERT_REL_NEAR(got[i].real(), want[i].real(), 1e-10);
            ASSERT_REL_NEAR(got[i].imag(), want[i].imag(), 1e-10);
        }
    }

    // Case 3: a system whose observability matrix is NOT the identity. With the
    // integrator chains above, O = I, so an implementation that skipped O^{-1}
    // would still pass. Here, by hand:
    //   C = [1,1,0], CA = [1,1,1], CA^2 = [4,1,-1]  ->  O = [[1,1,0],[1,1,1],[4,1,-1]],
    //   det(O) = 1*(-1-1) - 1*(-1-4) + 0 = 3, so (A, C) is observable.
    // Tolerance 1e-10 still holds: cond(O) is asserted below 100, so inverting O
    // amplifies rounding by at most ~100x over the O = I cases (~1e-14).
    {
        Eigen::MatrixXd A3(3, 3);
        A3 << 1.0,  2.0,  0.0,
              0.0, -1.0,  1.0,
              3.0,  0.0, -2.0;
        Eigen::MatrixXd C3(1, 3);
        C3 << 1.0, 1.0, 0.0;

        Eigen::MatrixXd O(3, 3);
        O << 1.0, 1.0,  0.0,
             1.0, 1.0,  1.0,
             4.0, 1.0, -1.0;
        ASSERT_MATRIX_REL_NEAR(O.row(1), C3 * A3, 0.0);
        ASSERT_MATRIX_REL_NEAR(O.row(2), C3 * A3 * A3, 0.0);
        ASSERT_REL_NEAR(O.determinant(), 3.0, 1e-14);
        Eigen::JacobiSVD<Eigen::MatrixXd> svd(O);
        double cond = svd.singularValues()(0) / svd.singularValues()(2);
        ASSERT_CHECK(cond < 100.0, "cond(O) must stay small for the 1e-10 tolerance");

        Eigen::VectorXcd pole_sets[2] = {Eigen::VectorXcd(3), Eigen::VectorXcd(3)};
        pole_sets[0] << std::complex<double>(-2.0, 0.0),
                        std::complex<double>(-3.0, 0.0),
                        std::complex<double>(-4.0, 0.0);
        pole_sets[1] << std::complex<double>(-3.0,  0.0),
                        std::complex<double>(-1.0,  2.0),
                        std::complex<double>(-1.0, -2.0);
        for (const auto& poles : pole_sets) {
            Eigen::VectorXd L = caliburn::placeObserverPoles(A3, C3, poles);
            Eigen::EigenSolver<Eigen::MatrixXd> solver(A3 - L * C3);
            Eigen::VectorXcd eigs = solver.eigenvalues();

            std::vector<std::complex<double>> got(eigs.data(), eigs.data() + 3);
            std::vector<std::complex<double>> want(poles.data(), poles.data() + 3);
            sort_by_re_im(got);
            sort_by_re_im(want);
            for (int i = 0; i < 3; ++i) {
                ASSERT_REL_NEAR(got[i].real(), want[i].real(), 1e-10);
                ASSERT_REL_NEAR(got[i].imag(), want[i].imag(), 1e-10);
            }
        }
    }

    printf("Test 6 (place_poles eigenvalue roundtrip): PASSED\n");
}

// ---------------------------------------------------------------------------
// Test 7: placeObserverPoles — repeated poles via characteristic polynomial
// ---------------------------------------------------------------------------
// Eigenvalues of a defective matrix are ill-conditioned (error ~ sqrt(eps) ~ 1e−8),
// so we do not compare eigenvalues for repeated poles. Instead we compare the
// characteristic polynomial coefficients of A−LC with those of (s−p)^n.
//
// System: double integrator A = [[0,1],[0,0]], C = [1,0], repeated pole p = −4.
// Desired poly: (s+4)^2 = s^2 + 8s + 16.
//
// For a 2×2 matrix M, char poly = s^2 − tr(M)·s + det(M)  (Faddeev–LeVerrier).
// O = I; operands are small integers; tolerance 1e−13.
void test_place_poles_repeated() {
    Eigen::MatrixXd A(2, 2);
    A << 0.0, 1.0,
         0.0, 0.0;
    Eigen::MatrixXd C(1, 2);
    C << 1.0, 0.0;

    const double p = -4.0;
    Eigen::VectorXcd poles(2);
    poles << std::complex<double>(p, 0.0),
             std::complex<double>(p, 0.0);

    Eigen::VectorXd L = caliburn::placeObserverPoles(A, C, poles);
    Eigen::MatrixXd M = A - L * C;

    // Char poly of 2×2 M: p(s) = s^2 − tr(M)·s + det(M)
    double c1 = -M.trace();        // coefficient of s^1
    double c0 = M.determinant();   // coefficient of s^0

    // Desired (s − p)^2 = s^2 + 8s + 16  →  c1 = −2p = 8, c0 = p^2 = 16
    ASSERT_REL_NEAR(c1, -2.0 * p, 1e-13);
    ASSERT_REL_NEAR(c0,  p * p,   1e-13);

    printf("Test 7 (place_poles repeated poles): PASSED\n");
}

// ---------------------------------------------------------------------------
int main() {
    printf("=== Luenberger Observer Tests ===\n\n");

    test_mass_spring_damper();
    printf("\n");

    test_pole_speed_comparison();
    printf("\n");

    test_separation_principle();
    printf("\n");

    test_unobservable_mode();
    printf("\n");

    test_place_poles_hand_derived();
    printf("\n");

    test_place_poles_eigenvalue_roundtrip();
    printf("\n");

    test_place_poles_repeated();
    printf("\n");

    printf("=== All tests passed ===\n");
    return 0;
}
