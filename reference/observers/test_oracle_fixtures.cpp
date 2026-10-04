// Default-suite oracle test for the steady-state Kalman gain.
//
// Two checks per run:
//  1. Staleness guard — recompute the plant hash and compare it against the
//     value baked into the fixture header.  A mismatch means the fixture was
//     generated from different parameters (STALE FIXTURE).
//  2. K_inf convergence — run the recursive DRE iteration using
//     KalmanFilter::predict/update until the post-update P^+ converges, then
//     extract the corresponding pre-update P^- and compare K_inf against the
//     oracle fixture within rtol = 1e-6.  The fixture stores the steady-state
//     Kalman gain derived from P^- (the pre-update covariance), matching the
//     convention used by gen_oracle_fixtures.py and export_kalman_dare.cpp.
//
// This test requires no external oracle.  The live-oracle test
// (test_live_oracle) verifies that the fixture itself matches SciPy.

#include "kalman_filter.h"
#include "double_mass_spring_damper.h"
#include "fixtures/kalman_dare_dmsd.h"
#include "tools/plant_hash.h"
#include "oracle_compare.h"

#include <Eigen/Dense>
#include <unsupported/Eigen/MatrixFunctions>
#include <cstdint>
#include <cstdio>
#include <cstdlib>

// ---------------------------------------------------------------------------
// Parameters — must match gen_oracle_fixtures.py and export_kalman_dare.cpp
// ---------------------------------------------------------------------------
static constexpr double DT      = 0.01;
static constexpr double Q_SCALE = 1e-4;
static constexpr double R_SCALE = 1e-2;

// Hard assertion that prints a message and exits 1 instead of calling abort(),
// so it fires in both Debug and Release builds.
#define CHECK(cond, msg)                                          \
    do {                                                          \
        if (!(cond)) {                                            \
            std::fprintf(stderr, "FAIL: %s\n  at %s:%d\n",       \
                         (msg), __FILE__, __LINE__);              \
            std::exit(1);                                         \
        }                                                         \
    } while (0)

// ---------------------------------------------------------------------------
// Test 1: Staleness guard
// ---------------------------------------------------------------------------
static void test_staleness_guard() {
    caliburn::DoubleMassSpringDamperParams p;  // default values

    uint64_t computed = kalman_dare_dmsd_plant_hash(
        p.m1, p.m2, p.k1, p.k2, p.c1, p.c2, DT, Q_SCALE, R_SCALE);

    if (computed != caliburn::fixtures::KALMAN_DARE_DMSD_PLANT_HASH) {
        std::fprintf(stderr,
            "STALE FIXTURE: plant hash mismatch\n"
            "  computed 0x%016llx\n"
            "  fixture  0x%016llx\n"
            "  Regenerate: python3 tools/gen_oracle_fixtures.py > "
            "fixtures/kalman_dare_dmsd.h\n",
            (unsigned long long)computed,
            (unsigned long long)caliburn::fixtures::KALMAN_DARE_DMSD_PLANT_HASH);
        std::exit(1);
    }

    std::printf("  [PASS] Test 1: Staleness guard (plant hash matches)\n");
}

// ---------------------------------------------------------------------------
// Test 2: K_inf matches fixture within rtol = 1e-6
// ---------------------------------------------------------------------------
static void test_K_inf_vs_fixture() {
    caliburn::DoubleMassSpringDamperParams p;  // default values

    // Build continuous-time state matrix
    Eigen::Matrix4d Ac;
    Ac <<  0,                1,               0,           0,
          -(p.k1+p.k2)/p.m1, -(p.c1+p.c2)/p.m1, p.k2/p.m1, p.c2/p.m1,
           0,                0,               0,           1,
           p.k2/p.m2,        p.c2/p.m2,      -p.k2/p.m2, -p.c2/p.m2;

    // ZOH discretisation
    Eigen::Matrix4d F = (Ac * DT).exp();

    // Measurement matrix: observe positions x1, x2
    Eigen::Matrix<double, 2, 4> H;
    H << 1, 0, 0, 0,
         0, 0, 1, 0;

    Eigen::Matrix4d Q = Q_SCALE * Eigen::Matrix4d::Identity();
    Eigen::Matrix2d R = R_SCALE * Eigen::Matrix2d::Identity();

    // Initialise KalmanFilter; run recursive predict/update until P^+ converges.
    // The pre-update covariance P^- at convergence is the DARE solution.
    caliburn::KalmanFilter kf(4, 2);
    kf.set_model(F.cast<double>(), H.cast<double>(),
                 Q.cast<double>(), R.cast<double>());
    kf.set_state(Eigen::VectorXd::Zero(4),
                 Eigen::MatrixXd::Identity(4, 4));

    // Dummy measurement: state does not affect covariance evolution.
    Eigen::VectorXd z = Eigen::VectorXd::Zero(2);

    Eigen::MatrixXd P_plus_prev = kf.covariance();
    Eigen::MatrixXd P_minus;   // pre-update covariance captured just before each update
    int iters = 0;
    for (; iters < 200000; ++iters) {
        kf.predict();
        P_minus = kf.covariance();   // P^- = F P^+ F^T + Q (pre-update)
        kf.update(z);
        double delta = (kf.covariance() - P_plus_prev).norm()
                       / (1.0 + P_plus_prev.norm());
        P_plus_prev = kf.covariance();
        if (delta < 1e-12) break;
    }

    // Steady-state Kalman gain: K_inf = P^-_inf H^T (H P^-_inf H^T + R)^{-1}
    Eigen::Matrix2d S_inf = H * P_minus * H.transpose() + R;
    Eigen::Matrix<double, 4, 2> K_computed =
        P_minus * H.transpose() * S_inf.llt().solve(Eigen::Matrix2d::Identity());

    // Verify DARE residual:  F P F^T + Q - P - F P H^T S^{-1} H P F^T  ≈  0
    Eigen::Matrix<double, 4, 2> FPHt = F * P_minus * H.transpose();
    Eigen::Matrix4d dare_residual =
        F * P_minus * F.transpose() + Q - P_minus
        - FPHt * S_inf.llt().solve(FPHt.transpose());
    double residual_norm = dare_residual.norm();
    CHECK(residual_norm < 1e-6,
          "Converged covariance does not satisfy the DARE (residual too large)");

    // Compare against oracle fixture
    Eigen::MatrixXd K_fixture = caliburn::fixtures::kalman_dare_K_inf().cast<double>();
    Eigen::MatrixXd K_computed_d = K_computed.cast<double>();

    if (!caliburn::matrices_close_rel(K_computed_d, K_fixture, 1e-6)) {
        std::fprintf(stderr, "K_inf mismatch (rtol 1e-6)\n");
        std::fprintf(stderr, "Computed:\n");
        for (int i = 0; i < 4; ++i)
            std::fprintf(stderr, "  %.17g  %.17g\n",
                         K_computed(i, 0), K_computed(i, 1));
        std::fprintf(stderr, "Fixture:\n");
        for (int i = 0; i < 4; ++i)
            std::fprintf(stderr, "  %.17g  %.17g\n",
                         K_fixture(i, 0), K_fixture(i, 1));
        std::exit(1);
    }

    std::printf("  [PASS] Test 2: K_inf vs oracle fixture "
                "(converged in %d iters, DARE residual = %.2e)\n",
                iters + 1, residual_norm);
}

// ---------------------------------------------------------------------------
int main() {
    test_staleness_guard();
    test_K_inf_vs_fixture();

    std::printf("\nAll oracle fixture tests passed.\n");
    return 0;
}
