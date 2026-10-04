// reference/observers/test_oracle_fixtures.cpp
//
// Default-suite check of the golden Kalman filter's steady state against the
// SciPy DARE fixture.  Needs no oracle at test time: the fixture is a
// checked-in header.
//
//  1. Staleness guard: the plant hash is recomputed from the live plant
//     (kalman_dare_plant.h) and compared with the one baked into the fixture.
//     A mismatch prints STALE FIXTURE, distinct from a numerical failure.
//  2. K_inf: the filter's own predict/update recursion runs until its
//     covariance stops moving.  The pre-update covariance P^- it settles on
//     must satisfy the DARE, and the gain it implies,
//     K_inf = P^- H' (H P^- H' + R)^-1, must match SciPy's to 1e-6 relative.
//     The filter does not expose its gain, so K_inf is formed from P^- here;
//     what is under test is the recursion's convergence, not a solver.

#include "kalman_filter.h"
#include "fixtures/kalman_dare_dmsd.h"
#include "tools/kalman_dare_plant.h"
#include "oracle_compare.h"

#include <Eigen/Dense>
#include <cinttypes>
#include <cstdint>
#include <cstdio>
#include <cstdlib>

using namespace caliburn;

namespace {

constexpr double kGainRelTol = 1e-6;
// The DARE residual of the converged P^-, relative to the size of its terms.
constexpr double kDareRelTol = 1e-9;

#define CHECK(cond, msg)                                          \
    do {                                                          \
        if (!(cond)) {                                            \
            std::fprintf(stderr, "FAIL: %s\n  at %s:%d\n",       \
                         (msg), __FILE__, __LINE__);              \
            std::exit(1);                                         \
        }                                                         \
    } while (0)

void test_staleness_guard() {
    const uint64_t live = kalman_dare::plantHash(kalman_dare::dmsdPlant());
    if (live != fixtures::KALMAN_DARE_DMSD_PLANT_HASH) {
        std::fprintf(stderr,
            "STALE FIXTURE: plant hash mismatch -- F, H, Q or R changed since\n"
            "  observers/fixtures/kalman_dare_dmsd.h was generated.\n"
            "  live hash    : %016" PRIx64 "\n"
            "  fixture hash : %016" PRIx64 "\n"
            "  Regenerate (from reference/):\n"
            "    cmake --build build --target export_kalman_dare\n"
            "    ./build/export_kalman_dare | python3 observers/tools/gen_oracle_fixtures.py - \\\n"
            "        > observers/fixtures/kalman_dare_dmsd.h\n",
            live, fixtures::KALMAN_DARE_DMSD_PLANT_HASH);
        std::exit(1);
    }
    std::printf("  [PASS] staleness guard (plant hash %016" PRIx64 ")\n", live);
}

void test_K_inf_vs_fixture() {
    const kalman_dare::Plant p = kalman_dare::dmsdPlant();

    KalmanFilter kf(4, 2);
    kf.set_model(p.F, p.H, p.Q, p.R);
    kf.set_state(Eigen::VectorXd::Zero(4), Eigen::MatrixXd::Identity(4, 4));

    // The covariance recursion does not depend on the measurements.
    const Eigen::VectorXd z = Eigen::VectorXd::Zero(2);

    Eigen::MatrixXd P_plus_prev = kf.covariance();
    Eigen::MatrixXd P_minus;
    int iters = 0;
    bool converged = false;
    for (; iters < 200000; ++iters) {
        kf.predict();
        P_minus = kf.covariance();
        kf.update(z);
        const double delta = (kf.covariance() - P_plus_prev).norm()
                             / (1.0 + P_plus_prev.norm());
        P_plus_prev = kf.covariance();
        if (delta < 1e-14) { converged = true; break; }
    }
    CHECK(converged, "the filter's covariance did not settle in 200000 steps");

    const Eigen::Matrix2d S = p.H * P_minus * p.H.transpose() + p.R;
    const Eigen::Matrix<double, 4, 2> K =
        P_minus * p.H.transpose() * S.llt().solve(Eigen::Matrix2d::Identity());

    // P^- must satisfy P = F P F' + Q - F P H' S^-1 H P F'.
    const Eigen::Matrix<double, 4, 2> FPHt = p.F * P_minus * p.H.transpose();
    const Eigen::Matrix4d FPFt = p.F * P_minus * p.F.transpose();
    const Eigen::Matrix4d gain_term = FPHt * S.llt().solve(FPHt.transpose());
    const double residual = (FPFt + p.Q - P_minus - gain_term).norm()
        / (FPFt.norm() + p.Q.norm() + P_minus.norm() + gain_term.norm());
    CHECK(residual < kDareRelTol,
          "the converged covariance does not satisfy the DARE");

    const Eigen::MatrixXd K_fixture = fixtures::kalman_dare_K_inf();
    if (!matrices_close_rel(K, K_fixture, kGainRelTol)) {
        std::fprintf(stderr, "FAIL: K_inf differs from the SciPy fixture (rel tol %.0e)\n",
                     kGainRelTol);
        for (int i = 0; i < 4; ++i)
            std::fprintf(stderr, "  filter  %.17g  %.17g   fixture  %.17g  %.17g\n",
                         K(i, 0), K(i, 1), K_fixture(i, 0), K_fixture(i, 1));
        std::exit(1);
    }

    std::printf("  [PASS] K_inf matches the SciPy fixture "
                "(%d steps, DARE rel residual %.2e, max |dK| %.2e)\n",
                iters + 1, residual, (K - K_fixture).cwiseAbs().maxCoeff());
}

}  // namespace

int main() {
    test_staleness_guard();
    test_K_inf_vs_fixture();
    std::printf("All Kalman oracle fixture tests passed.\n");
    return 0;
}
