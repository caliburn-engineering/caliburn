// reference/observers/test_live_oracle.cpp
//
// Live-oracle check of the Kalman DARE fixture: runs export_kalman_dare and
// gen_oracle_fixtures.py, and cross-checks the fresh SciPy solve against the
// checked-in observers/fixtures/kalman_dare_dmsd.h.
//
// Gated by CALIBURN_LIVE_ORACLE (default OFF) and labelled "live-oracle".
// Exits 77, which ctest reports as Skipped, when scipy does not import in the
// interpreter CALIBURN_ORACLE_PYTHON names.
//
// The fresh header must come from scipy (its Oracle line), carry the same plant
// hash, and give the same K_inf to 1e-10 relative.  The values are compared as
// numbers, never as text: a different scipy, LAPACK or CPU moves the last of
// the 17 printed digits, and that is not a stale fixture.
//
// To invoke (from reference/):
//   cmake -S . -B build -DCALIBURN_LIVE_ORACLE=ON \
//         -DCALIBURN_ORACLE_PYTHON=.oracle_venv/bin/python3
//   cmake --build build -j2
//   ctest --test-dir build -L live-oracle --output-on-failure

#include "fixtures/kalman_dare_dmsd.h"
#include "oracle_compare.h"

#include <Eigen/Dense>
#include <cinttypes>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <regex>
#include <sstream>
#include <string>
#include <unistd.h>

#ifndef EXPORT_KALMAN_DARE_EXE
#  error "EXPORT_KALMAN_DARE_EXE must be defined by CMake"
#endif
#ifndef ORACLE_SCRIPT
#  error "ORACLE_SCRIPT must be defined by CMake"
#endif
#ifndef ORACLE_PYTHON
#  error "ORACLE_PYTHON must be defined by CMake"
#endif

namespace {

constexpr int kSkip = 77;
// Two SciPy solves of the same matrices agree far tighter than this.
constexpr double kRelTol = 1e-10;

std::string read_file(const char* path) {
    std::ifstream f(path);
    std::ostringstream ss;
    ss << f.rdbuf();
    return ss.str();
}

}  // namespace

int main() {
    if (std::system("\"" ORACLE_PYTHON "\" -c \"import scipy\" 2>/dev/null") != 0) {
        std::printf("SKIP: scipy does not import in %s; install it there, or "
                    "point CALIBURN_ORACLE_PYTHON at a Python that has it\n",
                    ORACLE_PYTHON);
        return kSkip;
    }

    char fresh_path[256];
    std::snprintf(fresh_path, sizeof(fresh_path),
                  "/tmp/kalman_live_oracle_%d.h", (int)getpid());
    char cmd[2048];
    std::snprintf(cmd, sizeof(cmd), "\"%s\" | \"%s\" \"%s\" - > \"%s\"",
                  EXPORT_KALMAN_DARE_EXE, ORACLE_PYTHON, ORACLE_SCRIPT, fresh_path);
    const int rc = std::system(cmd);
    const std::string fresh = read_file(fresh_path);
    std::remove(fresh_path);
    if (rc != 0 || fresh.empty()) {
        std::fprintf(stderr, "FAIL: export_kalman_dare | gen_oracle_fixtures.py failed\n");
        return 1;
    }

    std::smatch m;
    if (!std::regex_search(fresh, m, std::regex(R"(// Oracle: (scipy [^\n]*))"))) {
        std::fprintf(stderr, "FAIL: the fresh fixture does not come from scipy\n");
        return 1;
    }
    const std::string oracle = m[1];

    if (!std::regex_search(fresh, m, std::regex(R"(PLANT_HASH = 0x([0-9a-f]{16})ULL)"))) {
        std::fprintf(stderr, "FAIL: no plant hash in the fresh fixture\n");
        return 1;
    }
    const uint64_t fresh_hash = std::stoull(m[1].str(), nullptr, 16);
    if (fresh_hash != caliburn::fixtures::KALMAN_DARE_DMSD_PLANT_HASH) {
        std::fprintf(stderr,
            "FAIL: plant hash %016" PRIx64 " from the exporter, %016" PRIx64
            " in the fixture -- the fixture is stale\n",
            fresh_hash, caliburn::fixtures::KALMAN_DARE_DMSD_PLANT_HASH);
        return 1;
    }

    const Eigen::MatrixXd stored = caliburn::fixtures::kalman_dare_K_inf();
    Eigen::MatrixXd K = Eigen::MatrixXd::Constant(stored.rows(), stored.cols(), NAN);
    const std::regex elem(R"(K\((\d+), (\d+)\) = ([^;]+);)");
    for (std::sregex_iterator it(fresh.begin(), fresh.end(), elem), end; it != end; ++it) {
        const int i = std::stoi((*it)[1]);
        const int j = std::stoi((*it)[2]);
        if (i < K.rows() && j < K.cols()) K(i, j) = std::stod((*it)[3]);
    }
    if (!K.allFinite()) {
        std::fprintf(stderr, "FAIL: the fresh fixture does not give every K_inf element\n");
        return 1;
    }
    if (!caliburn::matrices_close_rel(K, stored, kRelTol)) {
        std::fprintf(stderr, "FAIL: SciPy's K_inf differs from the fixture (rel tol %.0e)\n",
                     kRelTol);
        for (int i = 0; i < K.rows(); ++i)
            std::fprintf(stderr, "  fresh  %.17g  %.17g   fixture  %.17g  %.17g\n",
                         K(i, 0), K(i, 1), stored(i, 0), stored(i, 1));
        return 1;
    }

    std::printf("Live oracle check passed: %s agrees with kalman_dare_dmsd.h "
                "(max |dK| %.2e)\n", oracle.c_str(), (K - stored).cwiseAbs().maxCoeff());
    return 0;
}
