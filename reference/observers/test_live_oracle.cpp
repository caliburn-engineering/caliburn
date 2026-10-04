// Live-oracle test: run gen_oracle_fixtures.py and verify its K_inf matches
// the checked-in fixture within rtol = 1e-6.
//
// This test is gated by the CALIBURN_LIVE_ORACLE CMake option (default OFF)
// and carries the "live-oracle" ctest label.  It exits with code 77 when
// SciPy is not importable so ctest reports SKIP rather than FAIL.
//
// The Python interpreter is selected by CALIBURN_ORACLE_PYTHON (default
// "python3").

#include "fixtures/kalman_dare_dmsd.h"
#include "oracle_compare.h"

#include <Eigen/Dense>
#include <cassert>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <array>
#include <string>
#include <vector>

// Path to the Python script, injected by CMake via -D.
#ifndef ORACLE_SCRIPT
#  error "ORACLE_SCRIPT must be defined by CMake (-DORACLE_SCRIPT=<path>)"
#endif

// Python interpreter to use, injected by CMake via -D.
#ifndef ORACLE_PYTHON
#  define ORACLE_PYTHON "python3"
#endif

// ---------------------------------------------------------------------------
// Parse a K_inf block from the script's output.
// The script outputs a header file; we extract the four numeric rows inside
//   K <<
//       <r00>, <r01>,
//       ...;
// ---------------------------------------------------------------------------
static bool parse_K_from_header(const std::string& text,
                                 Eigen::Matrix<double, 4, 2>& K) {
    // Locate "K <<" then read 4 rows of two doubles
    const char* p = std::strstr(text.c_str(), "K <<");
    if (!p) return false;
    p += 4;

    int filled = 0;
    while (*p && filled < 8) {
        // skip non-digit chars (spaces, newlines, commas, '<', '>')
        while (*p && !std::isdigit((unsigned char)*p) && *p != '-' && *p != '+') {
            if (*p == ';') goto done;  // reached end of matrix literal
            ++p;
        }
        if (!*p) break;
        char* end;
        double v = std::strtod(p, &end);
        if (end == p) { ++p; continue; }
        K(filled / 2, filled % 2) = v;
        ++filled;
        p = end;
    }
done:
    return filled == 8;
}

int main() {
    // Build the command: python3 tools/gen_oracle_fixtures.py
    std::string cmd = std::string(ORACLE_PYTHON) + " " + ORACLE_SCRIPT + " 2>&1";

    // Run and capture stdout
    FILE* pipe = popen(cmd.c_str(), "r");
    if (!pipe) {
        std::fprintf(stderr, "popen failed for: %s\n", cmd.c_str());
        return 1;
    }

    std::string output;
    std::array<char, 4096> buf;
    while (std::fgets(buf.data(), static_cast<int>(buf.size()), pipe))
        output += buf.data();

    int rc = pclose(pipe);

    // exit 77 when SciPy is missing (script exits with code 1 and prints to stderr)
    if (rc != 0) {
        if (output.find("SciPy is not installed") != std::string::npos ||
            output.find("No module named 'scipy'") != std::string::npos ||
            output.find("No module named 'numpy'") != std::string::npos) {
            std::fprintf(stdout, "SKIP: SciPy not available — skipping live-oracle test\n");
            return 77;  // ctest SKIP
        }
        std::fprintf(stderr, "Oracle script failed (rc=%d):\n%s\n", rc, output.c_str());
        return 1;
    }

    // Parse K_inf from the generated header text
    Eigen::Matrix<double, 4, 2> K_oracle;
    if (!parse_K_from_header(output, K_oracle)) {
        std::fprintf(stderr,
            "Failed to parse K_inf from oracle output:\n%s\n", output.c_str());
        return 1;
    }

    // Compare against checked-in fixture
    Eigen::MatrixXd K_fixture = caliburn::fixtures::kalman_dare_K_inf().cast<double>();
    Eigen::MatrixXd K_oracle_d = K_oracle.cast<double>();

    bool ok = caliburn::matrices_close_rel(K_oracle_d, K_fixture, 1e-6);
    if (!ok) {
        std::fprintf(stderr, "LIVE ORACLE MISMATCH\n");
        std::fprintf(stderr, "SciPy K_inf:\n");
        for (int i = 0; i < 4; ++i)
            std::fprintf(stderr, "  %.17g  %.17g\n", K_oracle(i,0), K_oracle(i,1));
        std::fprintf(stderr, "Fixture K_inf:\n");
        for (int i = 0; i < 4; ++i)
            std::fprintf(stderr, "  %.17g  %.17g\n", K_fixture(i,0), K_fixture(i,1));
    }
    assert(ok && "SciPy K_inf does not match checked-in fixture");

    std::printf("  [PASS] Live oracle: SciPy K_inf matches fixture within rtol=1e-6\n");
    return 0;
}
