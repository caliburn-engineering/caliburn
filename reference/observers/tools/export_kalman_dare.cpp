// reference/observers/tools/export_kalman_dare.cpp
//
// Emits the Kalman DARE plant (F, H, Q, R) and its hash as JSON, for
// tools/gen_oracle_fixtures.py to solve with SciPy.  The plant comes from
// kalman_dare_plant.h, the same definition the default-suite test uses.
//
// Usage (from reference/):
//   cmake --build build --target export_kalman_dare
//   ./build/export_kalman_dare | python3 observers/tools/gen_oracle_fixtures.py - \
//       > observers/fixtures/kalman_dare_dmsd.h

#include "kalman_dare_plant.h"

#include <cinttypes>
#include <cstdio>

int main() {
    using namespace caliburn::kalman_dare;
    const Plant p = dmsdPlant();
    std::printf("{\n");
    std::printf("  \"name\": \"double-mass-spring-damper\",\n");
    std::printf("  \"plant_hash\": \"%016" PRIx64 "\",\n", plantHash(p));
    std::printf("  \"F\": \"%s\",\n", matrixFull(p.F).c_str());
    std::printf("  \"H\": \"%s\",\n", matrixFull(p.H).c_str());
    std::printf("  \"Q\": \"%s\",\n", matrixFull(p.Q).c_str());
    std::printf("  \"R\": \"%s\"\n", matrixFull(p.R).c_str());
    std::printf("}\n");
    return 0;
}
