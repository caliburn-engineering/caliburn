// Registered WILL_FAIL: ctest passes only if ASSERT_REL_NEAR rejects a NaN operand.
#include "assert_rel.h"

#include <limits>

int main() {
    ASSERT_REL_NEAR(std::numeric_limits<double>::quiet_NaN(), 0.0, 1.0);
    return 0;
}
