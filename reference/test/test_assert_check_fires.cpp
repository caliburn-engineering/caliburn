// Built with NDEBUG and registered WILL_FAIL: ctest passes only if ASSERT_CHECK exits non-zero.
#include "assert_rel.h"

int main() {
    ASSERT_CHECK(2.0 < 1.0, "false condition must fire under NDEBUG");
    return 0;
}
