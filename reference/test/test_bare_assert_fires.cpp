// Built with NDEBUG via target_compile_definitions; the source-file -UNDEBUG guard
// keeps assert active. WILL_FAIL: ctest passes only if assert fires (non-zero exit).
//
// assert() calls abort() which sends SIGABRT; ctest reports that as "Subprocess aborted"
// rather than treating it as a non-zero exit, so WILL_FAIL does not catch it. A SIGABRT
// handler that calls exit(1) converts the signal into the non-zero exit code WILL_FAIL
// expects, while leaving the bare assert call itself unchanged.
#include <cassert>
#include <csignal>
#include <cstdlib>

static void on_abort(int) { std::exit(1); }

int main() {
    std::signal(SIGABRT, on_abort);
    assert(false);
    return 0;
}
