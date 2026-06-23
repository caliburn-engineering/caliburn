#include "constraint_enforcer.h"

#include <cassert>
#include <cmath>
#include <cstdio>

using caliburn::ConstraintEnforcer;
using caliburn::JointLimit;

// ---------------------------------------------------------------------------
// 1. Position within limits — no change
// ---------------------------------------------------------------------------
static void test_within_limits() {
    JointLimit lim{0, 1, -1.0, 1.0, 0.0};
    ConstraintEnforcer ce({lim});

    Eigen::VectorXd state(2);
    state << 0.5, 2.0;

    int active = ce.enforce(state);

    assert(active == 0);
    assert(state(0) == 0.5);
    assert(state(1) == 2.0);
    std::printf("  [PASS] Within limits — no change\n");
}

// ---------------------------------------------------------------------------
// 2. Position exceeds max — clamped, velocity zeroed
// ---------------------------------------------------------------------------
static void test_exceeds_max() {
    JointLimit lim{0, 1, -1.0, 1.0, 0.0};
    ConstraintEnforcer ce({lim});

    Eigen::VectorXd state(2);
    state << 1.5, 3.0;

    int active = ce.enforce(state);

    assert(active == 1);
    assert(state(0) == 1.0);
    assert(state(1) == 0.0);
    std::printf("  [PASS] Exceeds max — position clamped, velocity zeroed\n");
}

// ---------------------------------------------------------------------------
// 3. Position below min — clamped, velocity zeroed
// ---------------------------------------------------------------------------
static void test_below_min() {
    JointLimit lim{0, 1, -1.0, 1.0, 0.0};
    ConstraintEnforcer ce({lim});

    Eigen::VectorXd state(2);
    state << -2.0, -5.0;

    int active = ce.enforce(state);

    assert(active == 1);
    assert(state(0) == -1.0);
    assert(state(1) == 0.0);
    std::printf("  [PASS] Below min — position clamped, velocity zeroed\n");
}

// ---------------------------------------------------------------------------
// 4. Velocity saturation — velocity clamped, position untouched
// ---------------------------------------------------------------------------
static void test_velocity_saturation() {
    JointLimit lim{0, 1, -1.0, 1.0, 5.0};  // max_velocity = 5.0
    ConstraintEnforcer ce({lim});

    Eigen::VectorXd state(2);
    state << 0.5, 10.0;

    int active = ce.enforce(state);

    assert(active == 1);
    assert(state(0) == 0.5);
    assert(std::abs(state(1) - 5.0) < 1e-12);
    std::printf("  [PASS] Velocity saturation\n");
}

// ---------------------------------------------------------------------------
// 5. Multiple joints
// ---------------------------------------------------------------------------
static void test_multiple_joints() {
    std::vector<JointLimit> limits = {
        {0, 2, -0.5, 0.5, 0.0},  // joint 0: pos at idx 0, vel at idx 2
        {1, 3, -0.3, 0.3, 0.0},  // joint 1: pos at idx 1, vel at idx 3
    };
    ConstraintEnforcer ce(limits);

    Eigen::VectorXd state(4);
    state << 0.8, -0.5, 1.0, -2.0;

    int active = ce.enforce(state);

    assert(active == 2);
    assert(state(0) == 0.5);
    assert(state(1) == -0.3);
    assert(state(2) == 0.0);
    assert(state(3) == 0.0);
    std::printf("  [PASS] Multiple joints constrained\n");
}

// ---------------------------------------------------------------------------
// 6. Static clamp utility
// ---------------------------------------------------------------------------
static void test_clamp() {
    assert(ConstraintEnforcer::clamp(5.0, -1.0, 1.0) == 1.0);
    assert(ConstraintEnforcer::clamp(-5.0, -1.0, 1.0) == -1.0);
    assert(ConstraintEnforcer::clamp(0.5, -1.0, 1.0) == 0.5);
    std::printf("  [PASS] Static clamp\n");
}

// ---------------------------------------------------------------------------
int main() {
    std::printf("Constraint enforcer tests:\n");

    test_within_limits();
    test_exceeds_max();
    test_below_min();
    test_velocity_saturation();
    test_multiple_joints();
    test_clamp();

    std::printf("All constraint enforcer tests passed.\n");
    return 0;
}
