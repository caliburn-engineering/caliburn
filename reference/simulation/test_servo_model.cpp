#include "servo_model.h"

#include "assert_rel.h"
#include <cmath>
#include <cstdio>

using caliburn::ServoModel;
using caliburn::ServoParams;

static constexpr double DEG = M_PI / 180.0;

// ---------------------------------------------------------------------------
// 1. Step response: after 5*tau, angle reaches ~99% of command
// ---------------------------------------------------------------------------
static void test_step_response() {
    ServoParams p{0.1, 1.0, 100.0, -M_PI, M_PI, 0.0};
    ServoModel servo(p);

    double target = 30.0 * DEG;
    double dt = 0.001;
    int steps = static_cast<int>(5.0 * p.tau / dt);  // 5*tau

    for (int i = 0; i < steps; ++i) {
        servo.step(target, dt);
    }

    double error = std::abs(servo.angle() - target);
    // 2%: first-order lag residual at 5τ is e^{-5}≈0.67%; 2% gives margin for dt=1ms discretisation
    ASSERT_CHECK(error < target * 0.02, "step response should be within 2% of target after 5 time constants");
    std::printf("  [PASS] Step response converges (error=%.6f rad)\n", error);
}

// ---------------------------------------------------------------------------
// 2. Time constant: after 1*tau, angle is ~63% of command
// ---------------------------------------------------------------------------
static void test_time_constant() {
    ServoParams p{0.1, 1.0, 100.0, -M_PI, M_PI, 0.0};
    ServoModel servo(p);

    double target = 1.0;  // 1 radian
    double dt = 0.0001;
    int steps = static_cast<int>(p.tau / dt);

    for (int i = 0; i < steps; ++i) {
        servo.step(target, dt);
    }

    double expected = target * (1.0 - std::exp(-1.0));  // 0.6321
    double error = std::abs(servo.angle() - expected);
    // 1e-2: Euler integration of first-order lag over τ=0.1s with dt=1e-4; global error O(dt/τ)≈1e-3
    ASSERT_CHECK(error < 0.01, "angle after 1 time constant should be within 0.01 rad of 63.2% of target");
    std::printf("  [PASS] Time constant check (angle=%.4f, expected=%.4f)\n",
                servo.angle(), expected);
}

// ---------------------------------------------------------------------------
// 3. Velocity saturation: large step clamps angular rate
// ---------------------------------------------------------------------------
static void test_velocity_saturation() {
    ServoParams p{0.01, 1.0, 5.0, -M_PI, M_PI, 0.0};  // omega_max = 5 rad/s
    ServoModel servo(p);

    // Large step: error/tau = 1.0/0.01 = 100 rad/s >> omega_max
    servo.step(1.0, 0.001);

    // 1e-10: velocity clamp adds floating-point tolerance; effective bound is omega_max
    ASSERT_CHECK(std::abs(servo.angular_velocity()) <= 5.0 + 1e-10, "angular velocity must not exceed omega_max");
    std::printf("  [PASS] Velocity saturation (omega=%.4f)\n", servo.angular_velocity());
}

// ---------------------------------------------------------------------------
// 4. Position clamping: command beyond limits is clamped
// ---------------------------------------------------------------------------
static void test_position_clamping() {
    ServoParams p{0.1, 1.0, 100.0, -0.5, 0.5, 0.0};
    ServoModel servo(p);

    double dt = 0.001;
    for (int i = 0; i < 10000; ++i) {
        servo.step(2.0, dt);  // command 2.0 rad, limit is 0.5
    }

    // 1e-10: position clamp adds floating-point tolerance; effective bound is angle_max
    ASSERT_CHECK(servo.angle() <= 0.5 + 1e-10, "servo position must not exceed position limit");
    std::printf("  [PASS] Position clamping (angle=%.4f)\n", servo.angle());
}

// ---------------------------------------------------------------------------
// 5. Dead zone: small errors produce no motion
// ---------------------------------------------------------------------------
static void test_dead_zone() {
    ServoParams p{0.1, 1.0, 100.0, -M_PI, M_PI, 0.02};  // 0.02 rad dead zone
    ServoModel servo(p);

    // Command within dead zone
    servo.step(0.01, 0.01);
    // 1e-10: no motion in dead zone; state should remain at exact zero
    ASSERT_REL_NEAR(servo.angle(), 0.0, 1e-10);
    ASSERT_REL_NEAR(servo.angular_velocity(), 0.0, 1e-10);

    // Command outside dead zone
    servo.step(0.1, 0.01);
    ASSERT_CHECK(servo.angular_velocity() > 0.0, "velocity positive outside dead zone");
    // error = K·θ_cmd - θ = 1·0.1 - 0 = 0.1; effective = 0.1 - dead_zone = 0.08;
    // ω = 0.08 / τ = 0.08 / 0.1 = 0.8.  Mutation (+= dead_zone) gives ω = 1.2.
    // 1e-15: 0.1, 0.02 and τ = 0.1 are not exact in binary (≤ u relative each), and the
    // subtraction and division each round once (≤ u), so |ω - 0.8| ≤ ~5u·0.8 ≈ 4.4e-16.
    ASSERT_REL_NEAR(servo.angular_velocity(), 0.8, 1e-15);
    std::printf("  [PASS] Dead zone\n");
}

// ---------------------------------------------------------------------------
// 6. Reset
// ---------------------------------------------------------------------------
static void test_reset() {
    ServoParams p{0.1, 1.0, 100.0, -M_PI, M_PI, 0.0};
    ServoModel servo(p);

    servo.step(1.0, 0.01);
    ASSERT_CHECK(servo.angle() != 0.0, "angle must change after a step command");

    servo.reset(0.5);
    ASSERT_CHECK(servo.angle() == 0.5, "angle set to exact reset value");
    ASSERT_CHECK(servo.angular_velocity() == 0.0, "velocity zeroed by reset");
    std::printf("  [PASS] Reset\n");
}

// ---------------------------------------------------------------------------
int main() {
    std::printf("Servo model tests:\n");

    test_step_response();
    test_time_constant();
    test_velocity_saturation();
    test_position_clamping();
    test_dead_zone();
    test_reset();

    std::printf("All servo model tests passed.\n");
    return 0;
}
