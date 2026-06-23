#include "servo_model.h"

#include <cassert>
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
    assert(error < target * 0.02);  // within 2%
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
    assert(error < 0.01);
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

    assert(std::abs(servo.angular_velocity()) <= 5.0 + 1e-10);
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

    assert(servo.angle() <= 0.5 + 1e-10);
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
    assert(std::abs(servo.angle()) < 1e-10);
    assert(std::abs(servo.angular_velocity()) < 1e-10);

    // Command outside dead zone
    servo.step(0.1, 0.01);
    assert(servo.angular_velocity() > 0.0);
    std::printf("  [PASS] Dead zone\n");
}

// ---------------------------------------------------------------------------
// 6. Reset
// ---------------------------------------------------------------------------
static void test_reset() {
    ServoParams p{0.1, 1.0, 100.0, -M_PI, M_PI, 0.0};
    ServoModel servo(p);

    servo.step(1.0, 0.01);
    assert(servo.angle() != 0.0);

    servo.reset(0.5);
    assert(servo.angle() == 0.5);
    assert(servo.angular_velocity() == 0.0);
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
