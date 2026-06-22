#include "kalman_filter.h"

#include <cassert>
#include <cmath>
#include <cstdio>
#include <random>

// ---------------------------------------------------------------------------
// Test 1: Constant velocity 1D tracking
// ---------------------------------------------------------------------------
void test_constant_velocity_tracking() {
    const int n = 2;  // state: [position, velocity]
    const int m = 1;  // measurement: position only
    const double dt = 0.1;
    const double true_velocity = 5.0;
    const int num_steps = 100;

    // State transition matrix
    Eigen::MatrixXd F(n, n);
    F << 1.0, dt,
         0.0, 1.0;

    // Measurement matrix (observe position only)
    Eigen::MatrixXd H(m, n);
    H << 1.0, 0.0;

    // Process noise covariance (small)
    Eigen::MatrixXd Q = 0.01 * Eigen::MatrixXd::Identity(n, n);

    // Measurement noise covariance
    Eigen::MatrixXd R(m, m);
    R << 1.0;

    caliburn::KalmanFilter kf(n, m);
    kf.set_model(F, H, Q, R);

    // Initial state: zero position, zero velocity, large uncertainty
    Eigen::VectorXd x0 = Eigen::VectorXd::Zero(n);
    Eigen::MatrixXd P0 = 10.0 * Eigen::MatrixXd::Identity(n, n);
    kf.set_state(x0, P0);

    // Deterministic pseudo-random noise with fixed seed
    std::mt19937 rng(42);
    std::normal_distribution<double> noise(0.0, 1.0);

    // Track errors over the last 20 steps for convergence check
    double pos_error_sum = 0.0;
    double vel_estimate_sum = 0.0;
    const int tail_start = num_steps - 20;

    for (int k = 0; k < num_steps; ++k) {
        double t = (k + 1) * dt;
        double true_pos = true_velocity * t;

        // Noisy measurement
        Eigen::VectorXd z(m);
        z << true_pos + noise(rng);

        kf.predict();
        kf.update(z);

        if (k >= tail_start) {
            pos_error_sum += std::abs(kf.state()(0) - true_pos);
            vel_estimate_sum += kf.state()(1);
        }
    }

    double avg_pos_error = pos_error_sum / 20.0;
    double avg_vel_estimate = vel_estimate_sum / 20.0;

    assert(avg_pos_error < 0.5 && "Position estimate error should be < 0.5 after convergence");
    assert(std::abs(avg_vel_estimate - true_velocity) < 1.0 &&
           "Velocity estimate should converge near 5.0");

    std::printf("  [PASS] Test 1: Constant velocity 1D tracking\n");
}

// ---------------------------------------------------------------------------
// Test 2: Covariance converges (steady-state)
// ---------------------------------------------------------------------------
void test_covariance_convergence() {
    const int n = 2;
    const int m = 1;
    const double dt = 0.1;
    const double true_velocity = 5.0;
    const int num_steps = 200;

    Eigen::MatrixXd F(n, n);
    F << 1.0, dt,
         0.0, 1.0;

    Eigen::MatrixXd H(m, n);
    H << 1.0, 0.0;

    Eigen::MatrixXd Q = 0.01 * Eigen::MatrixXd::Identity(n, n);

    Eigen::MatrixXd R(m, m);
    R << 1.0;

    caliburn::KalmanFilter kf(n, m);
    kf.set_model(F, H, Q, R);

    Eigen::VectorXd x0 = Eigen::VectorXd::Zero(n);
    Eigen::MatrixXd P0 = 10.0 * Eigen::MatrixXd::Identity(n, n);
    kf.set_state(x0, P0);

    std::mt19937 rng(123);
    std::normal_distribution<double> noise(0.0, 1.0);

    for (int k = 0; k < num_steps; ++k) {
        double t = (k + 1) * dt;
        double true_pos = true_velocity * t;

        Eigen::VectorXd z(m);
        z << true_pos + noise(rng);

        kf.predict();
        kf.update(z);
    }

    const Eigen::MatrixXd& P_final = kf.covariance();

    // Diagonal elements of P should have decreased from initial values
    assert(P_final(0, 0) < P0(0, 0) &&
           "P(0,0) should decrease from initial value");
    assert(P_final(1, 1) < P0(1, 1) &&
           "P(1,1) should decrease from initial value");

    std::printf("  [PASS] Test 2: Covariance converges (steady-state)\n");
}

// ---------------------------------------------------------------------------
// Test 3: Innovation consistency
// ---------------------------------------------------------------------------
void test_innovation_consistency() {
    const int n = 2;
    const int m = 1;
    const double dt = 0.1;
    const double true_velocity = 5.0;
    const int num_steps = 200;
    const int tail_count = 50;
    const int tail_start = num_steps - tail_count;

    Eigen::MatrixXd F(n, n);
    F << 1.0, dt,
         0.0, 1.0;

    Eigen::MatrixXd H(m, n);
    H << 1.0, 0.0;

    Eigen::MatrixXd Q = 0.01 * Eigen::MatrixXd::Identity(n, n);

    Eigen::MatrixXd R(m, m);
    R << 1.0;

    caliburn::KalmanFilter kf(n, m);
    kf.set_model(F, H, Q, R);

    Eigen::VectorXd x0 = Eigen::VectorXd::Zero(n);
    Eigen::MatrixXd P0 = 10.0 * Eigen::MatrixXd::Identity(n, n);
    kf.set_state(x0, P0);

    std::mt19937 rng(7);
    std::normal_distribution<double> noise(0.0, 1.0);

    double innovation_abs_sum = 0.0;

    for (int k = 0; k < num_steps; ++k) {
        double t = (k + 1) * dt;
        double true_pos = true_velocity * t;

        Eigen::VectorXd z(m);
        z << true_pos + noise(rng);

        kf.predict();
        kf.update(z);

        if (k >= tail_start) {
            innovation_abs_sum += std::abs(kf.innovation()(0));
        }
    }

    double avg_innovation = innovation_abs_sum / static_cast<double>(tail_count);
    double three_sigma = 3.0 * std::sqrt(R(0, 0));

    assert(avg_innovation < three_sigma &&
           "Average innovation magnitude should be within 3-sigma of sqrt(R)");

    std::printf("  [PASS] Test 3: Innovation consistency\n");
}

// ---------------------------------------------------------------------------
int main() {
    test_constant_velocity_tracking();
    test_covariance_convergence();
    test_innovation_consistency();

    std::printf("\nAll Kalman filter tests passed.\n");
    return 0;
}
