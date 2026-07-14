#pragma once
#include <Eigen/Dense>

namespace caliburn {

/// Parameters for the inverted pendulum on cart system.
struct InvertedPendulumParams {
    double M = 1.0;     // cart mass [kg]
    double m = 0.1;     // pendulum mass (point mass at tip) [kg]
    double L = 0.5;     // pendulum length [m]
    double g = 9.81;    // gravitational acceleration [m/s^2]
};

/// Linearised state-space model of inverted pendulum on cart.
/// States: [x, x_dot, theta, theta_dot]
/// Input: F (horizontal force on cart)
/// Output: [x, theta] (cart position and pendulum angle)
/// Linearised about theta = 0 (upright equilibrium).
struct InvertedPendulumModel {
    Eigen::Matrix4d A;
    Eigen::Vector4d B;
    Eigen::Matrix<double, 2, 4> C;
};

/// Build the linearised continuous-time state-space model for an
/// inverted pendulum on a cart.
InvertedPendulumModel build_inverted_pendulum(const InvertedPendulumParams& p);

}  // namespace caliburn
