#include "inverted_pendulum.h"

namespace caliburn {

InvertedPendulumModel build_inverted_pendulum(const InvertedPendulumParams& p) {
    InvertedPendulumModel model;

    model.A = Eigen::Matrix4d::Zero();
    model.B = Eigen::Vector4d::Zero();
    model.C = Eigen::Matrix<double, 2, 4>::Zero();

    // Mass matrix determinant: D = m * L^2 * M
    const double D = p.m * p.L * p.L * p.M;

    // Linearised equations (about theta=0):
    //   (M+m)*x_ddot + m*L*theta_ddot = F
    //   m*L*x_ddot + m*L^2*theta_ddot = m*g*L*theta
    //
    // Solving for accelerations via mass matrix inversion:
    //   x_ddot     = [m*L^2 * F - m^2*g*L^2 * theta] / D
    //   theta_ddot = [-(m*L) * F + (M+m)*m*g*L * theta] / D

    // Row 0: x_dot = x_dot (trivial)
    model.A(0, 1) = 1.0;

    // Row 1: x_ddot = -m^2*g*L^2 / D * theta  (+ F terms in B)
    //       = -m*g/M * theta
    model.A(1, 2) = -(p.m * p.m * p.g * p.L * p.L) / D;

    // Row 2: theta_dot = theta_dot (trivial)
    model.A(2, 3) = 1.0;

    // Row 3: theta_ddot = (M+m)*m*g*L / D * theta  (+ F terms in B)
    //       = (M+m)*g/(M*L) * theta
    model.A(3, 2) = (p.M + p.m) * p.m * p.g * p.L / D;

    // Input: F on cart
    // x_ddot contribution:     m*L^2 / D = 1/M
    // theta_ddot contribution: -m*L / D  = -1/(M*L)
    model.B(1) =  p.m * p.L * p.L / D;
    model.B(3) = -p.m * p.L / D;

    // Output: cart position and pendulum angle
    model.C(0, 0) = 1.0;
    model.C(1, 2) = 1.0;

    return model;
}

}  // namespace caliburn
