#pragma once
#include <Eigen/Dense>

namespace caliburn {

/// Parameters for a quarter-car suspension model.
struct QuarterCarParams {
    double m_b = 300.0;     // sprung mass (body) [kg]
    double m_w = 40.0;      // unsprung mass (wheel) [kg]
    double k_s = 20000.0;   // suspension spring stiffness [N/m]
    double c_s = 1500.0;    // suspension damper coefficient [N*s/m]
    double k_t = 200000.0;  // tyre stiffness [N/m]
};

/// State-space model of a quarter-car suspension.
/// States: [z_b, z_dot_b, z_w, z_dot_w]
/// Control input: F_a (active suspension force)
/// Disturbance input: z_r (road profile)
/// Output: [z_b, z_w] (body and wheel displacements)
struct QuarterCarModel {
    Eigen::Matrix4d A;
    Eigen::Vector4d B_u;   // control input column
    Eigen::Vector4d B_w;   // disturbance input column
    Eigen::Matrix<double, 2, 4> C;
};

/// Build the continuous-time state-space model for a quarter-car
/// suspension from physical parameters.
QuarterCarModel build_quarter_car(const QuarterCarParams& p);

}  // namespace caliburn
