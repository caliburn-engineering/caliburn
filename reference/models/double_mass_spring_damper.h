#pragma once
#include <Eigen/Dense>

namespace caliburn {

/// Parameters for a double mass-spring-damper system.
/// Wall -- [k1,c1] -- m1 -- [k2,c2] -- m2 -- F(t)
struct DoubleMassSpringDamperParams {
    double m1 = 1.0;   // mass 1 [kg]
    double m2 = 1.0;   // mass 2 [kg]
    double k1 = 10.0;  // spring 1 stiffness [N/m]
    double k2 = 10.0;  // spring 2 stiffness [N/m]
    double c1 = 0.5;   // damper 1 coefficient [N*s/m]
    double c2 = 0.5;   // damper 2 coefficient [N*s/m]
};

/// State-space model of a double mass-spring-damper system.
/// States: [x1, v1, x2, v2]  Input: F on m2  Output: [x1, x2]
struct DoubleMassSpringDamperModel {
    Eigen::Matrix4d A;
    Eigen::Vector4d B;
    Eigen::Matrix<double, 2, 4> C;
};

/// Build the continuous-time state-space model (A, B, C) for a double
/// mass-spring-damper system from physical parameters.
DoubleMassSpringDamperModel build_double_msd(const DoubleMassSpringDamperParams& p);

}  // namespace caliburn
