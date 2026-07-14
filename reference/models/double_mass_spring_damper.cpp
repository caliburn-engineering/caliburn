#include "double_mass_spring_damper.h"

namespace caliburn {

DoubleMassSpringDamperModel build_double_msd(const DoubleMassSpringDamperParams& p) {
    DoubleMassSpringDamperModel model;

    model.A = Eigen::Matrix4d::Zero();
    model.B = Eigen::Vector4d::Zero();
    model.C = Eigen::Matrix<double, 2, 4>::Zero();

    // Row 0: x1_dot = v1 (trivial kinematic row)
    model.A(0, 1) = 1.0;

    // Row 1: v1_dot = [-(k1+k2)*x1 - (c1+c2)*v1 + k2*x2 + c2*v2] / m1
    model.A(1, 0) = -(p.k1 + p.k2) / p.m1;
    model.A(1, 1) = -(p.c1 + p.c2) / p.m1;
    model.A(1, 2) =  p.k2 / p.m1;
    model.A(1, 3) =  p.c2 / p.m1;

    // Row 2: x2_dot = v2 (trivial kinematic row)
    model.A(2, 3) = 1.0;

    // Row 3: v2_dot = [k2*x1 + c2*v1 - k2*x2 - c2*v2 + F] / m2
    model.A(3, 0) =  p.k2 / p.m2;
    model.A(3, 1) =  p.c2 / p.m2;
    model.A(3, 2) = -p.k2 / p.m2;
    model.A(3, 3) = -p.c2 / p.m2;

    // Input: F acts on m2
    model.B(3) = 1.0 / p.m2;

    // Output: positions of both masses
    model.C(0, 0) = 1.0;
    model.C(1, 2) = 1.0;

    return model;
}

}  // namespace caliburn
