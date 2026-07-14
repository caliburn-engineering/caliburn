#include "quarter_car.h"

namespace caliburn {

QuarterCarModel build_quarter_car(const QuarterCarParams& p) {
    QuarterCarModel model;

    model.A   = Eigen::Matrix4d::Zero();
    model.B_u = Eigen::Vector4d::Zero();
    model.B_w = Eigen::Vector4d::Zero();
    model.C   = Eigen::Matrix<double, 2, 4>::Zero();

    // Row 0: z_b_dot = z_dot_b (trivial kinematic row)
    model.A(0, 1) = 1.0;

    // Row 1: z_ddot_b = [-k_s*(z_b - z_w) - c_s*(z_dot_b - z_dot_w) + F_a] / m_b
    model.A(1, 0) = -p.k_s / p.m_b;
    model.A(1, 1) = -p.c_s / p.m_b;
    model.A(1, 2) =  p.k_s / p.m_b;
    model.A(1, 3) =  p.c_s / p.m_b;

    // Row 2: z_w_dot = z_dot_w (trivial kinematic row)
    model.A(2, 3) = 1.0;

    // Row 3: z_ddot_w = [k_s*(z_b-z_w) + c_s*(z_dot_b-z_dot_w) - k_t*(z_w-z_r) - F_a] / m_w
    model.A(3, 0) =  p.k_s / p.m_w;
    model.A(3, 1) =  p.c_s / p.m_w;
    model.A(3, 2) = -(p.k_s + p.k_t) / p.m_w;
    model.A(3, 3) = -p.c_s / p.m_w;

    // Control input: F_a (active suspension force)
    // Pushes body up (+1/m_b), pushes wheel down (-1/m_w)
    model.B_u(1) =  1.0 / p.m_b;
    model.B_u(3) = -1.0 / p.m_w;

    // Disturbance input: z_r (road profile)
    // Enters via tyre spring: k_t * z_r / m_w
    model.B_w(3) = p.k_t / p.m_w;

    // Output: body and wheel displacements
    model.C(0, 0) = 1.0;
    model.C(1, 2) = 1.0;

    return model;
}

}  // namespace caliburn
