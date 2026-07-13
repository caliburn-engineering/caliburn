#include "bicycle_model.h"

#include <Eigen/Eigenvalues>
#include <limits>

namespace caliburn {

BicycleModel::BicycleModel(const VehicleParams& params) : params_(params) {}

void BicycleModel::compute_matrices(double V, Eigen::Matrix2d& A, Eigen::Vector2d& B) const {
    const double m = params_.m;
    const double Iz = params_.Iz;
    const double Lf = params_.Lf;
    const double Lr = params_.Lr;
    const double Cf = params_.Cf;
    const double Cr = params_.Cr;

    A(0, 0) = -(Cf + Cr) / (m * V);
    A(0, 1) = -V - (Lf * Cf - Lr * Cr) / (m * V);
    A(1, 0) = -(Lf * Cf - Lr * Cr) / (Iz * V);
    A(1, 1) = -(Lf * Lf * Cf + Lr * Lr * Cr) / (Iz * V);

    B(0) = Cf / m;
    B(1) = Lf * Cf / Iz;
}

void BicycleModel::step_euler(State& x, double delta, double V, double dt) const {
    Eigen::Matrix2d A;
    Eigen::Vector2d B;
    compute_matrices(V, A, B);
    x += (A * x + B * delta) * dt;
}

void BicycleModel::step_rk4(State& x, double delta, double V, double dt) const {
    Eigen::Matrix2d A;
    Eigen::Vector2d B;
    compute_matrices(V, A, B);

    auto f = [&](const State& s) -> State { return A * s + B * delta; };

    State k1 = f(x);
    State k2 = f(x + 0.5 * dt * k1);
    State k3 = f(x + 0.5 * dt * k2);
    State k4 = f(x + dt * k3);

    x += (dt / 6.0) * (k1 + 2.0 * k2 + 2.0 * k3 + k4);
}

double BicycleModel::steady_state_yaw_rate(double delta, double V) const {
    double L = params_.wheelbase();
    double K_us = params_.understeer_gradient();
    return V * delta / (L + K_us * V * V);
}

double BicycleModel::steady_state_sideslip(double delta, double V) const {
    double L = params_.wheelbase();
    double K_us = params_.understeer_gradient();
    double Lr = params_.Lr;
    double Cr = params_.Cr;
    double m = params_.m;
    // beta_ss = (Lr / L - m * V^2 * Lf / (L^2 * Cr)) * delta / (1 + K_us * V^2 / L)
    double num = (Lr / L) - (m * V * V * params_.Lf) / (L * L * Cr);
    double den = 1.0 + K_us * V * V / L;
    return num / den * delta;
}

double BicycleModel::critical_speed() const {
    double K_us = params_.understeer_gradient();
    if (K_us >= 0.0) {
        return std::numeric_limits<double>::infinity();
    }
    double L = params_.wheelbase();
    return std::sqrt(-L / K_us);
}

Eigen::Vector2cd BicycleModel::eigenvalues(double V) const {
    Eigen::Matrix2d A;
    Eigen::Vector2d B;
    compute_matrices(V, A, B);

    Eigen::EigenSolver<Eigen::Matrix2d> solver(A, false);
    return solver.eigenvalues();
}

bool BicycleModel::is_stable(double V) const {
    auto eigs = eigenvalues(V);
    return eigs(0).real() < 0.0 && eigs(1).real() < 0.0;
}

}  // namespace caliburn
