#pragma once

#include <Eigen/Dense>
#include <cmath>

namespace caliburn {

/// Vehicle parameters for the bicycle model
struct VehicleParams {
    double m;    // Mass [kg]
    double Iz;   // Yaw moment of inertia [kg*m^2]
    double Lf;   // CG to front axle [m]
    double Lr;   // CG to rear axle [m]
    double Cf;   // Front cornering stiffness [N/rad]
    double Cr;   // Rear cornering stiffness [N/rad]

    double wheelbase() const { return Lf + Lr; }
    double understeer_gradient() const {
        double L = wheelbase();
        return (m / L) * (Lr / Cf - Lf / Cr);
    }
};

/// 2-state bicycle model for lateral vehicle dynamics
/// States: [v (lateral velocity), r (yaw rate)]
/// Input:  delta (steering angle)
class BicycleModel {
public:
    using State = Eigen::Vector2d;

    explicit BicycleModel(const VehicleParams& params);

    /// Compute state-space matrices A and B at given longitudinal speed V
    void compute_matrices(double V, Eigen::Matrix2d& A, Eigen::Vector2d& B) const;

    /// Simulate one timestep using Euler integration
    /// @param x   Current state [v, r] — modified in place
    /// @param delta  Steering angle [rad]
    /// @param V      Longitudinal speed [m/s]
    /// @param dt     Timestep [s]
    void step_euler(State& x, double delta, double V, double dt) const;

    /// Simulate one timestep using RK4 integration
    void step_rk4(State& x, double delta, double V, double dt) const;

    /// Steady-state yaw rate for given steering angle and speed
    double steady_state_yaw_rate(double delta, double V) const;

    /// Steady-state sideslip angle at CG
    double steady_state_sideslip(double delta, double V) const;

    /// Critical speed (only meaningful for oversteer vehicles, K_us < 0)
    /// Returns infinity for understeer vehicles
    double critical_speed() const;

    /// Compute eigenvalues of A matrix at given speed
    Eigen::Vector2cd eigenvalues(double V) const;

    /// Check if the system is stable at given speed
    bool is_stable(double V) const;

    const VehicleParams& params() const { return params_; }

private:
    VehicleParams params_;
};

}  // namespace caliburn
