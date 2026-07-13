#pragma once

#include <cmath>
#include <algorithm>

namespace caliburn {

/// Pacejka Magic Formula tyre model parameters
struct PacejkaParams {
    double B;  // Stiffness factor (4-20)
    double C;  // Shape factor (1.0-1.7)
    double D;  // Peak factor (= mu * Fz)
    double E;  // Curvature factor (-2 to +1)
};

/// Linear tyre model — valid for small slip angles (<5 degrees)
class LinearTyre {
public:
    explicit LinearTyre(double cornering_stiffness);

    /// Lateral force from slip angle
    /// @param alpha  Slip angle [rad]
    /// @return Lateral force [N]
    double lateral_force(double alpha) const;

    double stiffness() const { return C_alpha_; }

private:
    double C_alpha_;  // Cornering stiffness [N/rad]
};

/// Pacejka Magic Formula tyre model — full nonlinear force-slip curve
class PacejkaTyre {
public:
    explicit PacejkaTyre(const PacejkaParams& params);

    /// Force from slip (generic — works for longitudinal or lateral)
    /// @param slip  Slip ratio (longitudinal) or slip angle [rad] (lateral)
    /// @return Force [N]
    double force(double slip) const;

    /// Force derivative (useful for linearisation and controller design)
    double force_derivative(double slip) const;

    /// Find the slip value at peak force
    double peak_slip() const;

    /// Peak force magnitude
    double peak_force() const;

    const PacejkaParams& params() const { return params_; }

private:
    PacejkaParams params_;
};

/// Traction circle constraint
/// Checks if combined Fx, Fy is within the friction limit
class TractionCircle {
public:
    /// @param mu  Friction coefficient
    TractionCircle(double mu);

    /// Check if forces are within the friction circle
    /// @param Fx  Longitudinal force [N]
    /// @param Fy  Lateral force [N]
    /// @param Fz  Normal load [N]
    bool is_within(double Fx, double Fy, double Fz) const;

    /// Saturate forces to lie on or within the friction circle
    /// Preserves direction, scales magnitude if needed
    void saturate(double& Fx, double& Fy, double Fz) const;

    /// Maximum force available in a given direction
    /// @param ratio  Fx / F_total (0 = pure lateral, 1 = pure longitudinal)
    double max_force(double Fz, double ratio) const;

    double mu() const { return mu_; }

private:
    double mu_;
};

/// Combined slip tyre model — integrates Pacejka with traction circle
class CombinedSlipTyre {
public:
    CombinedSlipTyre(const PacejkaParams& long_params,
                     const PacejkaParams& lat_params,
                     double mu);

    /// Compute combined longitudinal and lateral forces
    /// @param slip_x     Longitudinal slip ratio [-1, 1]
    /// @param slip_y     Lateral slip angle [rad]
    /// @param Fz         Normal load [N]
    /// @param[out] Fx    Resulting longitudinal force
    /// @param[out] Fy    Resulting lateral force
    void forces(double slip_x, double slip_y, double Fz,
                double& Fx, double& Fy) const;

private:
    PacejkaTyre long_tyre_;
    PacejkaTyre lat_tyre_;
    TractionCircle circle_;
};

}  // namespace caliburn
