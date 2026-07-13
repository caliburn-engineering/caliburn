#include "tyre_model.h"

#include <cmath>
#include <algorithm>

namespace caliburn {

// --- LinearTyre ---

LinearTyre::LinearTyre(double cornering_stiffness) : C_alpha_(cornering_stiffness) {}

double LinearTyre::lateral_force(double alpha) const {
    return C_alpha_ * alpha;
}

// --- PacejkaTyre ---

PacejkaTyre::PacejkaTyre(const PacejkaParams& params) : params_(params) {}

double PacejkaTyre::force(double slip) const {
    double Bs = params_.B * slip;
    double inner = Bs - params_.E * (Bs - std::atan(Bs));
    return params_.D * std::sin(params_.C * std::atan(inner));
}

double PacejkaTyre::force_derivative(double slip) const {
    // Numerical derivative (central difference)
    constexpr double h = 1e-6;
    return (force(slip + h) - force(slip - h)) / (2.0 * h);
}

double PacejkaTyre::peak_slip() const {
    // Binary search for peak (force derivative crosses zero)
    double lo = 0.0;
    double hi = 1.0;  // Upper bound for slip ratio

    for (int i = 0; i < 50; ++i) {
        double mid = (lo + hi) / 2.0;
        if (force_derivative(mid) > 0.0) {
            lo = mid;
        } else {
            hi = mid;
        }
    }
    return (lo + hi) / 2.0;
}

double PacejkaTyre::peak_force() const {
    return force(peak_slip());
}

// --- TractionCircle ---

TractionCircle::TractionCircle(double mu) : mu_(mu) {}

bool TractionCircle::is_within(double Fx, double Fy, double Fz) const {
    double F_max = mu_ * Fz;
    return (Fx * Fx + Fy * Fy) <= (F_max * F_max);
}

void TractionCircle::saturate(double& Fx, double& Fy, double Fz) const {
    double F_max = mu_ * Fz;
    double F_mag = std::sqrt(Fx * Fx + Fy * Fy);
    if (F_mag > F_max && F_mag > 1e-10) {
        double scale = F_max / F_mag;
        Fx *= scale;
        Fy *= scale;
    }
}

double TractionCircle::max_force(double Fz, double ratio) const {
    return mu_ * Fz;  // The circle radius — actual component depends on direction
}

// --- CombinedSlipTyre ---

CombinedSlipTyre::CombinedSlipTyre(const PacejkaParams& long_params,
                                   const PacejkaParams& lat_params,
                                   double mu)
    : long_tyre_(long_params), lat_tyre_(lat_params), circle_(mu) {}

void CombinedSlipTyre::forces(double slip_x, double slip_y, double Fz,
                              double& Fx, double& Fy) const {
    // Normalise slips by their respective peak values
    double peak_x = long_tyre_.peak_slip();
    double peak_y = lat_tyre_.peak_slip();

    double sigma_x = slip_x / peak_x;
    double sigma_y = slip_y / peak_y;
    double sigma = std::sqrt(sigma_x * sigma_x + sigma_y * sigma_y);

    if (sigma < 1e-8) {
        Fx = 0.0;
        Fy = 0.0;
        return;
    }

    // Total force at combined slip magnitude (using longitudinal curve as reference)
    // Scale D parameter with normal load
    double F_total = long_tyre_.force(sigma * peak_x);

    // Distribute by direction
    Fx = F_total * (sigma_x / sigma);
    Fy = F_total * (sigma_y / sigma);

    // Enforce traction circle
    circle_.saturate(Fx, Fy, Fz);
}

}  // namespace caliburn
