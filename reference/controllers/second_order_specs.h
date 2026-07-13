/**
 * second_order_specs.h — Second-order system time-domain specification utilities
 *
 * Computes overshoot, settling time, rise time, peak time from pole locations (zeta, omega_n).
 * Also provides the inverse: given performance specs, compute required pole locations.
 *
 * Reference: knowledge/control-theory/second-order-systems.md
 */

#pragma once

#include <cmath>
#include <complex>

namespace caliburn {

struct TimeSpecs {
    double overshoot_percent;
    double settling_time_2pct;
    double rise_time;
    double peak_time;
    double damped_frequency;
};

/// Compute time-domain specs from damping ratio and natural frequency
inline TimeSpecs computeSpecs(double zeta, double omega_n) {
    double sigma = zeta * omega_n;
    double omega_d = omega_n * std::sqrt(1.0 - zeta * zeta);

    TimeSpecs specs;
    specs.overshoot_percent = 100.0 * std::exp(-M_PI * zeta / std::sqrt(1.0 - zeta * zeta));
    specs.settling_time_2pct = 4.0 / sigma;
    specs.rise_time = 1.8 / omega_n;
    specs.peak_time = M_PI / omega_d;
    specs.damped_frequency = omega_d;
    return specs;
}

/// Required pole locations to achieve given performance specifications
struct PoleRequirements {
    double zeta;
    double omega_n;
    std::complex<double> pole;  // -sigma + j*omega_d
};

/// Inverse problem: from max overshoot and max settling time, compute required poles
inline PoleRequirements polesFromSpecs(double max_overshoot_percent,
                                       double max_settling_time) {
    // From overshoot: solve %OS = 100*exp(-pi*zeta/sqrt(1-zeta^2)) for zeta
    double ln_os = std::log(max_overshoot_percent / 100.0);
    double zeta = -ln_os / std::sqrt(M_PI * M_PI + ln_os * ln_os);

    // From settling time: omega_n = 4 / (zeta * t_s)
    double omega_n = 4.0 / (zeta * max_settling_time);

    double sigma = zeta * omega_n;
    double omega_d = omega_n * std::sqrt(1.0 - zeta * zeta);

    PoleRequirements req;
    req.zeta = zeta;
    req.omega_n = omega_n;
    req.pole = std::complex<double>(-sigma, omega_d);
    return req;
}

/// Check if a pair of poles satisfies given overshoot and settling time specs
inline bool meetsSpecs(std::complex<double> pole,
                       double max_os, double max_ts) {
    double sigma = -pole.real();
    double omega_d = std::abs(pole.imag());
    double omega_n = std::sqrt(sigma * sigma + omega_d * omega_d);
    double zeta = sigma / omega_n;

    double actual_os = 100.0 * std::exp(-M_PI * zeta / std::sqrt(1.0 - zeta * zeta));
    double actual_ts = 4.0 / sigma;

    return (actual_os <= max_os) && (actual_ts <= max_ts);
}

} // namespace caliburn
