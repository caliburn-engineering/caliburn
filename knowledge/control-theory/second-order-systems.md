---
title: Second-Order System Dynamics & Time-Domain Specifications
sources:
  - { book: "Ogata - Modern Control Engineering", chapter: "5", pages: "200-230" }
  - { book: "Franklin, Powell, Emami-Naeini - Feedback Control", chapter: "3" }
requires:
  - state-space.md
  - ../math/linear-algebra/eigenvalues.md
related:
  - stability.md
  - frequency-response.md
  - controllers/lqr.md
---

# Second-Order System Dynamics & Time-Domain Specifications

The standard second-order system is the workhorse model for understanding dynamic behaviour. Every controller design starts with: "what's the natural frequency? what's the damping?" Pole location directly maps to physical behaviour — overshoot percentage, settling time, rise time — giving engineers concrete numbers to specify before choosing a control law.

## Standard Form

```
G(s) = omega_n^2 / (s^2 + 2*zeta*omega_n*s + omega_n^2)

Poles: s = -zeta*omega_n +/- omega_n*sqrt(zeta^2 - 1)

For 0 < zeta < 1 (underdamped):
  s = -sigma +/- j*omega_d
  where sigma = zeta*omega_n (decay rate)
        omega_d = omega_n*sqrt(1 - zeta^2) (damped frequency)
```

## Time-Domain Specifications

| Spec | Formula | Depends on |
|---|---|---|
| **Overshoot** | %OS = 100 * exp(-pi*zeta / sqrt(1-zeta^2)) | zeta only |
| **Settling time** (2%) | t_s = 4 / (zeta*omega_n) = 4/sigma | sigma = zeta*omega_n |
| **Rise time** (10-90%) | t_r = 1.8 / omega_n | omega_n (approximate) |
| **Peak time** | t_p = pi / omega_d = pi / (omega_n*sqrt(1-zeta^2)) | omega_d |
| **Steady-state** | Final value = G(0) = 1 (for this standard form) | DC gain |

## Overshoot vs Damping Table

| Damping zeta | Overshoot % | Phase Margin (approx) | Character |
|---|---|---|---|
| 0.1 | 73% | ~10 deg | Highly oscillatory |
| 0.2 | 53% | ~20 deg | Very oscillatory |
| 0.3 | 37% | ~30 deg | Oscillatory |
| 0.4 | 25% | ~40 deg | Moderately oscillatory |
| 0.5 | 16% | ~50 deg | Moderate |
| 0.6 | 10% | ~60 deg | Well-damped |
| 0.7 | 5% | ~70 deg | Critically well-damped (typical target) |
| 0.8 | 1.5% | ~80 deg | Overdamped-ish |
| 1.0 | 0% | 90 deg | Critically damped (no oscillation) |

**Rough formula:** zeta ~ PM/100 (for PM 30-70 deg, not exact but useful for quick estimates)

## Pole Location to Behaviour Mapping

```
Im(s)
  ^
  |     X = pole location
  |
  |   /  <- constant zeta line (angle = acos(zeta))
  |  /
  | /  X (zeta=0.7, omega_n=10)
  |/____________________> Re(s)
  |\
  | \  X (conjugate)
  |  \
  |

  sigma = zeta*omega_n = distance from imaginary axis
  omega_d = omega_n*sqrt(1-zeta^2) = imaginary part
  omega_n = distance from origin to pole
  zeta = cos(angle from negative real axis)
```

**Design rule:** Move poles LEFT for faster response (larger omega_n). Move poles toward the real axis for less overshoot (larger zeta). Moving left costs control effort.

## Dominant Poles Approximation

For higher-order systems: if one pair of complex poles is "closest" to the imaginary axis (slowest to decay), the system response is approximately second-order with those poles determining the specs.

**Rule of thumb:** A pole dominates if all other poles are at least 5x further from the imaginary axis.

This is why second-order analysis applies to real (higher-order) systems — you design for the dominant pair.

## Connection to Controller Design

| Spec | What you set | Controller implication |
|---|---|---|
| omega_n (speed) | Where to place poles | Higher LQR Q -> larger omega_n -> faster |
| zeta (damping) | How oscillatory | Higher LQR Q on velocity states -> more damping |
| Settling time | 4/(zeta*omega_n) | Directly from pole placement |
| Overshoot constraint | Minimum zeta | Constrains the angle of desired poles |

## Automotive Context

- **Cruise control:** omega_n ~ 0.5-1 rad/s, zeta ~ 0.8-1.0 (slow, well-damped, comfortable)
- **Throttle control:** omega_n ~ 20-50 rad/s, zeta ~ 0.7 (fast, minimal overshoot)
- **Active suspension:** omega_n ~ 3-10 rad/s, zeta ~ 0.5-0.7 (balance between isolation and control)
- **EPS assist:** omega_n ~ 10-30 rad/s, zeta ~ 0.7-0.9 (responsive but smooth steering feel)

## Implementation Notes

### Computing Time-Domain Specs from Pole Location

```cpp
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

// Inverse: from specs, what poles do I need?
struct PoleRequirements {
    double zeta;
    double omega_n;
    std::complex<double> pole;  // -sigma + j*omega_d
};

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

// Check if a pair of poles satisfies specs
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
```

### Numerical Considerations

- The overshoot formula is only valid for underdamped systems (0 < zeta < 1). For zeta >= 1, overshoot is zero by definition.
- The settling time approximation (4/sigma for 2% criterion) assumes a pure second-order system. Higher-order zeros can increase actual settling time.
- Rise time approximation (1.8/omega_n) is a rough fit for 0.3 < zeta < 0.8. More precise formulas exist but this is adequate for initial design.
