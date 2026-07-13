---
title: Tyre Dynamics — Force, Slip, and the Traction Circle
sources:
  - { book: "Pacejka — Tire and Vehicle Dynamics", chapter: "4" }
  - { book: "Rajamani — Vehicle Dynamics and Control", chapter: "2" }
requires: []
related:
  - bicycle-model.md
  - ../control-theory/controllers/sliding-mode.md
  - ../control-theory/controllers/pid.md
---

# Tyre Dynamics — Force, Slip, and the Traction Circle

The tyre is the only interface between vehicle and road. All forces that accelerate, brake, or turn the vehicle must pass through four small contact patches. Understanding the nonlinear relationship between slip and force is fundamental to every vehicle control system.

## Slip Definitions

### Longitudinal Slip

```
Braking:   lambda = (wR - v) / v
Driving:   lambda = (wR - v) / wR
```

where w = wheel angular velocity, R = effective rolling radius, v = vehicle longitudinal speed.

- lambda = 0: free rolling (no slip)
- lambda = -1: locked wheel (v > 0, w = 0)
- lambda = +1: spinning wheel (v = 0, w > 0)

### Lateral Slip Angle

```
alpha = arctan(v_lat / v_long)
```

The angle between the wheel's heading and its velocity vector. Small at normal driving (1-3 degrees), large during limit handling (8-12 degrees).

## Force-Slip Curve

The relationship between slip and generated force is nonlinear with a distinct peak:

```
Force
  ^
  |        .---.
  |       /     \
  |      /       ` - - - - - - - - (friction limit)
  |     /
  |    /
  |   / <- linear region (slope = cornering stiffness C)
  |  /
  | /
  +-----|-----------|-----------> Slip
      ~3%        ~10-20%
   (linear)      (peak)
```

**Key characteristics:**

| Region | Slip Range | Behaviour | Stability |
|---|---|---|---|
| Linear | 0 — ~3% (lateral: 0-3 deg) | F proportional to slip | Stable — self-correcting |
| Peak | ~10-20% (lateral: 8-12 deg) | Maximum force generated | Transition point |
| Saturated | Beyond peak | Force decreases with more slip | Unstable — runaway slip |

The fundamental nonlinearity: beyond the peak, applying more slip produces LESS force. This is why:
- ABS releases brake pressure when slip exceeds peak — more force with less braking
- TCS cuts engine torque when drive slip exceeds peak — more traction with less power
- ESP brakes individual wheels to stay within the stable region

## Traction Circle

The total horizontal force a tyre can generate is bounded by the friction limit:

```
Fx^2 + Fy^2 <= (mu * Fz)^2
```

where:
- Fx = longitudinal force (braking/driving)
- Fy = lateral force (cornering)
- mu = friction coefficient (~0.9 dry asphalt, ~0.3 wet, ~0.1 ice)
- Fz = normal load (weight on this tyre)

**Interpretation:** The tyre has a fixed "budget" of grip. Using grip for braking reduces what's available for cornering, and vice versa. This is why trail braking works — gradually releasing brakes as steering increases keeps the operating point near the friction circle boundary.

```
        Fy (lateral)
         ^
         |    .---.
         |   /     \
         |  |       |  <- friction circle (radius = mu * Fz)
         |   \     /
         |    '---'
    -----+-----------> Fx (longitudinal)
         |
```

## Pacejka Magic Formula

The industry-standard empirical tyre model:

```
F = D * sin(C * arctan(B*slip - E*(B*slip - arctan(B*slip))))
```

where:
- B = stiffness factor (controls initial slope)
- C = shape factor (controls peak location, typically 1.3-1.7)
- D = peak factor (= mu * Fz, the maximum force)
- E = curvature factor (controls shape beyond peak, -1 to +1)

### Physical meaning of each parameter:

| Parameter | Controls | Typical range |
|---|---|---|
| B | Initial slope (cornering stiffness) | 4-20 |
| C | Width of peak region | 1.0-1.7 |
| D | Peak force magnitude | mu * Fz |
| E | Post-peak shape | -2 to +1 |

### Simplified form for simulation:

For quick prototyping without full Pacejka parameterisation, a useful approximation:

```
F = mu * Fz * sin(C * arctan(slip / slip_peak))
```

where slip_peak is the slip ratio at peak force (~0.1 for longitudinal, ~8 degrees for lateral).

## Combined Slip

When both longitudinal and lateral slip are present simultaneously (e.g., braking while cornering), the forces interact through the friction circle constraint. The combined model:

1. Compute normalised slips: sigma_x = lambda / lambda_peak, sigma_y = alpha / alpha_peak
2. Compute combined slip magnitude: sigma = sqrt(sigma_x^2 + sigma_y^2)
3. Compute total force at combined slip using the Magic Formula
4. Distribute force by direction: Fx = F * sigma_x / sigma, Fy = F * sigma_y / sigma

## Normal Load Sensitivity

Tyre force does not scale linearly with normal load. The friction coefficient decreases as load increases:

```
mu(Fz) = mu_0 * (1 - k * Fz)    (simplified)
```

This means: transferring load from the inside wheel to the outside wheel during cornering causes a NET loss of lateral force — the loaded wheel gains less than the unloaded wheel loses. This is the fundamental reason why weight transfer reduces total grip.

## Implementation Notes

### Linear Tyre Model (Small Angles)

For controller design and observer synthesis, the linear approximation is sufficient:

```cpp
double Fy_front = Cf * alpha_f;  // Cf = front cornering stiffness [N/rad]
double Fy_rear  = Cr * alpha_r;  // Cr = rear cornering stiffness [N/rad]
```

Valid for |alpha| < 3-5 degrees. This is what the bicycle model uses internally.

### Nonlinear Model for Simulation

```cpp
struct PacejkaTyre {
    double B, C, D, E;

    double force(double slip) const {
        double Bs = B * slip;
        return D * std::sin(C * std::atan(Bs - E * (Bs - std::atan(Bs))));
    }

    double force_combined(double slip_x, double slip_y, double Fz) const {
        double sigma = std::sqrt(slip_x * slip_x + slip_y * slip_y);
        if (sigma < 1e-6) return 0.0;
        double F_total = force(sigma);
        // Return as [Fx, Fy] vector — caller extracts components
        return F_total;
    }
};
```

### Numerical Considerations

- At zero speed (v = 0), slip definitions divide by zero. Use a regularisation: lambda = (wR - v) / max(v, v_min) with v_min ~ 0.5 m/s.
- The force-slip curve has a discontinuous derivative at the peak. For gradient-based controllers, use a smoothed version or operate only in the linear region.
- Pacejka parameters are temperature and pressure dependent. For simulation, assume nominal conditions unless modelling thermal effects explicitly.
