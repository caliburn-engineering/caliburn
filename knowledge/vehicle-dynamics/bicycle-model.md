---
title: Bicycle Model — Lateral Vehicle Dynamics
sources:
  - { book: "Rajamani — Vehicle Dynamics and Control", chapter: "3" }
requires:
  - ../control-theory/state-space.md
  - tyre-dynamics.md
related:
  - understeer-oversteer.md
  - weight-transfer.md
reference: ../../../reference/vehicle-dynamics/bicycle_model.h
---

# Bicycle Model — Lateral Vehicle Dynamics

The bicycle model is the standard reduced-order model for lateral vehicle dynamics. It collapses a four-wheeled vehicle into two tyres (front and rear) on the centreline, yielding a 2-state linear system that captures yaw rate and sideslip behavior. This is the model inside ESP/ESC controllers — it runs in real-time on ECUs and provides the reference yaw rate for stability control.

## Assumptions

1. Small angles (sin(theta) ~ theta, cos(theta) ~ 1)
2. Linear tyre model (F = C * alpha, valid below ~5 degrees slip)
3. Constant longitudinal speed V (lateral/yaw dynamics decoupled from longitudinal)
4. No roll dynamics (planar model)
5. Left and right tyres lumped into single front/rear axles

## State-Space Formulation

### States and Input

```
States:  x = [v, r]^T      v = lateral velocity [m/s], r = yaw rate [rad/s]
Input:   u = delta           steering angle [rad]
```

### Equations of Motion

```
m * v_dot  = Fyf + Fyr - m*V*r         (lateral force balance)
Iz * r_dot = Lf*Fyf - Lr*Fyr           (yaw moment balance)
```

### Tyre Forces (Linear Model)

```
Fyf = Cf * alpha_f       (front lateral force)
Fyr = Cr * alpha_r       (rear lateral force)

alpha_f = delta - (v + Lf*r) / V    (front slip angle)
alpha_r = -(v - Lr*r) / V           (rear slip angle)
```

### Standard Form: x_dot = A*x + B*u

```
A = [ -(Cf+Cr)/(m*V)        -V - (Lf*Cf - Lr*Cr)/(m*V)  ]
    [ -(Lf*Cf - Lr*Cr)/(Iz*V)   -(Lf^2*Cf + Lr^2*Cr)/(Iz*V) ]

B = [ Cf/m       ]
    [ Lf*Cf/Iz   ]
```

### Parameters

| Symbol | Meaning | Typical (passenger car) |
|---|---|---|
| m | Vehicle mass | 1500 kg |
| Iz | Yaw inertia | 2500 kg*m^2 |
| Lf | CG to front axle | 1.2 m |
| Lr | CG to rear axle | 1.4 m |
| L = Lf + Lr | Wheelbase | 2.6 m |
| Cf | Front cornering stiffness | 80,000 N/rad |
| Cr | Rear cornering stiffness | 80,000 N/rad |
| V | Forward speed | varies |

## Steady-State Analysis

At steady state (v_dot = 0, r_dot = 0), the yaw rate for a given steering input is:

```
r_ss = V * delta / (L + K_us * V^2)
```

where the **understeer gradient** is:

```
K_us = m / L * (Lr/Cf - Lf/Cr)     [rad / (m/s^2)]
```

### Stability Interpretation

| Condition | K_us | Behaviour |
|---|---|---|
| K_us > 0 | Understeer | Stable — larger steering needed at higher speed |
| K_us = 0 | Neutral steer | Yaw rate proportional to V (Ackermann) |
| K_us < 0 | Oversteer | Unstable above critical speed V_crit |

**Critical speed** (oversteer only):

```
V_crit = sqrt(-L / K_us)
```

Above this speed, the open-loop system is unstable — small perturbations grow exponentially. ESP is required for safe operation.

## Eigenvalue Analysis

The A matrix eigenvalues determine the dynamic stability at a given speed:

```
Characteristic equation: lambda^2 - tr(A)*lambda + det(A) = 0
```

- Both eigenvalues have negative real parts: stable
- As V increases, eigenvalues move toward the imaginary axis
- For an oversteer vehicle, one eigenvalue crosses into the right half-plane at V_crit

The natural frequency and damping of the yaw/sideslip modes can be read directly from the eigenvalues. Typical passenger cars are overdamped at low speed and become underdamped at highway speeds.

## Outputs of Interest

### Sideslip Angle at CG

```
beta = v / V    [rad]
```

Not directly measurable — must be estimated (see observer design). ESP uses a sideslip observer combining IMU yaw rate with wheel speed signals.

### Lateral Acceleration

```
a_y = V * r + v_dot ≈ V * r    (at steady state)
```

Measurable via accelerometer. Used as a feedback signal in many stability systems.

## Connection to ESP/ESC

The ESP controller uses the bicycle model as an internal reference:

1. **Desired yaw rate:** r_des = V * delta / (L + K_us * V^2), clamped by friction limit
2. **Measured yaw rate:** from gyroscope
3. **Error:** e = r_measured - r_des
4. **Correction:** apply differential braking to generate a corrective yaw moment

If |e| exceeds threshold:
- Oversteer (|r| too large): brake outer front wheel — creates stabilising moment
- Understeer (|r| too small): brake inner rear wheel — tightens the trajectory

## Implementation Notes

### C++ State-Space Simulation

```cpp
struct BicycleModel {
    double m, Iz, Lf, Lr, Cf, Cr;

    void compute_matrices(double V, Eigen::Matrix2d& A, Eigen::Vector2d& B) const {
        double L = Lf + Lr;
        A(0, 0) = -(Cf + Cr) / (m * V);
        A(0, 1) = -V - (Lf * Cf - Lr * Cr) / (m * V);
        A(1, 0) = -(Lf * Cf - Lr * Cr) / (Iz * V);
        A(1, 1) = -(Lf * Lf * Cf + Lr * Lr * Cr) / (Iz * V);

        B(0) = Cf / m;
        B(1) = Lf * Cf / Iz;
    }

    // Euler step (for RK4, see simulation/rk4.md)
    void step(Eigen::Vector2d& x, double delta, double V, double dt) const {
        Eigen::Matrix2d A;
        Eigen::Vector2d B;
        compute_matrices(V, A, B);
        x += (A * x + B * delta) * dt;
    }

    double steady_state_yaw_rate(double delta, double V) const {
        double L = Lf + Lr;
        double K_us = (m / L) * (Lr / Cf - Lf / Cr);
        return V * delta / (L + K_us * V * V);
    }

    double understeer_gradient() const {
        double L = Lf + Lr;
        return (m / L) * (Lr / Cf - Lf / Cr);
    }
};
```

### Test Cases

1. **Steady-state cornering:** Apply constant delta, run until convergence, verify r matches analytical r_ss
2. **Step steer:** Apply sudden delta, verify oscillation frequency matches eigenvalues
3. **Speed sweep:** Compute eigenvalues at V = 10, 20, ..., 60 m/s, verify stability boundary matches V_crit for oversteer parameters
