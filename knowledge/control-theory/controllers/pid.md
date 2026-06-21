---
title: PID Control
sources:
  - { book: "Kreyszig - Advanced Engineering Mathematics", chapter: "4" }
  - { note: "engineering experience" }
requires:
  - ../state-space.md
related:
  - controllers/lqr.md
---

# PID Control

The Proportional-Integral-Derivative controller is the most widely deployed feedback controller in industry. It computes a control signal from the tracking error e(t) = setpoint - measurement using three complementary terms. PID is the default choice for SISO loops where a plant model is unavailable or approximate, and is the primary controller in the ball-balancer project for dual-axis plate tilt control.

## Key Equations

### Time-Domain Control Law

```
u(t) = Kp * e(t) + Ki * integral(e(tau) dtau, 0, t) + Kd * de(t)/dt
```

| Term | Effect | Limitation |
|---|---|---|
| **P** = Kp * e(t) | Immediate proportional response to error | Alone, produces steady-state offset |
| **I** = Ki * integral(e) | Eliminates steady-state error by accumulating past error | Can wind up when actuator saturates |
| **D** = Kd * de/dt | Damps oscillation by responding to rate of change | Amplifies high-frequency sensor noise |

### Transfer Function (Laplace Domain)

```
G_pid(s) = Kp + Ki/s + Kd*s = (Kd*s^2 + Kp*s + Ki) / s
```

The integral term introduces a pole at s = 0 (type-1 system), guaranteeing zero steady-state error for step inputs.

### Closed-Loop Transfer Function

For plant H(s) with unity feedback:

```
T(s) = G_pid(s) * H(s) / (1 + G_pid(s) * H(s))
```

## Anti-Windup Strategies

Integral windup occurs when the controller output saturates (e.g., servo angle limits) but the integral term continues accumulating. The integrator "winds up" to a large value and the controller remains saturated long after the error reverses sign.

### 1. Clamping (Integral Limits)

Compute integral bounds from the output range and integral gain:

```cpp
integral_min_ = output_min_ / gains_.Ki;
integral_max_ = output_max_ / gains_.Ki;

// After integration step:
integral_ = std::clamp(integral_, integral_min_, integral_max_);
```

### 2. Back-Calculation

Undo the most recent integration step when output saturates:

```cpp
double output_unsat = P + I + D;
double output = std::clamp(output_unsat, output_min_, output_max_);

if (output != output_unsat) {
    integral_ -= error * dt_;  // back out the step that caused saturation
}
```

Back-calculation is more responsive than clamping because it reacts immediately to saturation events rather than waiting for the integral to hit a fixed bound.

### 3. Conditional Integration

Only integrate when the error is within a threshold or the output is not saturated:

```cpp
bool saturated = (output == output_min_) || (output == output_max_);
bool error_same_sign_as_output = (error * output) > 0.0;

if (!(saturated && error_same_sign_as_output)) {
    integral_ += error * dt_;
}
```

## Derivative on Measurement (Not Error)

Computing the derivative from the error signal causes a **derivative kick** on setpoint step changes: the error jumps instantaneously, producing an infinite derivative spike. Instead, differentiate the measurement:

```cpp
// D = -Kd * d(measurement)/dt
double derivative = -(measurement - prev_measurement_) / dt_;
```

The negation compensates for the sign flip: d(error)/dt = d(setpoint)/dt - d(measurement)/dt, and d(setpoint)/dt = 0 between step changes.

## Derivative Low-Pass Filter

Raw differentiation amplifies sensor noise. Apply a first-order exponential filter:

```cpp
// alpha in (0, 1]: smaller = heavier filtering. 0.1 is a good default.
derivative_filtered_ = alpha * derivative + (1.0 - alpha) * derivative_filtered_;
double D = gains_.Kd * derivative_filtered_;
```

The filter time constant tau_f = dt_ * (1 - alpha) / alpha. For the ball-balancer at dt = 0.01s and alpha = 0.1, tau_f = 0.09s.

## Tuning Methods

### Ziegler-Nichols (Closed-Loop)

1. Set Ki = 0, Kd = 0
2. Increase Kp until the system oscillates with constant amplitude at critical gain Ku
3. Measure the oscillation period Tu
4. Apply the table:

| Controller | Kp | Ti = 1/Ki | Td = Kd/Kp |
|---|---|---|---|
| P only | 0.5 * Ku | - | - |
| PI | 0.45 * Ku | Tu / 1.2 | - |
| PID | 0.6 * Ku | Tu / 2 | Tu / 8 |

Ziegler-Nichols tends to produce aggressive, oscillatory responses (quarter-decay ratio). For the ball-balancer, start from ZN values and reduce Kp by 30-50% for smoother convergence.

### Manual Tuning Heuristic

1. Start with Kp only. Increase until response is fast but oscillatory.
2. Add Kd to damp oscillation. Typical starting ratio: Kd/Kp = 0.1 * dt_sample.
3. Add Ki last to eliminate steady-state error. Start small: Ki = Kp / (10 * T_settle).

## Implementation Notes

### Ball-Balancer Dual-Axis Coupling

Physical axes are swapped in the ball-balancer: tilting around the Y-axis moves the ball in the X direction, and vice versa. Each axis gets its own independent PID instance:

```cpp
// X position controlled by Y-axis tilt (theta_y)
double theta_y_cmd = pid_x_.compute(setpoint.x(), x);

// Y position controlled by X-axis tilt (varphi_x)
double varphi_x_cmd = pid_y_.compute(setpoint.y(), y);
```

### Eigen Usage

PID is inherently scalar (one loop per axis), so Eigen is not required. However, for vectorised multi-axis PID:

```cpp
Eigen::Vector2d error = setpoint - measurement;
Eigen::Vector2d integral = integral_prev + error * dt;
Eigen::Vector2d derivative = (error - error_prev) / dt;
Eigen::Vector2d output = Kp.cwiseProduct(error)
                       + Ki.cwiseProduct(integral)
                       + Kd.cwiseProduct(derivative);
```

### Numerical Stability

- Use `double` precision for the integral accumulator to avoid truncation over long runs.
- Reset the integrator on mode changes (e.g., switching from manual to auto).
- Guard against dt = 0 (division by zero in derivative computation).
- Limit dt to a sane range (e.g., 0.001 to 0.1 seconds) to reject spurious timer glitches.
