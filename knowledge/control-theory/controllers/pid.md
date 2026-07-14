---
title: PID Control
sources:
  - { book: "Kreyszig - Advanced Engineering Mathematics", chapter: "4" }
  - { note: "engineering experience" }
requires:
  - ../state-space.md
related:
  - lqr.md
  - sliding-mode.md
  - ../steady-state-error.md
  - ../second-order-systems.md
  - ../frequency-response.md
  - ../design-problems/traction-control.md
reference: ../../../reference/controllers/pid.h
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

## Diagnostic Tuning Table

When observing closed-loop behaviour, use this table to diagnose and fix:

| Observed Symptom | Most Likely Cause | Recommended Action |
|---|---|---|
| Persistent steady-state offset | No I term, or Ki too low | Add/increase Ki |
| Excessive overshoot (>20%) | Kp too high, or insufficient damping | Increase Kd or decrease Kp |
| Sluggish response (slow rise) | Kp too low | Increase Kp |
| Sustained oscillation (constant amplitude) | At stability boundary | Reduce Kp, check phase margin |
| Growing oscillation | Unstable — gain too high | Reduce Kp immediately |
| Actuator chattering (rapid switching) | Derivative amplifying noise | Filter derivative (increase tau_f), or reduce Kd |
| Output saturates and stays stuck | Integrator windup | Add anti-windup (see above) |
| Oscillation only after setpoint change | Derivative kick | Switch to derivative-on-measurement |
| Good tracking but poor disturbance rejection | Insufficient loop gain at disturbance frequency | Increase Ki (for low-freq disturbance) or Kp (for mid-freq) |
| Response good at one operating point, bad at another | Plant nonlinearity (gain varies with state) | Gain scheduling — vary PID gains with operating point |

**Meta-rule:** If in doubt, measure the phase margin. PM < 30° means you're too aggressive.
PM > 70° means you're too conservative. Target 45-60° for a good balance.

## Steady-State Error and System Type

Adding the I term introduces a pole at s=0, increasing the system type by 1.
A type-1 system has zero steady-state error for step inputs (the Final Value Theorem guarantee).

For the complete theory of system types, error constants, and the error table
for different input types, see [Steady-State Error](../steady-state-error.md).

## Automotive PID Applications

PID is ubiquitous in vehicle control. These examples show how the same algorithm adapts to different domains:

| System | Controlled Variable | Controller Type | Key Notes |
|---|---|---|---|
| **Cruise control** | Vehicle speed | PI + feedforward | Feedforward for road grade (if inclinometer available). I for steady-state accuracy. D rarely used — vehicle dynamics are already well-damped. |
| **Idle speed control** | Engine RPM at idle | PI | Anti-windup critical — throttle actuator saturates at closed position. Load disturbances from A/C compressor, power steering pump. |
| **Electronic throttle** | Throttle plate angle | PID | Filtered D essential — throttle position sensor is noisy. Fast response needed (~50ms settling). |
| **Boost pressure (turbo)** | Intake manifold pressure | PI + gain scheduling | Wastegate actuator. Gains vary with engine RPM and load — the turbo's response characteristics change across the map. |
| **Coolant temperature** | Engine/battery temp | PI | D rarely used — thermal systems are slow and noisy. Long time constants (minutes). Tight anti-windup for heater/cooler on/off. |
| **Traction control** | Wheel slip ratio | PID (limited) | Fixed PID struggles — tyre curve is nonlinear (gain changes across operating point). Real systems use gain-scheduled PID or switch to nonlinear control (SMC). |
| **EGR valve** | Exhaust gas recirculation rate | PI | Position control of EGR valve against exhaust pressure disturbance. |
| **Fuel rail pressure** | Common rail pressure | PI | High-pressure system (2000+ bar). Fast dynamics, tight tolerance. |

### Common Automotive PID Patterns

1. **Feedforward + PI feedback:** Use known physics (gravity, drag, load) for the bulk of the control effort. PI handles the residual error. Reduces reliance on high gains.

2. **Gain scheduling:** When the plant gain changes with operating point (speed, RPM, temperature), vary PID gains accordingly. Implemented as lookup tables indexed by operating condition.

3. **Cascaded loops:** Outer loop (slower) sets the reference for an inner loop (faster). Example: outer = speed controller (PI), inner = torque controller (fast PI or feedforward).

4. **Anti-windup everywhere:** Automotive actuators saturate constantly (throttle limits, brake pressure limits, motor torque limits). Every I term needs anti-windup.

5. **Derivative filtering always:** Real sensors are noisy. No automotive PID uses raw d/dt. Typical filter: first-order with tau_f = T_sample * 3-10.

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

## Key Vocabulary (Interview-Ready)

| Term | One-liner Definition |
|---|---|
| System type | Number of free integrators in open-loop — determines which inputs give zero e_ss |
| Phase margin | How far from -180° the phase is at gain crossover — measures stability robustness |
| Gain margin | How much gain can increase before instability — measured at -180° phase crossover |
| Anti-windup | Mechanism to stop I-term growing when actuator is saturated |
| Gain scheduling | Varying controller gains based on operating point — for nonlinear plants |
| Feedforward | Open-loop correction from known disturbance — feedback handles the residual |
| Derivative kick | Infinite derivative spike from step setpoint change — fix: differentiate PV only |
| Back-calculation | Anti-windup method: undo integration when output saturates |
| Bumpless transfer | Ensuring smooth controller output when switching modes (manual→auto) |
