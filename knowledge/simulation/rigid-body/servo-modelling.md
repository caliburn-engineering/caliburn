---
title: Servo Motor Modelling
sources:
  - { book: "Autsou et al. - Principles and Methods of Servomotor Control", chapter: "3" }
  - { note: "engineering experience" }
requires:
  - equations-of-motion.md
  - ../../control-theory/state-space.md
related:
  - constraints-and-joints.md
  - ../../control-theory/controllers/pid.md
  - ../../simulation/sim-loop.md
reference: ../../../reference/simulation/servo_model.h
---

# Servo Motor Modelling

A servomotor is a closed-loop actuator that controls shaft angle with high accuracy. In the ball-balancer, two servos drive the plate tilt axes. For simulation, the servo is modelled as a **first-order dynamic system** — the simplest abstraction that captures the lag between commanded and actual angle.

## First-Order Transfer Function

The servo's position response to a commanded angle:

```
G(s) = theta_out(s) / theta_cmd(s) = K / (tau*s + 1)
```

| Parameter | Meaning | Typical value |
|---|---|---|
| K | Steady-state gain | ≈ 1.0 (position servo) |
| tau | Time constant | 0.05–0.15 s |

The time-domain step response:

```
theta(t) = K * theta_cmd * (1 - e^{-t/tau})
```

The servo reaches 63% of the command in one time constant, 95% in 3*tau, 99% in 5*tau.

### As an ODE

The first-order model is a single ODE:

```
tau * d(theta)/dt + theta = K * theta_cmd
```

Or equivalently:

```
d(theta)/dt = (K * theta_cmd - theta) / tau
```

This ODE is integrated alongside the plant dynamics in the simulation loop.

## Nonlinear Effects

### Velocity Saturation

At large commanded steps, the servo's angular rate hits a hardware limit:

```
|d(theta)/dt| <= omega_max
```

Typical values: 400–600 °/s (7–10.5 rad/s) for hobby servos. The first-order model alone doesn't enforce this — velocity clamping must be applied after computing the derivative.

### Dead Zone

Many hobby servos have a dead band of ±1–2° where no correction occurs:

```
if |theta_cmd - theta| < deadzone:
    effective_error = 0
else:
    effective_error = theta_cmd - theta - sign(theta_cmd - theta) * deadzone
```

### Backlash

Gearbox play creates hysteresis: the output shaft doesn't move until the gear teeth re-engage after a direction reversal. Modelled as a dead zone that activates only on direction change.

For the ball-balancer operating near level (small angles, no direction reversals during stable control), dead zone and backlash are negligible. Velocity saturation matters for large setpoint changes.

## PWM Interface

Standard hobby servo PWM:
- Frequency: 50 Hz (20 ms period)
- Pulse width: 1.0 ms → 0°, 1.5 ms → 90°, 2.0 ms → 180°
- Resolution: typically 1 µs ≈ 0.18°

```
theta_cmd = (pulse_width_ms - 1.0) * 180.0  [degrees]
```

In simulation, the PWM conversion is abstracted away — the controller outputs a desired angle directly, which becomes theta_cmd for the servo model.

## Cascade Control Architecture

The servo has its own internal PID loop (position feedback from a potentiometer or encoder). The ball-balancer's outer PID commands angle setpoints to the servo's inner loop:

```
Ball position error
  → [Outer PID] → theta_cmd
      → [Servo internal PID] → motor voltage
          → [Gearbox + motor] → theta_actual
              → [Ball physics] → ball position
```

For simulation, the servo's internal PID is abstracted into the first-order model — we don't simulate the motor voltage or current, just the resulting angular response.

## Implementation Notes

The reference implementation provides a `ServoModel` class that:
1. Computes the first-order response: `d(theta)/dt = (theta_cmd - theta) / tau`
2. Clamps velocity to `omega_max`
3. Clamps position to `[theta_min, theta_max]`
4. Optionally models dead zone

The `step(theta_cmd, dt)` method integrates one timestep and returns the new angle. It uses forward Euler internally (the servo dynamics are not stiff enough to need RK4).
