---
title: Rigid Body Constraints and Joint Limits
sources:
  - { book: "Rigid Body Dynamics (Borisov & Mamaev)", chapter: "1-2" }
  - { note: "engineering experience" }
requires:
  - equations-of-motion.md
related:
  - servo-modelling.md
  - ../../control-theory/stability.md
reference: ../../../reference/simulation/constraint_enforcer.h
---

# Rigid Body Constraints and Joint Limits

Real mechanisms have physical limits: servos can't rotate beyond their travel range, joints have hard stops, and actuators saturate. A simulation must enforce these constraints to prevent impossible poses and ensure the controller sees realistic dynamics.

## Types of Constraints

### Joint Limits

A revolute joint (hinge) has a range of motion:

```
theta_min <= theta <= theta_max
```

For the ball-balancer, each servo axis has a physical travel limit (typically ±90° maximum, ±30° operational). When the controller commands an angle beyond the limit, the joint clamps to the boundary.

### Velocity Limits

Servos have a maximum angular velocity (datasheet value, e.g. 600°/s):

```
|omega| <= omega_max
```

This is velocity saturation — even if the controller demands an instantaneous large step, the servo slews at its maximum rate.

### Actuator Saturation

The control signal itself may be bounded:

```
u_min <= u <= u_max
```

For PWM-controlled servos, the pulse width has a physical range (1–2 ms). Commands outside this range are clamped by the servo electronics.

## Enforcement Methods

### Clamping (Projection)

The simplest approach: after each integration step, project the state back onto the constraint surface.

```
theta = clamp(theta, theta_min, theta_max)
if theta was clamped:
    omega = 0  (or omega = -e * omega for elastic bounce)
```

**Pros:** Simple, robust, no solver needed.
**Cons:** Energy is not conserved (the clamp absorbs energy). Acceptable for servo-driven joints where the motor provides energy.

### Penalty Method

Add a stiff spring force that pushes back when the joint approaches its limit:

```
if theta > theta_max:
    F_penalty = -k_penalty * (theta - theta_max) - b_penalty * omega
```

**Pros:** Smooth force profile, differentiable (good for gradient-based optimisers).
**Cons:** Requires tuning k_penalty and b_penalty. Very stiff penalties require small timesteps.

### Lagrange Multipliers (Constraint Forces)

Solve for the constraint force that keeps the joint within limits. Used in physics engines (Bullet, ODE) but overkill for the ball-balancer's 2-DOF system.

## Ball-Balancer Application

The ball-balancer uses **clamping** for all constraints:

1. **Servo angle limits:** After computing the commanded angle from PID output, clamp to `[theta_min, theta_max]`. If clamped, set the angular velocity to zero (hard stop).

2. **Servo velocity limits:** After computing the desired angular velocity from the first-order servo model, clamp to `[-omega_max, omega_max]`.

3. **Ball boundary:** The ball can roll off the plate. When the ball position exceeds the plate half-width, the simulation flags it as "ball lost" rather than clamping (the ball doesn't bounce off the edge — it falls).

```
if |ball_x| > plate_half_width or |ball_y| > plate_half_width:
    ball_on_plate = false
```

### Constraint Order in the Loop

```
1. Controller outputs commanded angle θ_cmd
2. Clamp θ_cmd to servo limits                     [actuator saturation]
3. Servo model computes θ_dot from (θ_cmd - θ)     [first-order dynamics]
4. Clamp θ_dot to velocity limit                   [velocity saturation]
5. Integrate: θ += θ_dot * dt
6. Clamp θ to joint limits (safety)                [position limit]
7. Compute ball dynamics with constrained θ
8. Check ball boundary
```

## Implementation Notes

The reference implementation provides a `ConstraintEnforcer` class that applies positional clamping with optional velocity zeroing. It works on Eigen vectors so it can constrain any state vector dimension.

Key design: constraints are data (min/max pairs), not code. The enforcer takes a vector of limits and applies them uniformly. This makes it easy to add or change constraints without modifying the enforcer.
