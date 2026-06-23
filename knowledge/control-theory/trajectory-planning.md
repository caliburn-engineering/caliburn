---
title: Trajectory Planning
sources:
  - { book: "Biagiotti & Melchiorri - Trajectory Planning for Automatic Machines and Robots", chapter: "3-4" }
  - { note: "engineering experience" }
requires:
  - state-space.md
related:
  - controllers/pid.md
  - controllers/lqr.md
  - ../simulation/rigid-body/servo-modelling.md
reference: ../../../reference/controllers/trajectory_planner.h
---

# Trajectory Planning

A trajectory planner generates a smooth time-parameterised reference signal for a controller to track. Instead of commanding an instantaneous step change in setpoint (which demands infinite actuator bandwidth), the planner creates a feasible path from the current state to the target state that respects velocity, acceleration, and jerk limits.

In the ball-balancer, trajectory planning creates smooth ball position setpoint transitions — moving the target from (0,0) to (0.1, 0.05) over a defined time horizon rather than jumping instantaneously.

## Polynomial Trajectories

### Third-Order (Cubic) Polynomial

Given boundary conditions on position and velocity at start and end:

```
q(0) = q_0,    q_dot(0) = v_0
q(T) = q_f,    q_dot(T) = v_f
```

The trajectory is:

```
q(t) = a_0 + a_1*t + a_2*t^2 + a_3*t^3
```

Solving the 4 boundary conditions:

```
a_0 = q_0
a_1 = v_0
a_2 = (3*(q_f - q_0) - (2*v_0 + v_f)*T) / T^2
a_3 = (2*(q_0 - q_f) + (v_0 + v_f)*T) / T^3
```

### Fifth-Order (Quintic) Polynomial

Adds acceleration boundary conditions:

```
q(0) = q_0,  q_dot(0) = v_0,  q_ddot(0) = a_0
q(T) = q_f,  q_dot(T) = v_f,  q_ddot(T) = a_f
```

Six boundary conditions, six coefficients. The quintic provides continuous acceleration — important when the actuator's torque profile matters.

## Minimum-Jerk Trajectory

The minimum-jerk trajectory minimises the integral of squared jerk (third derivative of position):

```
J = integral(0, T, q_dddot(t)^2 dt)  →  min
```

For rest-to-rest motion (zero velocity and acceleration at both ends), the result is a 5th-order polynomial:

```
s(t) = 10*(t/T)^3 - 15*(t/T)^4 + 6*(t/T)^5
q(t) = q_0 + (q_f - q_0) * s(t)
```

where `s(t)` is a normalised S-curve from 0 to 1.

Properties:
- Velocity profile is bell-shaped (smooth ramp up and down)
- Acceleration is continuous and starts/ends at zero
- Jerk is continuous — minimises mechanical vibration
- Used in robotics and human motor control modelling (Flash & Hogan 1985)

For the ball-balancer, minimum-jerk is the default trajectory type: it produces the smoothest ball motion that won't excite oscillations in the plate dynamics.

## Trapezoidal Velocity Profile

A piecewise trajectory with three phases:

```
Phase 1 (accelerate):  0 ≤ t < t_a
    q_ddot = a_max
    v(t) = a_max * t

Phase 2 (cruise):      t_a ≤ t < T - t_a
    q_ddot = 0
    v(t) = v_max

Phase 3 (decelerate):  T - t_a ≤ t ≤ T
    q_ddot = -a_max
    v(t) = v_max - a_max * (t - (T - t_a))
```

The acceleration time is:

```
t_a = v_max / a_max
```

And the total time:

```
T = (q_f - q_0) / v_max + v_max / a_max
```

If the distance is too short to reach v_max, the cruise phase vanishes and the profile becomes triangular.

Properties:
- Time-optimal for given velocity and acceleration limits
- Velocity is continuous but acceleration is discontinuous (jumps at phase boundaries)
- Simple to compute and widely used in CNC and motor drives

## Waypoint Interpolation

For a sequence of waypoints [q_0, q_1, ..., q_n], generate a trajectory through all points with specified arrival times. Options:

1. **Segment-wise polynomials:** Independent cubic/quintic per segment, matching boundary conditions at each waypoint. Simple but velocity may be discontinuous at waypoints unless explicitly constrained.

2. **Cubic spline:** Fit a natural or clamped cubic spline through all waypoints. Produces C² continuity (continuous position, velocity, acceleration). More complex but smoother.

For the ball-balancer, segment-wise minimum-jerk with zero velocity at each waypoint is sufficient: the ball stops briefly at each setpoint before moving to the next.

## Implementation Notes

The reference implementation provides three trajectory generators:
- `CubicTrajectory` — cubic polynomial with position and velocity BCs
- `MinJerkTrajectory` — minimum-jerk 5th-order polynomial (rest-to-rest)
- `TrapezoidalTrajectory` — trapezoidal velocity profile with velocity and acceleration limits

All return position, velocity, and acceleration at a given time `t`. All are scalar (1D) — for multi-axis use, run one per axis independently.
