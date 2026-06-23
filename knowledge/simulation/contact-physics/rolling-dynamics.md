---
title: Contact Physics and Rolling Dynamics
sources:
  - { book: "Rigid Body Dynamics (Borisov & Mamaev)", chapter: "6" }
  - { note: "engineering experience" }
requires:
  - ../rigid-body/equations-of-motion.md
  - ../rigid-body/inertia-tensors.md
  - ../../math/transforms/homogeneous-transforms.md
related:
  - ../rigid-body/constraints-and-joints.md
  - ../../control-theory/controllers/pid.md
reference: ../../../reference/simulation/rolling_dynamics.h
---

# Contact Physics and Rolling Dynamics

The ball-balancer's core physics is a ball rolling on a tilted plate under gravity. This file covers the forces, constraints, and dynamics of that interaction.

## Gravity Decomposition on a Tilted Surface

When a plate is tilted by angles alpha (about x-axis) and beta (about y-axis), the gravitational acceleration component along the plate surface drives the ball's motion.

For small angles (linearised):

```
a_x = g * sin(beta)  ≈  g * beta
a_y = g * sin(alpha)  ≈  g * alpha
```

The normal force keeps the ball on the plate:

```
F_normal = m * g * cos(alpha) * cos(beta)  ≈  m * g   (small angles)
```

For large angles, the full rotation matrix must be used to decompose gravity into plate-frame components (see `homogeneous-transforms.md`).

## Rolling Without Slipping

A uniform sphere rolling without slipping on a surface has the constraint:

```
v_contact = v_CoM + omega × r_contact = 0
```

where `r_contact` points from the centre of mass to the contact point (downward by radius R for a sphere on a flat surface).

This constraint couples translational and rotational motion:

```
v_x = R * omega_y
v_y = -R * omega_x
```

### Effective Acceleration

For a uniform solid sphere (I = 2/5 * m * R²), the rolling constraint modifies the effective translational acceleration:

```
a_eff = F / (m + I/R²) = F / (m * (1 + 2/5)) = F / (7/5 * m) = (5/7) * F/m
```

So the ball accelerates at (5/7) of what a point mass would:

```
a_ball_x = (5/7) * g * sin(beta)
a_ball_y = (5/7) * g * sin(alpha)
```

The 5/7 factor is a direct consequence of the moment of inertia of a uniform sphere — energy goes into rotation as well as translation.

## Slip Condition

Rolling without slipping holds as long as the friction force doesn't exceed the static friction limit:

```
F_friction <= mu_s * F_normal
```

The friction force required for pure rolling is:

```
F_friction = (2/7) * m * g * sin(theta)
```

Slip occurs when the plate tilt exceeds:

```
sin(theta_slip) > (7/2) * mu_s
```

For typical materials (steel ball on aluminium plate, mu_s ≈ 0.5), slip threshold is about 54° — far beyond the ball-balancer's operating range. The rolling-without-slipping assumption is safe.

## Rolling Friction

Even on a level surface, a rolling ball decelerates due to rolling resistance:

```
F_rolling = C_rr * F_normal
```

where C_rr is the coefficient of rolling resistance (typically 0.001–0.01 for hard ball on hard surface).

The deceleration from rolling friction:

```
a_friction = -(5/7) * C_rr * g * sign(v)
```

Rolling friction is small but important for steady-state: without it, the ball never truly stops at the setpoint — it oscillates around it. Including rolling friction in the simulation makes the controller's job easier (the plant has natural damping).

## Ball Boundary Detection

The ball stays on the plate as long as its position is within the plate bounds:

```
on_plate = |x| < W/2  and  |y| < H/2
```

where W × H is the plate size. When the ball reaches the edge, it falls off. The simulation should detect this and stop the ball's dynamics (or flag a reset).

There is no edge collision — the ball doesn't bounce. It simply leaves the plate surface.

## Full Equations of Motion (Ball on Tilted Plate)

State vector for one axis:

```
x = [position, velocity]
```

Derivatives:

```
x_dot = [
    velocity,
    (5/7) * g * sin(theta) - (5/7) * C_rr * g * sign(velocity)
]
```

For both axes (4-state system):

```
state = [ball_x, ball_y, vel_x, vel_y]

d(ball_x)/dt = vel_x
d(ball_y)/dt = vel_y
d(vel_x)/dt = (5/7) * (g * sin(beta)  - C_rr * g * sign(vel_x))
d(vel_y)/dt = (5/7) * (g * sin(alpha) - C_rr * g * sign(vel_y))
```

This is integrated with RK4 alongside the servo dynamics at each timestep.

## Implementation Notes

The reference implementation provides a `RollingBallDynamics` class that:
1. Computes ball acceleration from plate tilt angles
2. Includes the 5/7 rolling factor
3. Includes rolling friction
4. Checks plate boundary
5. Returns the state derivative for RK4 integration
