---
title: Rigid Body Equations of Motion
sources:
  - { note: "engineering experience" }
requires:
  - ../../math/linear-algebra/matrix-operations.md
  - ../rk4.md
related:
  - inertia-tensors.md
  - quaternion-rotation.md
---

# Rigid Body Equations of Motion

The motion of a rigid body is governed by the Newton-Euler equations: Newton's second law for translational dynamics and Euler's equation for rotational dynamics. A rigid body has 6 degrees of freedom (3 translational, 3 rotational), and its state evolves as a coupled system of first-order ODEs suitable for numerical integration.

In the ball-balancer context, the tilting plate is a rigid body actuated by servos, and the ball is modelled as a point mass (or uniform sphere with rolling constraint) on the plate surface. The full Newton-Euler treatment is needed when the small-angle linearisation breaks down.

## Key Equations

### Newton-Euler Equations

Translational dynamics (Newton):

```
F = m * a
```

Rotational dynamics (Euler, in body frame with principal axes):

```
tau = I * alpha + omega x (I * omega)
```

Expanded in principal-axis components (A, B, C are principal moments of inertia):

```
A * p_dot = tau_x + (B - C) * q * r
B * q_dot = tau_y + (C - A) * r * p
C * r_dot = tau_z + (A - B) * p * q
```

where (p, q, r) are the angular velocity components in the body frame and (tau_x, tau_y, tau_z) are the applied torque components.

### Translational State Equations

```
p_dot = v           (position derivative = velocity)
v_dot = F / m       (velocity derivative = force / mass)
```

For a body in a gravity field with external forces:

```
v_dot = g + F_ext / m
```

### Rotational State Equations

```
omega_dot = I^{-1} * (tau - omega x (I * omega))
```

For principal axes this simplifies to the Euler equations above. The orientation is tracked separately via quaternion kinematics (see `quaternion-rotation.md`).

### Full State Vector

A rigid body's state for simulation is:

```
x = [position(3), velocity(3), orientation(4), angular_velocity(3)]
```

Total: 13 components (orientation is a unit quaternion with 4 components and 1 constraint).

The time derivative of this state vector:

```
x_dot = [
    velocity,                                    // 3
    forces / mass,                               // 3
    0.5 * q * [0, omega],                        // 4  (quaternion derivative)
    I^{-1} * (tau - omega x (I * omega))         // 3
]
```

### System of ODEs for RK4 Integration

Package the 13-component state into a single vector and define the derivative function:

```
f(t, x) = [v, F(t,x)/m, 0.5*q*omega_quat, I_inv*(tau(t,x) - omega x (I*omega))]
```

Feed this into a standard RK4 integrator. After each step, re-normalise the quaternion component to prevent drift (see `quaternion-rotation.md`).

### Ball-Balancer Application

**Plate as rigid body:** The tilting plate has two servo-driven rotation axes (alpha, beta). In the linearised model, plate dynamics reduce to alpha_ddot = f(servo_input). For large tilts, the full Euler equations with the plate's inertia tensor are needed.

**Ball on surface:** Under small-angle approximation, the ball's acceleration on the plate is:

```
a_ball = (5/7) * g * sin(theta)  ~  (5/7) * g * theta   (small angle)
```

The 5/7 factor arises from the rolling constraint of a uniform sphere (I_sphere = 2/5 * m * R^2), giving an effective translational acceleration of g*sin(theta) / (1 + I/(mR^2)) = (5/7)*g*sin(theta).

**Coupled system:** For high-fidelity simulation, the plate and ball form a coupled dynamical system. The plate's angular state determines the gravitational component acting on the ball, and the ball's weight and position create a torque on the plate that the servos must counteract.

## Implementation Notes

State vector as an Eigen type:

```cpp
struct RigidBodyState {
    Eigen::Vector3d position;
    Eigen::Vector3d velocity;
    Eigen::Quaterniond orientation;   // unit quaternion
    Eigen::Vector3d angular_velocity; // body frame
};
```

Euler's equations in code:

```cpp
Eigen::Vector3d euler_rotational_acceleration(
    const Eigen::Matrix3d& I_inv,
    const Eigen::Matrix3d& I,
    const Eigen::Vector3d& omega,
    const Eigen::Vector3d& torque)
{
    return I_inv * (torque - omega.cross(I * omega));
}
```

Numerical stability considerations:

- The `omega x (I * omega)` gyroscopic term can dominate at high angular velocities. Use double precision throughout.
- Re-normalise the quaternion after every RK4 step: `q.normalize()`.
- For stiff systems (e.g., high servo stiffness), consider an implicit integrator or reduce the timestep.
- When coupling plate and ball dynamics, solve both in the same RK4 step to avoid lag between subsystems.
