---
title: "Worked Example: Inverted Pendulum on Cart (2 DOF)"
sources:
  - { book: "Kreyszig - Advanced Engineering Mathematics", chapter: "4" }
  - { book: "Ogata — Modern Control Engineering", chapter: "3" }
  - { note: "engineering experience" }
requires:
  - ../state-space.md
  - ../first-principles-modelling.md
related:
  - double-mass-spring-damper.md
  - quarter-car-model.md
  - ../controllers/lqr.md
  - ../stability.md
reference: ../../../reference/models/inverted_pendulum.h
---

# Worked Example: Inverted Pendulum on Cart (2 DOF)

The inverted pendulum on a cart is the classic Euler-Lagrange example. It is hard to model with Newton (constraint forces at the pivot, rotating reference frame) but elegant with energy methods. It is also the canonical unstable system for demonstrating LQR and state feedback control.

## System Diagram

```
                     *  (pendulum tip, mass m)
                    /
                   /  L (rod length, point mass at tip)
                  /
                 * (pivot on cart)
          ┌──────┴──────┐
          │    Cart M    │<---- F(t)
          └──────────────┘
     ═══════════════════════════
                x -> positive
```

**DOF = 2.** Generalised coordinates: q_1 = x (cart position), q_2 = theta (pendulum angle from vertical upright position). Convention: theta = 0 is upright (unstable equilibrium).

## Step 1: Position of Pendulum Tip

The pendulum mass position in Cartesian coordinates:

```
x_p = x + L * sin(theta)
y_p = L * cos(theta)
```

Velocities:

```
x_dot_p = x_dot + L * theta_dot * cos(theta)
y_dot_p = -L * theta_dot * sin(theta)
```

## Step 2: Kinetic Energy T

**Cart:**
```
T_cart = (1/2) * M * x_dot^2
```

**Pendulum (point mass at tip):**
```
T_pend = (1/2) * m * (x_dot_p^2 + y_dot_p^2)
       = (1/2) * m * [(x_dot + L*theta_dot*cos(theta))^2 + (L*theta_dot*sin(theta))^2]
       = (1/2) * m * [x_dot^2 + 2*x_dot*L*theta_dot*cos(theta) + L^2*theta_dot^2]
```

**Total kinetic energy:**
```
T = (1/2)*(M+m)*x_dot^2 + m*L*x_dot*theta_dot*cos(theta) + (1/2)*m*L^2*theta_dot^2
```

## Step 3: Potential Energy V

```
V = m * g * L * cos(theta)
```

Convention: V = 0 at pivot level. V is maximum when theta = 0 (upright).

## Step 4: Lagrangian

```
L = T - V = (1/2)*(M+m)*x_dot^2 + m*L*x_dot*theta_dot*cos(theta)
            + (1/2)*m*L^2*theta_dot^2 - m*g*L*cos(theta)
```

## Step 5: Euler-Lagrange for x

```
dL/d(x_dot) = (M+m)*x_dot + m*L*theta_dot*cos(theta)

d/dt[dL/d(x_dot)] = (M+m)*x_ddot + m*L*theta_ddot*cos(theta) - m*L*theta_dot^2*sin(theta)

dL/dx = 0

E-L equation:
  (M+m)*x_ddot + m*L*theta_ddot*cos(theta) - m*L*theta_dot^2*sin(theta) = F(t)
```

## Step 6: Euler-Lagrange for theta

```
dL/d(theta_dot) = m*L*x_dot*cos(theta) + m*L^2*theta_dot

d/dt[dL/d(theta_dot)] = m*L*x_ddot*cos(theta) - m*L*x_dot*theta_dot*sin(theta) + m*L^2*theta_ddot

dL/d(theta) = -m*L*x_dot*theta_dot*sin(theta) + m*g*L*sin(theta)

E-L equation:
  m*L*x_ddot*cos(theta) + m*L^2*theta_ddot - m*g*L*sin(theta) = 0
```

(No external torque on the pendulum, so Q_theta = 0. If there were pivot damping: Q_theta = -c * theta_dot.)

## Step 7: Linearisation Around Equilibrium (theta ~ 0)

Small angle approximations: sin(theta) ~ theta, cos(theta) ~ 1, theta_dot^2 ~ 0.

**Linearised equations:**
```
Equation 1 (x):      (M+m)*x_ddot + m*L*theta_ddot = F
Equation 2 (theta):  m*L*x_ddot + m*L^2*theta_ddot = m*g*L*theta
```

## Step 8: Solve for Accelerations via Mass Matrix Inversion

In matrix form:

```
| M+m    m*L  | | x_ddot     |   | F             |
|              | |            | = |               |
| m*L    m*L^2| | theta_ddot |   | m*g*L*theta   |
```

Determinant of the mass matrix:

```
D = (M+m)*m*L^2 - (m*L)^2 = m*L^2*M
```

Solving by Cramer's rule (or direct inversion):

```
x_ddot     = [m*L^2 * F - m*L * m*g*L*theta] / D
           = F/M - m*g*theta/M     (simplified)
           = [m*L^2 * F - m^2*g*L^2 * theta] / (m*L^2*M)

theta_ddot = [-(m*L) * F + (M+m) * m*g*L*theta] / D
           = [-m*L * F + (M+m)*m*g*L * theta] / (m*L^2*M)
```

Exact expressions:

```
x_ddot     = m*L^2*F/(m*L^2*M) - m^2*g*L^2*theta/(m*L^2*M)
           = F/M - m*g*theta/M

theta_ddot = -(m*L)*F/(m*L^2*M) + (M+m)*m*g*L*theta/(m*L^2*M)
           = -F/(M*L) + (M+m)*g*theta/(M*L)
```

## Step 9: State-Space (Linearised)

States: **x = [x, x_dot, theta, theta_dot]^T**, Input: u = F

Using D = m*L^2*M:

```
A = | 0   1         0                    0 |
    | 0   0    -m^2*g*L^2/D              0 |
    | 0   0         0                    1 |
    | 0   0    (M+m)*m*g*L/D             0 |

B = [0, m*L^2/D, 0, -m*L/D]^T
```

Substituting D = m*L^2*M:

```
A = | 0   1         0              0 |
    | 0   0    -m*g/M              0 |
    | 0   0         0              1 |
    | 0   0    (M+m)*g/(M*L)       0 |

B = [0, 1/M, 0, -1/(M*L)]^T
```

## Key Properties

**Open-loop instability:** The A matrix has an eigenvalue with positive real part. The term (M+m)*g/(M*L) in A(4,3) drives the instability -- gravity pulling the pendulum away from vertical. This makes the inverted pendulum the canonical test for feedback stabilisation.

**Controllability:** The system is controllable from the cart force F. Both cart position and pendulum angle can be regulated with a single input.

**Natural LQR application:** Because the system is open-loop unstable and fully controllable, LQR is the natural controller. Choose Q to penalise theta (angle) heavily, R to limit force magnitude. The resulting K will stabilise the pendulum while keeping the cart near the origin.

## C Matrix (Typical Outputs)

```
C = | 1  0  0  0 |    (output: cart position and pendulum angle)
    | 0  0  1  0 |
```

## Numerical Example

For M = 1 kg, m = 0.1 kg, L = 0.5 m, g = 9.81 m/s^2:

```
D = 0.1 * 0.25 * 1.0 = 0.025

A = | 0    1       0        0     |
    | 0    0      -0.981    0     |
    | 0    0       0        1     |
    | 0    0      21.582    0     |

B = [0, 1.0, 0, -2.0]^T
```

Open-loop eigenvalues: 0, 0, +4.645, -4.645. The positive eigenvalue confirms open-loop instability.
