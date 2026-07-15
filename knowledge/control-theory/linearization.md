---
title: Linearization
sources:
  - { book: "Ogata — Modern Control Engineering", chapter: "3" }
  - { book: "Slotine & Li — Applied Nonlinear Control", chapter: "3" }
requires:
  - state-space.md
  - first-principles-modelling.md
related:
  - stability.md
  - frequency-response.md
  - controllers/gain-scheduling.md
  - second-order-systems.md
---

# Linearization

Linearization approximates a nonlinear system with a linear one near an operating point. This is the bridge between first-principles modelling (which produces nonlinear ODEs) and linear control design tools (Bode, root locus, LQR, pole placement) which all require state-space or transfer function models.

## Jacobian Linearization

Given a nonlinear system:

```
x_dot = f(x, u)
y = h(x, u)
```

At an equilibrium point (x0, u0) where f(x0, u0) = 0, the linearized system is:

```
delta_x_dot = A * delta_x + B * delta_u
delta_y = C * delta_x + D * delta_u
```

Where delta_x = x - x0, delta_u = u - u0, and the Jacobian matrices are:

```
A = df/dx |_(x0, u0)     (n x n)
B = df/du |_(x0, u0)     (n x m)
C = dh/dx |_(x0, u0)     (p x n)
D = dh/du |_(x0, u0)     (p x m)
```

This is a first-order Taylor expansion — higher-order terms are dropped.

## Validity Region

The linear model is accurate only "near" the operating point. How near depends on the curvature of f:

- **Mildly nonlinear** (sin at small angles): valid over a wide range. sin(0.1) ≈ 0.1 with 0.17% error, sin(0.3) ≈ 0.3 with 1.5% error.
- **Strongly nonlinear** (saturation, dead zones, friction): valid over a very narrow range. Coulomb friction switches sign discontinuously — linearization fails at v=0.
- **Rule of thumb:** if the nonlinear and linear time responses agree within 5% for the expected input range, the linearization is adequate for control design.

## Numerical Linearization

When the analytical Jacobian is tedious (many states, complex coupling), compute A and B numerically via central finite differences:

```
A(:, i) = (f(x0 + eps*e_i, u0) - f(x0 - eps*e_i, u0)) / (2 * eps)
B(:, j) = (f(x0, u0 + eps*e_j) - f(x0, u0 - eps*e_j)) / (2 * eps)
```

Where e_i is the i-th unit vector and eps is a small perturbation (typically 1e-6). Central differences give O(eps^2) accuracy, much better than forward differences.

**Best practice:** Derive A, B analytically first, then validate against the numerical result. This catches derivation errors while building physical understanding.

## When Linearization Fails

- **Discontinuities:** Coulomb friction, backlash, relay controllers. The Jacobian doesn't exist at the switching surface.
- **Limit cycles:** Oscillations that linearization cannot predict (requires describing function or simulation).
- **Large-signal behaviour:** Saturation, actuator limits, constraint activation. The linear model doesn't know about these.
- **Multiple equilibria:** Linearization is valid around one equilibrium. If the system has several (e.g., inverted vs hanging pendulum), you need separate linear models for each.

For these cases, use nonlinear control methods: sliding mode (robust to model uncertainty), gain scheduling (multiple linear models), or MPC (handles constraints directly).

## Worked Example: Ball-on-Plate

See `projects/ball-balancer/src/ball_plant_linear.h` for the implementation and `reference/models/linearizer.h` for the validation tool.

**Nonlinear dynamics:**
```
ax = (5/7) * g * sin(beta)    — Y-axis tilt drives x-acceleration
ay = (5/7) * g * sin(alpha)   — X-axis tilt drives y-acceleration
```

**Linearization at (x=0, v=0, alpha=0, beta=0):**
- sin(alpha) ≈ alpha, sin(beta) ≈ beta
- Rolling friction drops out at zero velocity
- System decouples into two identical double integrators

**Result:** A is 4x4 with ones on the (0,2) and (1,3) entries (position-velocity coupling). B has (5/7)*g on the (2,1) and (3,0) entries (input-acceleration coupling). The system is controllable and observable.

## Connection to Frequency Domain

Once you have (A, B, C, D), you can compute the transfer function matrix:

```
G(s) = C * (sI - A)^(-1) * B + D
```

This enables Bode plots, Nyquist diagrams, and root locus analysis — all starting from the linearized state-space model. See [Frequency Response](frequency-response.md) for details.

## Connection to Gain Scheduling

When one linear model isn't enough, linearize at multiple operating points and schedule between them. See [Gain Scheduling](controllers/gain-scheduling.md) for the approach.
