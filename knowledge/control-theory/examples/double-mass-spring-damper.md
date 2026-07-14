---
title: "Worked Example: Double Mass-Spring-Damper (2 DOF)"
sources:
  - { book: "Kreyszig - Advanced Engineering Mathematics", chapter: "4" }
  - { video: "Steve Brunton — Mechanical Systems and State-Space", url: "https://www.youtube.com/watch?v=O_1NNB1rqiU" }
  - { note: "engineering experience" }
requires:
  - ../state-space.md
  - ../first-principles-modelling.md
related:
  - inverted-pendulum-cart.md
  - quarter-car-model.md
  - ../controllers/lqr.md
  - ../stability.md
reference: ../../../reference/models/double_mass_spring_damper.h
---

# Worked Example: Double Mass-Spring-Damper (2 DOF)

This is the canonical multi-DOF mechanical system. Two masses connected by springs and dampers, one end fixed to a wall. A force F(t) acts on m_2. It appears constantly in interviews because it tests the complete modelling pipeline: FBD, Newton, coupled ODEs, state-space, and optionally Laplace/transfer functions.

## System Diagram

```
            k_1          k_2
┌──────┐ /\/\/\ ┌──────┐ /\/\/\ ┌──────┐
│ Wall │────────│  m_1  │────────│  m_2  │
│      │────────│      │────────│      │
└──────┘  c_1    └──┬───┘  c_2    └──┬───┘
 (fixed)   ───      │       ───      │
                    │                │
                   x_1              x_2        -> positive direction
                                     ^
                                    F(t) (external force on m_2)
```

**DOF = 2.** Coordinates: x_1 (displacement of m_1 from equilibrium), x_2 (displacement of m_2). Positive direction: to the right. Both measured from equilibrium position.

## Step 1: Free Body Diagram -- Mass 1

```
     <---- k_1 * x_1              k_2 * (x_2 - x_1) ---->
     <---- c_1 * x_dot_1          c_2 * (x_dot_2 - x_dot_1) ---->
                    ┌──────┐
                    │  m_1  │
                    └──────┘
```

Forces on m_1:
- Spring 1 pulls back: -k_1 * x_1 (restoring toward equilibrium)
- Damper 1 resists motion: -c_1 * x_dot_1
- Spring 2 pulls/pushes based on relative displacement: +k_2 * (x_2 - x_1)
- Damper 2 based on relative velocity: +c_2 * (x_dot_2 - x_dot_1)

## Step 2: Newton's 2nd Law -- Mass 1

```
m_1 * x_ddot_1 = -k_1 * x_1 - c_1 * x_dot_1 + k_2 * (x_2 - x_1) + c_2 * (x_dot_2 - x_dot_1)
```

Expand and group:

```
m_1 * x_ddot_1 = -(k_1 + k_2) * x_1 - (c_1 + c_2) * x_dot_1 + k_2 * x_2 + c_2 * x_dot_2
```

## Step 3: Free Body Diagram -- Mass 2

```
     <---- k_2 * (x_2 - x_1)                        F(t) ---->
     <---- c_2 * (x_dot_2 - x_dot_1)
                    ┌──────┐
                    │  m_2  │
                    └──────┘
```

Forces on m_2:
- Spring 2: -k_2 * (x_2 - x_1) (reaction force, Newton's 3rd law)
- Damper 2: -c_2 * (x_dot_2 - x_dot_1)
- External force: +F(t)

## Step 4: Newton's 2nd Law -- Mass 2

```
m_2 * x_ddot_2 = -k_2 * (x_2 - x_1) - c_2 * (x_dot_2 - x_dot_1) + F(t)
```

Expand:

```
m_2 * x_ddot_2 = k_2 * x_1 + c_2 * x_dot_1 - k_2 * x_2 - c_2 * x_dot_2 + F(t)
```

## Step 5: Isolate Accelerations

```
x_ddot_1 = [-(k_1+k_2)*x_1 - (c_1+c_2)*x_dot_1 + k_2*x_2 + c_2*x_dot_2] / m_1

x_ddot_2 = [k_2*x_1 + c_2*x_dot_1 - k_2*x_2 - c_2*x_dot_2 + F(t)] / m_2
```

## Step 6: State-Space Assembly

States: **x = [x_1, v_1, x_2, v_2]^T**, Input: u = F(t)

Row 1: x_dot_1 = v_1 (trivial: position derivative = velocity)
Row 2: v_dot_1 = x_ddot_1 (from Newton above)
Row 3: x_dot_2 = v_2 (trivial)
Row 4: v_dot_2 = x_ddot_2 (from Newton above)

```
     | x_dot_1 |   |  0            1             0          0       | | x_1 |   |   0   |
     | v_dot_1 | = | -(k1+k2)/m1  -(c1+c2)/m1   k2/m1      c2/m1  | | v_1 | + |   0   | * F(t)
     | x_dot_2 |   |  0            0             0          1       | | x_2 |   |   0   |
     | v_dot_2 |   |  k2/m2        c2/m2        -k2/m2     -c2/m2  | | v_2 |   | 1/m2  |
```

**Pattern check:** Rows 1 and 3 are trivial [0,1,0,0] and [0,0,0,1]. Rows 2 and 4 come from Newton divided by mass. B has 1/m_2 only at row 4 (force applied to m_2). Coupling terms have opposite signs between masses (Newton's 3rd law).

## Template A Matrix (Copy-Paste)

```
A = |  0            1             0          0       |
    | -(k1+k2)/m1  -(c1+c2)/m1   k2/m1      c2/m1  |
    |  0            0             0          1       |
    |  k2/m2        c2/m2        -k2/m2     -c2/m2  |

B = [0, 0, 0, 1/m2]^T

C = [1, 0, 0, 0;    (output = positions of both masses)
     0, 0, 1, 0]

D = [0; 0]
```

## Laplace Transform Approach

Taking Laplace of the Newton equations (zero initial conditions):

**Mass 1:**
```
m_1 * s^2 * X_1 + (c_1+c_2)*s*X_1 + (k_1+k_2)*X_1 = (c_2*s + k_2)*X_2

[m_1*s^2 + (c_1+c_2)*s + (k_1+k_2)] * X_1 = [c_2*s + k_2] * X_2
```

**Mass 2:**
```
[m_2*s^2 + c_2*s + k_2] * X_2 = [c_2*s + k_2] * X_1 + F(s)
```

**Impedance matrix form:**
```
| Z_11(s)    -Z_12(s) | | X_1 |   |  0   |
|                      | |     | = |      |
| -Z_12(s)    Z_22(s) | | X_2 |   | F(s) |
```

where:
- Z_11(s) = m_1*s^2 + (c_1+c_2)*s + (k_1+k_2)
- Z_22(s) = m_2*s^2 + c_2*s + k_2
- Z_12(s) = c_2*s + k_2

**Transfer functions** (via Cramer's rule or matrix inversion):

```
X_2(s)/F(s) = Z_11(s) / [Z_11(s)*Z_22(s) - Z_12(s)^2]

X_1(s)/F(s) = Z_12(s) / [Z_11(s)*Z_22(s) - Z_12(s)^2]
```

The denominator is a 4th-order polynomial whose roots are the system's natural frequencies (same as the eigenvalues of A).

## Numerical Example

For m_1 = m_2 = 1 kg, k_1 = k_2 = 10 N/m, c_1 = c_2 = 0.5 N*s/m:

```
A = |  0     1      0     0   |
    | -20   -1     10    0.5  |
    |  0     0      0     1   |
    |  10    0.5   -10   -0.5 |
```

Eigenvalues: two complex conjugate pairs corresponding to the two natural modes.
- Mode 1 (in-phase): both masses move together
- Mode 2 (out-of-phase): masses move in opposite directions (higher frequency)

## Key Insights

1. The A matrix has a **block structure**: 2x2 blocks for each mass, with coupling in the off-diagonal blocks.
2. **Symmetry in coupling:** the k_2/m_1 and k_2/m_2 terms appear with matching signs (positive where they pull, negative where they resist).
3. **Scaling to n-DOF:** for a chain of n masses, the A matrix becomes block tri-diagonal. The pattern is identical -- just repeat.
4. **Energy check:** the A matrix should produce a stable system when all stiffness and damping are positive. If your eigenvalues have positive real parts for positive k and c, you have a sign error.
