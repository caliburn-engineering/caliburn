---
title: Stability Analysis
sources:
  - { book: "Kreyszig - Advanced Engineering Mathematics", chapter: "4" }
  - { note: "engineering experience" }
requires:
  - ../math/linear-algebra/eigenvalues.md
  - state-space.md
related:
  - controllers/lqr.md
---

# Stability Analysis

Stability determines whether a system returns to equilibrium after a disturbance or diverges. For the ball-balancer, the open-loop plant is inherently unstable (the ball rolls off the plate under gravity), so the controller must move all closed-loop eigenvalues into the left half-plane. Stability analysis provides the mathematical tools to verify this before deploying on hardware.

## Key Equations

### Two Types of Stability

**BIBO (Bounded-Input Bounded-Output):** A system is BIBO stable if every bounded input produces a bounded output. Concerns input-output behavior only.

**Internal (Lyapunov) Stability:** A system is internally stable if, after any small perturbation from equilibrium, the state remains bounded (stable) or returns to equilibrium (asymptotically stable). Concerns the state trajectory.

For LTI systems, BIBO stability and internal stability are equivalent when the system is both controllable and observable (no hidden unstable modes).

### Eigenvalue Criterion

**A linear time-invariant system x_dot = Ax is asymptotically stable if and only if all eigenvalues of A have strictly negative real parts: Re(lambda_i) < 0 for all i.**

| Eigenvalue location | Behavior |
|---|---|
| Real negative (lambda < 0) | Exponential decay — stable |
| Complex with Re(lambda) < 0 | Damped oscillation — stable |
| Purely imaginary (Re = 0) | Sustained oscillation — marginally stable |
| Any Re(lambda) > 0 | Exponential growth — unstable |

### Phase Plane Classification (2D Systems)

For a 2D system y_dot = Ay, the trace p = tr(A) = lambda_1 + lambda_2 and determinant q = det(A) = lambda_1 * lambda_2 determine behavior:

| Condition | Type | Stability |
|---|---|---|
| p < 0, q > 0, Delta < 0 | Stable spiral | Asymptotically stable |
| p < 0, q > 0, Delta > 0 | Stable node | Asymptotically stable |
| p = 0, q > 0 | Center | Marginally stable (closed orbits) |
| q < 0 | Saddle point | Unstable |
| p > 0 | Unstable spiral or node | Unstable |

where Delta = p^2 - 4q (discriminant).

**Ball-balancer open-loop:** Without control, the ball-on-plate subsystem around the flat equilibrium is a saddle point (q < 0). The ball accelerates away from center under any displacement. PID or LQR control flips the eigenvalue signs, converting the saddle into a stable spiral — the geometric picture of what feedback control achieves.

## Lyapunov Stability Theory

### Direct Method (Second Method of Lyapunov)

For a nonlinear system x_dot = f(x) with equilibrium at x = 0, find a scalar function V(x) (Lyapunov candidate) such that:

1. V(0) = 0
2. V(x) > 0 for all x != 0 (positive definite)
3. V_dot(x) = dV/dt = (grad V)^T f(x) <= 0

If V_dot < 0 (strictly): the equilibrium is **asymptotically stable**.
If V_dot <= 0 (non-strictly): the equilibrium is **stable** (but not necessarily asymptotically).

### Quadratic Lyapunov for LTI Systems

For x_dot = Ax, the quadratic form V(x) = x^T P x is a Lyapunov function if P is symmetric positive definite and:

```
V_dot = x^T (A^T P + P A) x = -x^T Q_lyap x < 0
```

This gives the **Lyapunov equation:**

```
A^T P + P A = -Q_lyap
```

If Q_lyap is positive definite and P solves this equation as positive definite, the system is asymptotically stable. This is equivalent to all eigenvalues of A having negative real parts.

### Practical Use

Lyapunov analysis is most valuable for nonlinear systems where eigenvalue analysis only covers the linearised behavior. For the ball-balancer, the linearisation is valid for small angles (theta < 0.15 rad). Beyond that, the sin(theta) nonlinearity matters, and a Lyapunov argument can confirm stability over a larger operating region.

## Routh-Hurwitz Criterion

An algebraic test for whether all roots of a polynomial have negative real parts, without computing the roots explicitly. Useful for analysing the characteristic polynomial det(sI - A) = 0.

### Procedure

For the characteristic polynomial:

```
p(s) = a_n s^n + a_{n-1} s^{n-1} + ... + a_1 s + a_0
```

1. Construct the Routh array (n+1 rows)
2. First column: a_n, a_{n-1}, then computed entries
3. **Criterion:** All entries in the first column of the Routh array must be positive (assuming a_n > 0). The number of sign changes equals the number of roots with positive real parts.

### Example (Second-Order)

For p(s) = s^2 + a_1 s + a_0:

```
Row 1:  1     a_0
Row 2:  a_1   0
Row 3:  a_0
```

Stable iff a_1 > 0 and a_0 > 0. For a damped oscillator s^2 + 2*zeta*omega_n*s + omega_n^2: stable when zeta > 0 and omega_n > 0.

### When to Use

Routh-Hurwitz is most useful for low-order systems (n <= 4) where the polynomial coefficients have physical meaning. For higher-order systems (like the 9D ball-balancer), direct eigenvalue computation is more practical.

## Stability Margins

Frequency-domain measures of "how close to instability" a system is. Defined on the open-loop transfer function L(s) = G_controller(s) * G_plant(s).

### Gain Margin (GM)

The factor by which the loop gain can increase before the system becomes unstable:

```
GM = 1 / |L(j*omega_pi)|
```

where omega_pi is the phase crossover frequency (where angle(L) = -180 degrees).

GM > 1 (positive in dB) means stable. Typical target: GM >= 6 dB (factor of 2).

### Phase Margin (PM)

The additional phase lag the system can tolerate before instability:

```
PM = 180 + angle(L(j*omega_c))
```

where omega_c is the gain crossover frequency (where |L| = 1 = 0 dB).

PM > 0 means stable. Typical target: PM >= 30-45 degrees.

### Practical Interpretation

| Margin | Too small means | Ball-balancer symptom |
|---|---|---|
| Low gain margin | Small parameter changes cause instability | Increasing Kp slightly makes the ball oscillate |
| Low phase margin | System is oscillatory, near resonance | Ball overshoots and rings before settling |

Phase margin relates directly to damping ratio: PM ~ 100*zeta degrees for zeta < 0.7. A PM of 45 degrees corresponds to zeta ~ 0.45 — moderately damped.

## Implementation Notes

### Eigenvalue Stability Check in C++

```cpp
#include <Eigen/Eigenvalues>

bool is_stable(const Eigen::MatrixXd& A) {
    Eigen::EigenSolver<Eigen::MatrixXd> solver(A, /* compute eigenvectors = */ false);
    for (int i = 0; i < A.rows(); ++i) {
        if (solver.eigenvalues()(i).real() >= 0.0) {
            return false;
        }
    }
    return true;
}
```

For the closed-loop system after controller design:

```cpp
Eigen::MatrixXd A_cl = A - B * K;  // LQR closed-loop
assert(is_stable(A_cl) && "Closed-loop system is not stable!");
```

### Lyapunov Equation Solver

Solving A^T P + P A = -Q for P (continuous-time Lyapunov equation):

```cpp
// Bartels-Stewart algorithm via Schur decomposition
// For small systems, direct vectorization works:
// vec(P) = (I kron A^T + A^T kron I)^{-1} vec(Q)
// But prefer a dedicated solver for n > 4.

// Eigen does not provide a built-in Lyapunov solver.
// Use the Kronecker product approach for small n:
Eigen::MatrixXd I_n = Eigen::MatrixXd::Identity(n, n);
Eigen::MatrixXd M = kronecker(I_n, A.transpose()) + kronecker(A.transpose(), I_n);
Eigen::VectorXd p_vec = M.colPivHouseholderQr().solve(
    Eigen::Map<Eigen::VectorXd>(Q.data(), n * n));
Eigen::MatrixXd P = Eigen::Map<Eigen::MatrixXd>(p_vec.data(), n, n);
```

### Stability Margin Computation

For frequency-domain analysis, evaluate the loop transfer function on the imaginary axis:

```cpp
// Evaluate L(j*omega) = C * (j*omega*I - A)^{-1} * B at a grid of frequencies
std::complex<double> j(0.0, 1.0);
for (double omega : freq_grid) {
    Eigen::MatrixXcd sI_A = j * omega * I_n.cast<std::complex<double>>()
                          - A.cast<std::complex<double>>();
    Eigen::VectorXcd L = C.cast<std::complex<double>>()
                       * sI_A.colPivHouseholderQr().solve(
                           B.cast<std::complex<double>>());
    double magnitude = L.norm();
    double phase = std::arg(L(0));  // SISO
    // Find crossover frequencies and compute margins
}
```

### Numerical Considerations

- Eigenvalue computation is O(n^3). For the 9D ball-balancer, this is trivial (nanoseconds).
- For marginally stable systems (eigenvalues near the imaginary axis), use extended precision or verify with the Lyapunov equation.
- When checking stability after gain changes, recompute eigenvalues rather than relying on margin estimates — margins assume linearity and can be misleading for multi-loop systems.
- For discrete-time systems, the stability criterion changes: all eigenvalues must lie inside the unit circle (|lambda_i| < 1), not in the left half-plane.
