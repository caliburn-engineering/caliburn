---
title: Linear-Quadratic Regulator
sources:
  - { note: "engineering experience" }
requires:
  - ../state-space.md
  - ../../math/linear-algebra/eigenvalues.md
related:
  - pid.md
  - ../observers/kalman-filter.md
reference: ../../../reference/controllers/lqr.h
---

# Linear-Quadratic Regulator

LQR is an optimal state-feedback controller for linear time-invariant systems. Given a state-space model (A, B), it computes a gain matrix K that minimises a quadratic cost function balancing state error against control effort. Unlike PID, LQR considers the full state vector simultaneously and produces mathematically optimal gains for a given Q/R weighting. It is the natural upgrade path from PID when a reliable state-space model exists and the full state is observable (directly or via a Kalman filter).

## Key Equations

### Cost Function (Infinite-Horizon)

```
J(u) = integral_0^inf (x^T Q x + u^T R u) dt
```

- **x^T Q x** penalises deviation from the origin (state error energy)
- **u^T R u** penalises control effort (actuator energy)
- Q must be symmetric positive semi-definite: Q = Q^T >= 0
- R must be symmetric positive definite: R = R^T > 0

### Optimal Control Law

The control input that minimises J is a linear state feedback:

```
u(t) = -K x(t)
```

where the gain matrix is:

```
K = R^{-1} B^T P
```

and P is the solution to the algebraic Riccati equation.

### Algebraic Riccati Equation (ARE)

At steady state (infinite horizon), the Riccati differential equation reduces to:

```
A^T P + P A - P B R^{-1} B^T P + Q = 0
```

P is the unique symmetric positive definite solution when (A, B) is controllable and (A, Q^{1/2}) is observable.

### Closed-Loop Dynamics

With u = -Kx, the closed-loop system becomes:

```
x_dot = (A - B K) x
```

The eigenvalues of (A - BK) are guaranteed to have negative real parts (stable) when the ARE has a solution.

### Finite-Horizon LQR

For a terminal cost at time T:

```
J(u) = integral_0^T (x^T Q x + u^T R u) dt + x(T)^T Q_f x(T)
```

The Riccati equation becomes a differential equation solved backward from P(T) = Q_f:

```
-P_dot = A^T P + P A - P B R^{-1} B^T P + Q
```

The gain K(t) = R^{-1} B^T P(t) is time-varying. For most regulation tasks, the infinite-horizon formulation suffices and K is constant.

## Choosing Q and R Matrices

### Structure

Q and R are typically diagonal. Each diagonal entry weights the corresponding state or input:

```
Q = diag(q_1, q_2, ..., q_n)    — one weight per state
R = diag(r_1, r_2, ..., r_m)    — one weight per control input
```

### Tuning Intuition

| Want | Action |
|---|---|
| Tighter position tracking | Increase Q entry for position states |
| Less aggressive actuator motion | Increase R entries |
| Faster velocity damping | Increase Q entry for velocity states |
| Equal priority across states | Start with Q = I, R = I |

### Bryson's Rule (Starting Point)

Normalise each weight by the maximum acceptable value:

```
Q_ii = 1 / (max_acceptable_x_i)^2
R_jj = 1 / (max_acceptable_u_j)^2
```

For the ball-balancer: if max acceptable position error is 0.02 m and max tilt angle is 0.15 rad, then Q_pos = 1/0.02^2 = 2500 and R = 1/0.15^2 = 44.4.

### Iterative Refinement

1. Start with Q = I, R = rho * I (rho = 1)
2. Simulate closed-loop step response
3. If too slow: decrease rho (cheaper control)
4. If too aggressive or actuator saturates: increase rho
5. Adjust individual Q_ii to prioritise specific states

## Implementation Notes

### Solving the ARE in C++ with Eigen

Eigen does not provide a built-in Riccati solver. Common approaches:

**1. Schur decomposition of the Hamiltonian matrix:**

```cpp
// Hamiltonian: H = [A, -B*R_inv*B^T; -Q, -A^T]
Eigen::MatrixXd H(2*n, 2*n);
H.topLeftCorner(n, n) = A;
H.topRightCorner(n, n) = -B * R.inverse() * B.transpose();
H.bottomLeftCorner(n, n) = -Q;
H.bottomRightCorner(n, n) = -A.transpose();

// Real Schur decomposition, reorder stable eigenvalues to top-left
Eigen::RealSchur<Eigen::MatrixXd> schur(H);
// Extract U1, U2 from the first n columns of the unitary matrix
// P = U2 * U1.inverse()
```

**2. Iterative solution (simple but slower):**

```cpp
Eigen::MatrixXd P = Q;  // initial guess
for (int i = 0; i < max_iter; ++i) {
    Eigen::MatrixXd P_next = A.transpose() * P * A
        - A.transpose() * P * B * (R + B.transpose() * P * B).inverse()
        * B.transpose() * P * A + Q;
    if ((P_next - P).norm() < tol) break;
    P = P_next;
}
```

Note: the iterative form above is the discrete-time Riccati iteration. For continuous-time, use the Hamiltonian/Schur approach or a dedicated library.

### Computing the Gain

```cpp
Eigen::MatrixXd K = R.llt().solve(B.transpose() * P);
// K = R^{-1} B^T P — solved via Cholesky, not explicit inverse
```

Always solve the linear system rather than computing R.inverse() explicitly. Cholesky (LLT) is appropriate because R is symmetric positive definite.

### Verifying Stability

After computing K, check that the closed-loop system is stable:

```cpp
Eigen::MatrixXd A_cl = A - B * K;
Eigen::EigenSolver<Eigen::MatrixXd> es(A_cl);
for (int i = 0; i < n; ++i) {
    assert(es.eigenvalues()(i).real() < 0.0);  // all eigenvalues in LHP
}
```

### LQR with Integral Action

Standard LQR drives states to zero but does not guarantee zero steady-state error for non-zero setpoints. Augment the state with integral of the tracking error:

```
x_aug = [x; integral(e)]
A_aug = [A, 0; -C, 0]
B_aug = [B; 0]
```

Then design Q_aug and R for the augmented system. The resulting K_aug = [K_state, K_integral] combines proportional state feedback with integral action.

### Ball-Balancer Considerations

- The 9D state-space model (x, y, z_ball, vx, vy, vz_ball, varphi_x, theta_y, z_table) is controllable for the planar subsystem.
- Q should heavily weight position states (x, y) and lightly weight servo angle states (varphi_x, theta_y) since those are means, not ends.
- R penalises commanded tilt angles. Too small and the servos hit their limits; too large and the ball response is sluggish.
- When pairing LQR with a Kalman filter (LQG), the separation principle guarantees that the controller and estimator can be designed independently.

### Numerical Stability

- Symmetrise P after each iteration: P = 0.5 * (P + P^T).
- Check condition number of R before solving: cond(R) should be below 1e10.
- For ill-conditioned systems, use `FullPivLU` instead of `LLT` for the R solve.
- The ARE may not converge if (A, B) is not stabilisable. Always verify controllability first.
