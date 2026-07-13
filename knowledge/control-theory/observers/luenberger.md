---
title: Luenberger Observer — Deterministic State Estimation
sources:
  - { book: "Ogata — Modern Control Engineering", chapter: "10" }
  - { book: "Franklin, Powell, Emami-Naeini — Feedback Control", chapter: "7" }
requires:
  - ../state-space.md
related:
  - kalman-filter.md
  - extended-kalman.md
  - ../controllers/lqr.md
reference: ../../../reference/observers/luenberger.h
---

# Luenberger Observer — Deterministic State Estimation

The Luenberger observer is the foundational deterministic state estimator. Given a linear system with known dynamics and a measured output, it reconstructs the full state vector by running a model of the plant in parallel and correcting the model's prediction using the measurement error. The correction gain $L$ is chosen by pole placement to give desired convergence speed.

The Luenberger observer is the conceptual predecessor to the Kalman filter. Where Kalman computes the optimal gain given noise statistics, Luenberger lets the designer choose convergence dynamics directly via pole placement. This makes it simpler to design, tune, and reason about — at the cost of no noise optimality guarantees.

## Equations

**Observer dynamics (continuous-time):**

$$\dot{\hat{x}} = A \hat{x} + B u + L (y - C \hat{x})$$

Rearranging:

$$\dot{\hat{x}} = (A - LC) \hat{x} + B u + L y$$

**Estimation error dynamics:**

Define $e = x - \hat{x}$. Then:

$$\dot{e} = (A - LC) e$$

The estimation error is a linear autonomous system with eigenvalues determined entirely by the matrix $(A - LC)$. The observer converges if and only if all eigenvalues of $(A - LC)$ have negative real parts.

**Observer poles:**

$$\text{Observer poles} = \text{eig}(A - LC)$$

The designer places these poles to achieve the desired convergence rate.

## Design Rules

### Pole Placement Heuristic

Place observer poles **2-10x faster** than controller poles.

- **Faster observer (10x)** — tracks state changes quickly, estimates converge rapidly after disturbances. But amplifies sensor noise because high gain magnifies measurement errors.
- **Slower observer (2x)** — produces smoother estimates because low gain filters noise. But lags behind the true state during transients.
- **Sweet spot (3-5x)** — good for most mechatronics applications. Start here and adjust based on noise observed in practice.

### The Observer Bandwidth Trade-off

This is the fundamental tension in all observer design:

| Speed up observer | Slow down observer |
|---|---|
| Better tracking of fast dynamics | Smoother estimates |
| Faster convergence from wrong IC | Less sensor noise amplification |
| Higher noise sensitivity | Worse transient tracking |
| Larger control effort from noisy estimates | Observer lag affects closed-loop performance |

The Kalman filter resolves this trade-off optimally for stochastic systems. The Luenberger observer puts the choice in the designer's hands.

### Practical Guidelines

- Never place observer poles slower than controller poles — the observer must provide accurate estimates before the controller needs them.
- For systems with known sensor noise characteristics, use the Kalman filter instead — it will naturally find the right speed/noise balance.
- For deterministic simulation or when noise is negligible, Luenberger is simpler and gives direct pole control.
- For output feedback (controller using estimated states), the separation principle guarantees that observer and controller can be designed independently — see below.

## Observability Requirement

The system must be **observable** for the observer to work:

$$\text{rank}(\mathcal{O}) = n \quad \text{where} \quad \mathcal{O} = \begin{bmatrix} C \\ CA \\ CA^2 \\ \vdots \\ CA^{n-1} \end{bmatrix}$$

**Observable** means: the full state can be reconstructed from output measurements over a finite time window.

**If not observable:** there exist internal dynamics (state directions) that the sensors cannot see. The observer will diverge for those unobservable modes — no choice of $L$ can fix this. The system must be redesigned (add sensors, or only estimate the observable subspace).

### Checking Observability in Practice

For small systems (n < 5): compute the observability matrix rank directly.

For larger systems: use the PBH (Popov-Belevitch-Hautus) test — $(A, C)$ is observable iff $\text{rank}\begin{bmatrix} \lambda I - A \\ C \end{bmatrix} = n$ for every eigenvalue $\lambda$ of $A$. This reveals exactly which modes are unobservable.

## Relationship to Kalman Filter

| Aspect | Luenberger | Kalman |
|---|---|---|
| Design method | Pole placement — designer chooses L | Optimal computation — L from Riccati equation |
| Noise model | None (deterministic) | Requires Q (process) and R (measurement) |
| Optimality | None guaranteed | Minimises mean squared estimation error |
| Tuning | Pole locations (direct, geometric) | Q/R ratio (indirect, statistical) |
| Complexity | Simple eigenvalue assignment | Riccati equation solve (offline or recursive) |
| Runtime cost | Same — both are `x_hat = Ax + Bu + L(y - Cx)` | Same structure, different L |

**The Kalman filter IS a Luenberger observer** with an optimally computed gain:

- Large $Q_n/R_n$ ratio → trust sensors more → high gain → fast observer poles (like fast Luenberger placement)
- Small $Q_n/R_n$ ratio → trust model more → low gain → slow observer poles (like slow Luenberger placement)

The steady-state Kalman gain $K_\infty$ places the observer poles at a specific location determined by the noise covariances. A Luenberger observer with $L = K_\infty$ is identical to the steady-state Kalman filter.

## Separation Principle

For linear systems, the controller and observer can be designed independently:

1. Design the state-feedback controller as if all states are available (e.g., LQR gives gain $K$)
2. Design the observer to estimate the states (Luenberger gives gain $L$)
3. Use the estimated states for feedback: $u = -K \hat{x}$

**Result:** The closed-loop poles are the union of the controller poles $\text{eig}(A - BK)$ and the observer poles $\text{eig}(A - LC)$. They do not interact.

This holds exactly for linear systems. For nonlinear systems (EKF + nonlinear controller), separation is an approximation — it usually works well in practice but has no theoretical guarantee.

## Discrete-Time Form

For digital implementation at sample period $T_s$:

$$\hat{x}_{k+1} = A_d \hat{x}_k + B_d u_k + L_d (y_k - C \hat{x}_k)$$

where $A_d = e^{AT_s}$ and $B_d = A^{-1}(A_d - I)B$ are the discretised system matrices.

The discrete observer gain $L_d$ is designed by placing the eigenvalues of $(A_d - L_d C)$ inside the unit circle. The 2-10x rule translates to: observer pole magnitudes should be $(1/2)$ to $(1/10)$ of the controller pole magnitudes (closer to zero = faster in discrete time).

## Source Implementation

### Complete Algorithm (one cycle, discrete-time)

```
INPUTS:
  x_hat  -- previous state estimate      (n x 1)
  u      -- control input                 (p x 1)
  y      -- measurement                   (m x 1)
  A, B, C, L -- system and observer gain matrices

PREDICT + CORRECT (combined):
  innovation = y - C * x_hat              // measurement error
  x_hat = A * x_hat + B * u + L * innovation

OUTPUTS:
  x_hat  -- updated state estimate
```

Note: Unlike the Kalman filter, there is no covariance propagation. The Luenberger observer maintains only the state estimate — no uncertainty quantification.

## Implementation Notes

### Gain Computation via Ackermann's Formula (SISO)

For single-output systems, the observer gain $L$ can be computed using Ackermann's formula:

$$L = \alpha_c(A) \, \mathcal{O}^{-1} \, e_n$$

where $\alpha_c(A)$ is the desired characteristic polynomial evaluated at $A$, $\mathcal{O}$ is the observability matrix, and $e_n = [0, 0, \ldots, 1]^\top$.

For MIMO systems, use numerical pole placement algorithms (e.g., the robust pole placement method in control toolboxes).

### Numerical Considerations

- The gain $L$ is computed offline (during design) and stored as a constant matrix. No online matrix operations needed beyond the predict-correct step.
- For high-order systems, condition the observability matrix before inversion. If it is ill-conditioned, the desired pole locations may require impractically large gains.
- In discrete time, ensure observer poles stay well inside the unit circle. Poles near $|z| = 1$ give slow convergence; poles near $z = 0$ give deadbeat convergence (exact in $n$ steps) but require very high gains.

## Automotive Applications

| Application | States estimated | Sensors | Notes |
|---|---|---|---|
| Active suspension | Tyre deflection, road profile | Body accelerometer, suspension travel | Quarter-car model, Luenberger works well for linear range |
| Motor temperature | Rotor temperature | Stator temperature, current, voltage | Thermal model is linear — Luenberger on thermal network |
| Steering rack position | Rack position from motor angle | Motor encoder, torque sensor | Simple kinematic model |
