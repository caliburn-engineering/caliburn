---
title: Discrete-Time Kalman Filter
sources:
  - { note: "engineering experience" }
  - { web: "Tucker McClure — How Kalman Filters Work", url: "https://www.anuncommonlab.com/articles/how-kalman-filters-work/" }
requires:
  - ../../math/linear-algebra/matrix-operations.md
  - ../state-space.md
related:
  - extended-kalman.md
  - ../controllers/lqr.md
reference: ../../../reference/observers/kalman_filter.h
---

# Discrete-Time Kalman Filter

The Kalman filter is the optimal recursive state estimator for linear systems with Gaussian noise. It maintains a belief about the system state as a mean (state estimate) and covariance (uncertainty), updating both each time a new measurement arrives. For linear systems where the noise assumptions hold, it minimises the mean squared estimation error -- no other linear filter can do better.

In the ball-balancer context, the plate dynamics near level are well-approximated as linear, making the discrete-time KF the natural starting point before graduating to an EKF for large-angle operation.

## System Model

**Process model** (how the state evolves):

$$x_k = F \, x_{k-1} + B \, u_{k-1} + w_{k-1}$$

- $x_k \in \mathbb{R}^n$ -- state vector
- $F$ -- state transition matrix (discrete)
- $B$ -- control input matrix
- $u_{k-1}$ -- control input (known)
- $w_{k-1} \sim \mathcal{N}(0, Q)$ -- process noise

**Measurement model** (what we observe):

$$z_k = H \, x_k + v_k$$

- $z_k \in \mathbb{R}^m$ -- measurement vector
- $H$ -- observation matrix
- $v_k \sim \mathcal{N}(0, R)$ -- measurement noise

The noise terms $w$ and $v$ are assumed zero-mean, Gaussian, and mutually uncorrelated.

## Key Equations

### Predict Step

Propagate state and covariance forward in time:

$$\hat{x}_{k|k-1} = F \, \hat{x}_{k-1|k-1} + B \, u_{k-1}$$

$$P_{k|k-1} = F \, P_{k-1|k-1} \, F^\top + Q$$

Intuition: $F$ maps perturbations at $k{-}1$ to perturbations at $k$ (first-order Taylor). The process noise $Q$ always grows the covariance -- uncertainty increases between measurements.

### Update Step

Incorporate the measurement to correct the prediction:

**Innovation (measurement residual):**

$$y_k = z_k - H \, \hat{x}_{k|k-1}$$

**Innovation covariance:**

$$S_k = H \, P_{k|k-1} \, H^\top + R$$

**Kalman gain:**

$$K_k = P_{k|k-1} \, H^\top \, S_k^{-1}$$

**Corrected state estimate:**

$$\hat{x}_{k|k} = \hat{x}_{k|k-1} + K_k \, y_k$$

**Corrected covariance (Joseph form -- always use this):**

$$P_{k|k} = (I - K_k H) \, P_{k|k-1} \, (I - K_k H)^\top + K_k \, R \, K_k^\top$$

The Joseph form guarantees the covariance stays symmetric positive-semidefinite despite floating-point roundoff. The naive form `P = P - K * H * P` will eventually produce an indefinite covariance matrix and the filter will diverge.

### Kalman Gain Intuition

The gain $K$ is the ratio of "how uncertain we are about the prediction" to "how uncertain we are about the innovation." When measurement noise is low relative to prediction uncertainty, $K$ is large and we trust the measurement. When measurement noise is high, $K$ is small and we mostly trust the prediction.

## Source Implementations

### Complete Algorithm (one cycle)

```
INPUTS:
  x_hat  -- previous corrected state estimate (n x 1)
  P      -- previous corrected covariance     (n x n)
  u      -- control input                     (p x 1)
  z      -- new measurement                   (m x 1)
  F, B, H, Q, R  -- system matrices (constant for linear KF)

PREDICT:
  x_hat_minus = F * x_hat + B * u
  P_minus     = F * P * F' + Q

UPDATE:
  y     = z - H * x_hat_minus            // innovation
  S     = H * P_minus * H' + R           // innovation covariance
  K     = P_minus * H' * inv(S)          // Kalman gain
  x_hat = x_hat_minus + K * y            // corrected state

  // Joseph form covariance update
  A     = I - K * H
  P     = A * P_minus * A' + K * R * K'

OUTPUTS:
  x_hat, P
```

### Steady-State / Wiener Filter

When $F$, $H$, $Q$, $R$ are all constant, the covariance converges to a steady-state value independent of the state. The Kalman gain therefore also converges. Once converged, the filter reduces to:

```
x_hat = F * x_hat + B * u
x_hat = x_hat + K_ss * (z - H * x_hat)   // K_ss is constant
```

No covariance propagation needed at runtime. This is the lightest possible estimator with Kalman-optimal properties.

## Tuning Q and R

**R (measurement noise covariance):** Usually the easiest to set. Take sensor noise specifications (standard deviations) and place $\sigma^2$ values on the diagonal. For independent sensors, $R$ is diagonal.

**Q (process noise covariance):** Harder. Represents unmodelled dynamics and disturbances. Two approaches:

1. **Physics-based:** Identify the physical noise source (e.g. random acceleration), express its effect on the state via $F_q$, and compute $Q = F_q \, Q_{source} \, F_q^\top$. For a position-velocity system with random acceleration noise $\sigma_a^2$:

$$Q = \begin{bmatrix} \frac{\Delta t^4}{4} & \frac{\Delta t^3}{2} \\ \frac{\Delta t^3}{2} & \Delta t^2 \end{bmatrix} \sigma_a^2$$

2. **Empirical tuning:** Start with small diagonal $Q$, run Monte Carlo simulations, check filter consistency (NEES test), and increase $Q$ until 95% of normalised errors fall within chi-square bounds. Too-small $Q$ makes the filter optimistic (trusts its model too much, risks divergence). Too-large $Q$ makes it pessimistic (noisy estimates but robust).

**P_0 (initial covariance):** Set diagonal entries to $\sigma_i^2$ reflecting initial uncertainty in each state. If initialising from a measurement, use $R$ for measured states and a generous value for unobserved states (e.g. velocity).

## Innovation Sequence and Filter Health Monitoring

The innovation sequence $\{y_k\}$ is the single most informative diagnostic for a running Kalman filter.

**Healthy filter indicators:**
- Innovations are zero-mean (test with running average)
- Innovations are white (uncorrelated in time -- check autocorrelation)
- Innovations are consistent with $S_k$: the normalised innovation squared (NIS) $\epsilon_k = y_k^\top S_k^{-1} y_k$ should follow a chi-square distribution with $m$ degrees of freedom

**Divergence indicators:**
- Innovation mean drifts away from zero -- model bias
- Innovations are correlated in time -- model is missing dynamics
- NIS consistently exceeds chi-square bounds -- filter is optimistic, increase $Q$
- Covariance diagonal goes negative -- numerical failure, switch to Joseph form or square-root filter

**Practical monitoring** (embedded): Track a sliding window of NIS values. If the windowed average exceeds the 99% chi-square bound for $N$ consecutive samples, flag a filter fault. Optionally reinitialise from the current measurement.

## Implementation Notes

### C++ with Eigen

```cpp
// State: [x, y, vx, vy]  Measurement: [x, y]
constexpr int N = 4;  // state dimension
constexpr int M = 2;  // measurement dimension

using VecN = Eigen::Matrix<double, N, 1>;
using VecM = Eigen::Matrix<double, M, 1>;
using MatNN = Eigen::Matrix<double, N, N>;
using MatMM = Eigen::Matrix<double, M, M>;
using MatNM = Eigen::Matrix<double, N, M>;
using MatMN = Eigen::Matrix<double, M, N>;

struct KalmanFilter {
    VecN  x;   // state estimate
    MatNN P;   // covariance

    MatNN F;   // state transition
    MatMN H;   // observation
    MatNN Q;   // process noise covariance
    MatMM R;   // measurement noise covariance

    void predict(const VecN& Bu) {
        x = F * x + Bu;
        P = F * P * F.transpose() + Q;
    }

    void update(const VecM& z) {
        VecM  y = z - H * x;                            // innovation
        MatMM S = H * P * H.transpose() + R;             // innovation cov
        MatNM K = P * H.transpose() * S.inverse();       // Kalman gain

        x = x + K * y;

        // Joseph form
        MatNN A = MatNN::Identity() - K * H;
        P = A * P * A.transpose() + K * R * K.transpose();
    }
};
```

### Numerical Stability Notes

- **Always use Joseph form** for the covariance update. The naive `P = P - K*H*P` subtraction will accumulate roundoff and eventually produce negative diagonal entries.
- **Symmetry enforcement:** After each covariance update, force `P = 0.5 * (P + P.transpose())` to prevent asymmetry drift.
- **Avoid explicit matrix inverse** of $S$. Use `S.llt().solve(...)` (Cholesky) or `S.ldlt().solve(...)` for better numerical behaviour and speed.
- **Sequential scalar updates:** When $R$ is diagonal (independent sensors), update with one measurement at a time. The matrix inverse reduces to scalar division -- dramatically faster for large measurement vectors.
- **Square-root filter:** For embedded or low-precision contexts, store the Cholesky factor $C$ where $P = C C^\top$ instead of $P$. This doubles the effective numerical range. The Apollo Guidance Computer used this approach.
- **State scaling:** Keep state elements at similar numerical scales. Mixing metres and microradians in the same state vector causes catastrophic cancellation. Use appropriate units or normalisation.
- **Discretisation:** When deriving $F$ from continuous-time dynamics $\dot{x} = F_c x$, use the matrix exponential $F = e^{F_c \Delta t}$. For small $\Delta t$, the first-order approximation $F \approx I + F_c \Delta t$ is acceptable but drifts for larger steps. The bilinear (Tustin) transform offers a good middle ground.
