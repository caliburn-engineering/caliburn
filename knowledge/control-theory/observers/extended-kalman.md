---
title: Extended Kalman Filter (EKF)
sources:
  - { note: "engineering experience" }
  - { web: "Tucker McClure — How Kalman Filters Work", url: "https://www.anuncommonlab.com/articles/how-kalman-filters-work/" }
requires:
  - kalman-filter.md
  - ../../math/linear-algebra/matrix-operations.md
related:
  - kalman-filter.md
---

# Extended Kalman Filter (EKF)

The EKF extends the discrete-time Kalman filter to nonlinear systems by linearising the process and observation functions at the current estimate using their Jacobians. At each step, the nonlinear functions are used for state propagation and measurement prediction, while the Jacobians are used for covariance propagation and gain computation -- exactly like a linear KF but with fresh linearisation every cycle.

The EKF was developed for the Apollo program's onboard navigation and remains the workhorse state estimator in aerospace, robotics, and mechatronics. It trades a small amount of accuracy (first-order approximation only) for dramatic speed gains over sigma-point and particle filters.

## Nonlinear System Model

**Process model:**

$$x_k = f(x_{k-1}, u_{k-1}) + w_{k-1}, \quad w_{k-1} \sim \mathcal{N}(0, Q)$$

**Measurement model:**

$$z_k = h(x_k) + v_k, \quad v_k \sim \mathcal{N}(0, R)$$

The functions $f$ and $h$ are nonlinear but must be differentiable (smooth). The noise terms $w$ and $v$ are zero-mean, Gaussian, and mutually uncorrelated -- same assumptions as the linear KF.

## Key Equations

### Jacobians

The state transition Jacobian -- partial derivatives of $f$ with respect to the state, evaluated at the current estimate:

$$F_k = \left. \frac{\partial f}{\partial x} \right|_{\hat{x}_{k-1|k-1}}$$

The observation Jacobian -- partial derivatives of $h$ with respect to the state, evaluated at the predicted state:

$$H_k = \left. \frac{\partial h}{\partial x} \right|_{\hat{x}_{k|k-1}}$$

The Jacobian maps a small perturbation $\Delta x$ at the linearisation point to a perturbation in the output: $\Delta x_k \approx F_k \, \Delta x_{k-1}$. This is why the EKF is fundamentally a linear filter applied to a nonlinear problem -- all covariance operations use first-order Taylor approximations.

### Predict Step

$$\hat{x}_{k|k-1} = f(\hat{x}_{k-1|k-1}, \, u_{k-1})$$

$$P_{k|k-1} = F_k \, P_{k-1|k-1} \, F_k^\top + Q$$

Note: the state is propagated through the full nonlinear $f$, but the covariance is propagated through the linearised $F_k$. The covariance is assumed to stay centred on the propagated estimate -- one of the EKF's core assumptions.

### Update Step

**Innovation:**

$$y_k = z_k - h(\hat{x}_{k|k-1})$$

**Innovation covariance:**

$$S_k = H_k \, P_{k|k-1} \, H_k^\top + R$$

**Kalman gain:**

$$K_k = P_{k|k-1} \, H_k^\top \, S_k^{-1}$$

**Corrected state:**

$$\hat{x}_{k|k} = \hat{x}_{k|k-1} + K_k \, y_k$$

**Corrected covariance (Joseph form):**

$$P_{k|k} = (I - K_k H_k) \, P_{k|k-1} \, (I - K_k H_k)^\top + K_k \, R \, K_k^\top$$

The Joseph form is essential for the EKF -- more so than for the linear KF -- because the linearisation error compounds with roundoff to make the naive form unreliable.

## Source Implementations

### Complete Algorithm (one cycle)

```
INPUTS:
  x_hat  -- previous corrected state estimate
  P      -- previous corrected covariance
  u      -- control input
  z      -- new measurement
  f(x,u) -- nonlinear process function
  h(x)   -- nonlinear observation function
  F(x)   -- Jacobian of f wrt x
  H(x)   -- Jacobian of h wrt x
  Q, R   -- noise covariance matrices

PREDICT:
  x_hat_minus = f(x_hat, u)               // nonlinear propagation
  F_k         = F(x_hat)                   // Jacobian at previous estimate
  P_minus     = F_k * P * F_k' + Q        // covariance propagation

UPDATE:
  y     = z - h(x_hat_minus)              // innovation
  H_k   = H(x_hat_minus)                  // Jacobian at predicted state
  S     = H_k * P_minus * H_k' + R        // innovation covariance
  K     = P_minus * H_k' * inv(S)         // Kalman gain
  x_hat = x_hat_minus + K * y             // corrected state

  // Joseph form covariance update
  A     = I - K * H_k
  P     = A * P_minus * A' + K * R * K'

OUTPUTS:
  x_hat, P
```

### Iterated EKF (IEKF)

Re-linearise and re-correct multiple times at each measurement to improve accuracy on highly nonlinear problems. Each iteration uses the latest corrected state for Jacobian evaluation but always corrects relative to the prior:

```
x_prior = x_hat_minus
z_hat   = h(x_prior)
x_iter  = x_prior

for i = 1 to N_iterations:
    H_k   = H(x_iter)
    S     = H_k * P_minus * H_k' + R
    K     = P_minus * H_k' * inv(S)
    x_iter = x_prior + K * (z - z_hat)    // correct wrt prior, not wrt last iteration

P = (I - K * H_k) * P_minus * (I - K * H_k)' + K * R * K'
x_hat = x_iter
```

Two to three iterations are usually sufficient. Diminishing returns beyond that.

## EKF for the Ball-Balancer

State vector: $x = [x, y, \dot{x}, \dot{y}, \theta_x, \theta_y]^\top$ -- ball position, ball velocity, plate angles.

The rolling constraint couples ball velocity to plate angular rate nonlinearly. At small angles, the small-angle approximation ($\sin\theta \approx \theta$) recovers an exactly linear system and a standard Kalman filter suffices. The EKF upgrade path uses the exact Jacobian of the rolling dynamics, improving accuracy when plate angles are not small.

## When EKF Fails

The EKF makes two assumptions beyond the standard KF:

1. **Functions are smooth (differentiable).** Contact dynamics, bouncing, switching modes, and friction transitions violate this. The Jacobian is undefined at a discontinuity.

2. **Covariance stays centred on the estimate during propagation.** Highly nonlinear dynamics can shift the true mean away from the propagated estimate. The EKF does not capture this shift (a second-order effect).

Additional failure modes:

- **Divergence from poor initialisation.** If the initial estimate is far from truth, the Jacobians are evaluated at the wrong operating point and corrections push the estimate further away. The filter spirals.
- **Redundant states.** Estimating three Euler angles when only two are independent creates a near-singular covariance. Estimate only independent states.
- **Scale mismatch.** Mixing metres and microradians in the state vector causes catastrophic cancellation during matrix operations.

## Alternatives

When the EKF's assumptions break down, consider:

| Filter | When to use | Cost vs EKF |
|---|---|---|
| **UKF (Unscented)** | Strong nonlinearity, Jacobians hard to derive, need 2nd-order accuracy | ~3x slower, no Jacobians needed |
| **Particle filter** | Multimodal distributions, non-Gaussian noise, discontinuous dynamics | ~1000x slower, most general |
| **Linear KF with error states** | Dynamics linear near a known trajectory or operating point | Faster, constant Jacobians |

The UKF propagates $2n{+}1$ sigma points through the full nonlinear function, capturing second-order effects without Jacobians. It is the natural next step when the EKF struggles.

Particle filters represent uncertainty as weighted samples and handle arbitrary distributions, but are impractical for real-time embedded use with high-dimensional states.

## Practical: Analytical vs Numerical Jacobians

### Analytical Jacobians (preferred)

Derive $\partial f / \partial x$ and $\partial h / \partial x$ by hand (or with symbolic math tools) and implement as code. Benefits:

- Exact to machine precision
- Fast -- no repeated function evaluations
- Compiler can optimise and inline

For the ball-balancer, the dynamics are simple enough that analytical Jacobians are straightforward.

### Numerical Jacobians (fallback)

Approximate each column of $F$ by finite differences:

$$F_{:,j} \approx \frac{f(x + \epsilon \, e_j) - f(x - \epsilon \, e_j)}{2\epsilon}$$

where $e_j$ is the $j$-th unit vector. Central differences give second-order accuracy. The perturbation $\epsilon$ is typically $\sqrt{\text{machine epsilon}} \cdot \max(|x_j|, 1)$.

Drawbacks:
- Requires $2n$ function evaluations per Jacobian (for central differences)
- Sensitive to perturbation size -- too small causes roundoff, too large introduces truncation error
- Slower than analytical, especially for large state vectors

Use numerical Jacobians during prototyping, then replace with analytical for production.

### Automatic Differentiation

A middle ground: use an AD library (e.g. CppAD, Ceres Jet types) to compute exact Jacobians from the function source code without manual derivation. Produces results identical to analytical Jacobians with the convenience of numerical ones. Consider this for complex dynamics where hand-derivation is error-prone.

## Implementation Notes

### C++ with Eigen

```cpp
// Ball-balancer EKF: 6-state, 2-measurement
constexpr int N = 6;  // [x, y, vx, vy, theta_x, theta_y]
constexpr int M = 2;  // [x_meas, y_meas]

using VecN = Eigen::Matrix<double, N, 1>;
using VecM = Eigen::Matrix<double, M, 1>;
using MatNN = Eigen::Matrix<double, N, N>;
using MatMM = Eigen::Matrix<double, M, M>;
using MatNM = Eigen::Matrix<double, N, M>;
using MatMN = Eigen::Matrix<double, M, N>;

struct EKF {
    VecN  x;   // state estimate
    MatNN P;   // covariance

    MatNN Q;   // process noise covariance
    MatMM R;   // measurement noise covariance

    // Nonlinear process model -- user provides
    VecN  (*f)(const VecN& x, const VecN& u);
    MatNN (*calc_F)(const VecN& x, const VecN& u);

    // Nonlinear observation model -- user provides
    VecM  (*h)(const VecN& x);
    MatMN (*calc_H)(const VecN& x);

    void predict(const VecN& u) {
        MatNN F = calc_F(x, u);
        x = f(x, u);
        P = F * P * F.transpose() + Q;
    }

    void update(const VecM& z) {
        MatMN H = calc_H(x);
        VecM  y = z - h(x);                              // innovation
        MatMM S = H * P * H.transpose() + R;              // innovation cov

        // Solve K * S = P * H' via Cholesky -- avoids explicit inverse
        MatNM K = S.llt().solve(
            (P * H.transpose()).transpose()
        ).transpose();

        x = x + K * y;

        // Joseph form
        MatNN A = MatNN::Identity() - K * H;
        P = A * P * A.transpose() + K * R * K.transpose();

        // Force symmetry
        P = 0.5 * (P + P.transpose());
    }
};
```

### Numerical Stability

All guidance from the linear KF applies, plus:

- **Re-evaluate Jacobians every step.** Caching Jacobians across steps defeats the purpose of the EKF.
- **Joseph form is non-negotiable.** The combination of linearisation error and roundoff makes the naive covariance update reliably fail in EKFs.
- **Symmetry enforcement** after every covariance update: `P = 0.5 * (P + P.transpose())`.
- **Positive-definiteness check:** Periodically verify that all eigenvalues of $P$ are positive. If any go negative, the filter has diverged. In Eigen: `P.eigenvalues().real().minCoeff() > 0`.
- **Sequential scalar updates** work identically to the linear case when $R$ is diagonal. Process each sensor independently with its own $H$ row, reducing the matrix inverse to scalar division.
- **Innovation monitoring:** Track the normalised innovation squared $\epsilon_k = y_k^\top S_k^{-1} y_k$ over a sliding window. Persistent values above the chi-square bound indicate model mismatch or divergence.
