---
title: State-Space Representation
sources:
  - { book: "Kreyszig - Advanced Engineering Mathematics", chapter: "4" }
  - { note: "engineering experience" }
requires:
  - ../math/linear-algebra/matrix-operations.md
related:
  - controllers/pid.md
  - controllers/lqr.md
  - observers/kalman-filter.md
---

# State-Space Representation

State-space is the modern framework for modelling linear time-invariant (LTI) systems. It expresses system dynamics as a set of first-order matrix differential equations, making it natural for multivariable systems, computer implementation, and connection to optimal control (LQR) and state estimation (Kalman filter). Every transfer function has an equivalent state-space form, but state-space generalises to MIMO systems where transfer functions become unwieldy.

## Key Equations

### Continuous-Time State-Space (ABCD Form)

```
x_dot(t) = A x(t) + B u(t)    (state equation)
y(t)     = C x(t) + D u(t)    (output equation)
```

| Matrix | Dimension | Role |
|---|---|---|
| **A** | n x n | System dynamics — how the state evolves autonomously |
| **B** | n x m | Input matrix — how control inputs affect the state |
| **C** | p x n | Output matrix — which states are observed |
| **D** | p x m | Direct feedthrough — input-to-output bypass (often zero) |

Where:
- **x** is the n-dimensional state vector
- **u** is the m-dimensional control input vector
- **y** is the p-dimensional output vector

### Solution of the State Equation

For the autonomous system (u = 0):

```
x(t) = e^{A(t - t_0)} x(t_0)
```

where e^{At} is the matrix exponential (state transition matrix, Phi(t)).

For the forced system:

```
x(t) = e^{At} x(0) + integral_0^t e^{A(t - tau)} B u(tau) dtau
```

### Transfer Function from State-Space

```
H(s) = C (sI - A)^{-1} B + D
```

The matrix (sI - A)^{-1} is the resolvent. Its poles are the eigenvalues of A — the natural frequencies of the system. The characteristic polynomial is det(sI - A) = 0.

### State-Space from Transfer Function

Given a SISO transfer function H(s) = (b_m s^m + ... + b_0) / (s^n + a_{n-1} s^{n-1} + ... + a_0), the controllable canonical form is:

```
A = [0, 1, 0, ..., 0;
     0, 0, 1, ..., 0;
     ...
     -a_0, -a_1, -a_2, ..., -a_{n-1}]

B = [0; 0; ...; 1]

C = [b_0, b_1, ..., b_m, 0, ..., 0]

D = [0]  (if m < n, i.e., strictly proper)
```

## Controllability

A system (A, B) is **controllable** if any state can be reached from any other state by choosing an appropriate input u(t).

### Controllability Matrix

```
C_ctrl = [B, AB, A^2 B, ..., A^{n-1} B]
```

The system is controllable if and only if rank(C_ctrl) = n (full row rank).

### Physical Meaning

Controllability asks: "Can the input physically influence every state variable?" For the ball-balancer, the plate tilt commands (u) must be able to drive all 9 state variables. The vertical states (z_ball, vz_ball, z_table) are not controllable by tilt alone — they depend on gravity — so the effective controllable subsystem is the 6D planar model (x, y, vx, vy, varphi_x, theta_y).

### Pole Placement

If (A, B) is controllable, a state feedback u = -Kx can place the eigenvalues of (A - BK) at any desired location in the complex plane. This is the theoretical foundation for both manual pole placement and LQR.

## Observability

A system (A, C) is **observable** if the full state x(t) can be determined from the output history y(t) over a finite interval.

### Observability Matrix

```
O = [C; CA; CA^2; ...; CA^{n-1}]
```

The system is observable if and only if rank(O) = n (full column rank).

### Physical Meaning

Observability asks: "Can we infer every state from what we measure?" In the ball-balancer, the camera measures only ball position (x, y). Velocity, angle, and table height must be inferred. The system is observable for the planar subsystem because position measurements contain enough information (through system dynamics) to reconstruct velocities and angles over time. This is exactly what the Kalman filter exploits.

### Duality

The pair (A, B) is controllable if and only if (A^T, B^T) is observable, and vice versa. Controller design and observer design are dual problems.

## Discretization

Continuous-time models must be discretised for digital implementation.

### Forward Euler (First-Order Approximation)

```
A_d = I + A_c * dt
B_d = B_c * dt
```

Simple and fast. Adequate when dt is small relative to the fastest system time constant. Used in the ball-balancer at dt = 0.01 s:

```cpp
A_ = SystemMatrix::Identity() + Ac * dt_;
B_ = Bc * dt_;
```

### Exact Discretization (Matrix Exponential)

```
A_d = e^{A_c * dt}
B_d = A_c^{-1} (A_d - I) B_c
```

More accurate but requires computing the matrix exponential. Use for systems with fast dynamics or large dt.

### Tustin (Bilinear) Transform

```
s = (2/dt) * (z - 1) / (z + 1)
```

Preserves frequency-domain properties (maps the imaginary axis to the unit circle). Preferred for filter design but more complex to implement for state-space models.

### Choosing a Method

| Method | Accuracy | Cost | When to use |
|---|---|---|---|
| Forward Euler | O(dt) | Trivial | dt << tau_min (fastest time constant) |
| Matrix Exponential | Exact | Moderate | Moderate dt or stiff systems |
| Tustin | Exact at DC, good at Nyquist | Moderate | Frequency-domain-critical applications |

For the ball-balancer with tau_servo = 0.05 s and dt = 0.01 s, Forward Euler introduces ~10% discretization error on the servo dynamics. Acceptable for initial development; upgrade to matrix exponential for production.

## Implementation Notes

### Eigen State-Space Structure

```cpp
// State dimension
constexpr int N_STATES = 9;
constexpr int N_INPUTS = 2;
constexpr int N_OUTPUTS = 2;

using StateVector  = Eigen::Matrix<double, N_STATES, 1>;
using SystemMatrix = Eigen::Matrix<double, N_STATES, N_STATES>;
using InputMatrix  = Eigen::Matrix<double, N_STATES, N_INPUTS>;
using OutputMatrix = Eigen::Matrix<double, N_OUTPUTS, N_STATES>;

SystemMatrix A_ = SystemMatrix::Zero();
InputMatrix  B_ = InputMatrix::Zero();
OutputMatrix C_ = OutputMatrix::Zero();
// D is typically zero — omitted
```

### Ball-Balancer A Matrix Construction

```cpp
// Rolling dynamics: a = (5/7) * g * sin(theta) ≈ (5/7) * g * theta
constexpr double rolling_factor = 5.0 / 7.0 * 9.81;  // ~7.007 m/s^2

// Position-velocity coupling
Ac(X, VX)     = 1.0;
Ac(Y, VY)     = 1.0;

// Gravity-driven acceleration from plate tilt
Ac(VX, PHI)   = rolling_factor;   // varphi_x tilts ball in X
Ac(VY, THETA) = rolling_factor;   // theta_y tilts ball in Y

// First-order servo model: d(angle)/dt = (cmd - angle) / tau
Ac(PHI, PHI)     = -1.0 / tau_servo;
Ac(THETA, THETA) = -1.0 / tau_servo;

// Input matrix: commands enter via servo model
Bc(PHI, 0)   = 1.0 / tau_servo;
Bc(THETA, 1) = 1.0 / tau_servo;
```

### Controllability and Observability Checks

```cpp
// Controllability matrix: [B, AB, A^2B, ..., A^{n-1}B]
Eigen::MatrixXd C_ctrl(n, n * m);
Eigen::MatrixXd A_power = Eigen::MatrixXd::Identity(n, n);
for (int i = 0; i < n; ++i) {
    C_ctrl.middleCols(i * m, m) = A_power * B;
    A_power *= A;
}

// Rank check via column-pivoting QR
Eigen::ColPivHouseholderQR<Eigen::MatrixXd> qr(C_ctrl);
qr.setThreshold(1e-10);
bool controllable = (qr.rank() == n);
```

Same pattern for observability: build O = [C; CA; CA^2; ...] and check rank.

### Numerical Stability

- Use `ColPivHouseholderQR` for rank checks — it handles near-singular matrices gracefully.
- When computing A^k for large k, accumulate via repeated multiplication rather than matrix power, to track numerical drift.
- For the resolvent (sI - A)^{-1}, never form the explicit inverse. Use `lu().solve()` or `colPivHouseholderQr().solve()`.
- Eigenvalue computation for stability analysis: use `Eigen::EigenSolver` for general matrices or `SelfAdjointEigenSolver` if A is symmetric.
