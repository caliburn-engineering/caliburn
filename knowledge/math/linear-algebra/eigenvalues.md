---
title: Eigenvalues and Eigenvectors
sources:
  - { book: "Kreyszig - Advanced Engineering Mathematics", chapter: "8" }
  - { book: "Fedorov - Yet Another Introduction to Numerical Methods", chapter: "9", pages: "77-84" }
  - { note: "engineering experience" }
requires:
  - matrix-operations.md
related:
  - state-space-transforms.md
  - ../../control-theory/stability.md
---

# Eigenvalues and Eigenvectors

Eigenvalues reveal the intrinsic behavior of a linear system. For the ball-balancer, the eigenvalues of the system matrix A determine whether the controller stabilizes the ball, how fast disturbances decay, and whether oscillations occur. Every stability analysis starts here.

## Definition

For a square matrix A, a scalar lambda is an **eigenvalue** and a nonzero vector v is the corresponding **eigenvector** if:

```
Av = lambda * v       (v != 0)
```

The eigenvector v is a direction that A merely scales (by lambda) rather than rotating. The eigenvalue lambda is the scale factor.

## Key Equations

### Characteristic Polynomial

Eigenvalues are the roots of the characteristic polynomial:

```
P_A(lambda) = det(A - lambda * I) = 0
```

For an n x n matrix, this is a degree-n polynomial with n roots (counting multiplicity, possibly complex).

### Eigenspace

The eigenspace for eigenvalue lambda_i is the null space of (A - lambda_i * I):

```
V_{lambda_i} = ker(A - lambda_i * I) = { v in R^n | Av = lambda_i * v }
```

A multiplicity-k eigenvalue has an eigenspace of dimension 1 to k. If every eigenvalue's eigenspace dimension equals its multiplicity, A is diagonalizable.

### Trace and Determinant Relations

```
trace(A) = sum of eigenvalues = lambda_1 + lambda_2 + ... + lambda_n
det(A)   = product of eigenvalues = lambda_1 * lambda_2 * ... * lambda_n
```

These provide quick sanity checks without computing eigenvalues explicitly.

## Computing Eigenvalues

### Algorithm (by hand or conceptual)

1. Form A - lambda * I
2. Compute det(A - lambda * I) = 0 (characteristic polynomial)
3. Find roots lambda_1, ..., lambda_n
4. For each lambda_i, solve (A - lambda_i * I)v = 0 via row reduction to find eigenvectors

### Symmetric Matrices (A = A^T)

Symmetric matrices have exceptional spectral properties (Spectral Theorem):

1. **All eigenvalues are real** — no complex eigenvalues to worry about
2. **Eigenvectors for distinct eigenvalues are orthogonal** — proof: x^T A y = mu(x . y) and x^T A y = lambda(x . y), so (mu - lambda)(x . y) = 0, thus x . y = 0
3. **Always diagonalizable** — via orthogonal matrix Q: A = Q D Q^T where Q^T Q = I

This matters for the ball-balancer because covariance matrices (Kalman filter), inertia tensors, and stiffness matrices are all symmetric.

## Physical Interpretation

### Natural Frequencies and Vibration Modes

For a mass-spring system M x'' + K x = 0, the generalized eigenvalue problem K v = omega^2 M v gives:
- **Eigenvalues** omega_i^2: squares of natural frequencies
- **Eigenvectors** v_i: mode shapes (which parts move together)

### Stability of a Linear System

For the state-space system x_dot = Ax, the eigenvalues of A completely determine stability:

| Eigenvalue type | System behavior |
|---|---|
| All Re(lambda_i) < 0 | **Asymptotically stable** — all states decay to zero |
| Complex pair with Re < 0 | **Damped oscillation** — decaying sinusoid |
| Real negative lambda | **Exponential decay** — overdamped response |
| Any Re(lambda_i) > 0 | **Unstable** — state grows without bound |
| Re(lambda_i) = 0 (purely imaginary) | **Marginally stable** — sustained oscillation, no convergence |
| lambda = 0 | **Integrator** — state drifts, system does not return |

For the ball-balancer, the open-loop plant is unstable (positive real eigenvalue from gravity). The controller must place all closed-loop eigenvalues in the left half-plane.

### Eigenvalue Magnitude and Time Constants

The time constant for an eigenvalue lambda is:

```
tau = -1 / Re(lambda)
```

Larger |Re(lambda)| means faster decay. The dominant (slowest) eigenvalue determines the overall settling time of the system.

For complex eigenvalues lambda = sigma +/- j*omega:
- sigma determines decay rate
- omega determines oscillation frequency
- Damping ratio: zeta = -sigma / sqrt(sigma^2 + omega^2)

## Implementation Notes

### Eigen's Eigenvalue Solvers

Eigen provides two main solvers. Choosing the right one matters for correctness and performance.

**SelfAdjointEigenSolver — for symmetric matrices (preferred when applicable):**

```cpp
#include <Eigen/Eigenvalues>

// Symmetric matrix (covariance, stiffness, inertia tensor)
Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> solver(A);
Eigen::VectorXd eigenvalues = solver.eigenvalues();    // real, sorted ascending
Eigen::MatrixXd eigenvectors = solver.eigenvectors();  // orthonormal columns
```

Exploits symmetry for:
- Guaranteed real eigenvalues (no complex arithmetic)
- Orthonormal eigenvectors
- Better numerical stability
- Faster computation

**EigenSolver — for general (non-symmetric) matrices:**

```cpp
// General square matrix (system matrix A in state-space)
Eigen::EigenSolver<Eigen::MatrixXd> solver(A);
Eigen::VectorXcd eigenvalues = solver.eigenvalues();    // complex in general
Eigen::MatrixXcd eigenvectors = solver.eigenvectors();  // complex in general
```

Use this for the system matrix A of the ball-balancer plant, which is generally not symmetric.

### Stability Check Pattern

```cpp
bool is_stable(const Eigen::MatrixXd& A) {
    Eigen::EigenSolver<Eigen::MatrixXd> solver(A, /* computeEigenvectors = */ false);
    return (solver.eigenvalues().real().array() < 0).all();
}
```

Setting `computeEigenvectors = false` saves computation when only eigenvalues are needed (stability check).

### Checking Eigenvalue Quality

For ill-conditioned matrices, eigenvalue computation can be unreliable. Verify by checking the residual:

```cpp
// For each eigenvalue/eigenvector pair, check ||Av - lambda*v||
for (int i = 0; i < A.cols(); i++) {
    Eigen::VectorXcd v = solver.eigenvectors().col(i);
    std::complex<double> lambda = solver.eigenvalues()(i);
    double residual = (A.cast<std::complex<double>>() * v - lambda * v).norm();
    // residual should be close to machine epsilon * ||A|| * ||v||
}
```

### Dominant Eigenvalue for Settling Time

```cpp
// Find the eigenvalue closest to zero (slowest mode = bottleneck)
Eigen::EigenSolver<Eigen::MatrixXd> solver(A_cl);  // closed-loop system matrix
double slowest_decay = solver.eigenvalues().real().maxCoeff();
// For a stable system, slowest_decay < 0
// Approximate settling time (to 2% of initial value):
double settling_time = -4.0 / slowest_decay;  // 4 time constants
```
