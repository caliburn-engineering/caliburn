---
title: State-Space Transformations
sources:
  - { book: "Kreyszig - Advanced Engineering Mathematics", chapter: "8" }
  - { note: "engineering experience" }
requires:
  - matrix-operations.md
  - eigenvalues.md
related:
  - ../../control-theory/state-space.md
---

# State-Space Transformations

State-space transformations change the coordinate system in which a dynamical system is expressed. The physics does not change — only the representation. Choosing the right representation simplifies analysis, reveals structure, and makes controller design tractable. For the ball-balancer, transformations connect the physical coordinates (ball position, plate angle) to canonical forms where controllability and pole placement become straightforward.

## Similarity Transformations

Given a state-space system:

```
x_dot = Ax + Bu
y     = Cx + Du
```

A change of state variables x = Tz (where T is invertible) produces an equivalent system in z-coordinates:

```
z_dot = A'z + B'u
y     = C'z + Du
```

where:

```
A' = T^{-1} A T
B' = T^{-1} B
C' = C T
D' = D                (unchanged)
```

## Key Equations

### Eigenvalue Preservation

Similarity transformations preserve eigenvalues:

```
det(A' - lambda I) = det(T^{-1} A T - lambda I)
                   = det(T^{-1} (A - lambda I) T)
                   = det(T^{-1}) det(A - lambda I) det(T)
                   = det(A - lambda I)
```

The characteristic polynomial is invariant. Stability, natural frequencies, and damping ratios are intrinsic properties of the system, not artifacts of the coordinate choice.

Other preserved quantities:
- Trace: trace(A') = trace(A)
- Determinant: det(A') = det(A)
- Rank: rank(A') = rank(A)
- Controllability and observability (as properties, though the matrices change form)

### Eigenvector Transformation

If v is an eigenvector of A, then T^{-1}v is the corresponding eigenvector of A' = T^{-1}AT.

## Diagonalization

If A has n linearly independent eigenvectors v_1, ..., v_n with eigenvalues lambda_1, ..., lambda_n, form T = [v_1 | v_2 | ... | v_n]. Then:

```
T^{-1} A T = D = diag(lambda_1, lambda_2, ..., lambda_n)
```

In diagonal form, the system decouples into n independent first-order equations:

```
z_i_dot = lambda_i * z_i + [T^{-1} B]_i u
```

Each mode evolves independently. The solution is:

```
z_i(t) = e^{lambda_i * t} z_i(0) + integral of e^{lambda_i * (t-tau)} [T^{-1}B]_i u(tau) dtau
```

**Limitation:** Not all matrices are diagonalizable. A matrix with repeated eigenvalues whose eigenspace dimension is less than the multiplicity requires Jordan form.

## Jordan Normal Form

When A is not diagonalizable, the closest to diagonal form is the Jordan normal form:

```
T^{-1} A T = J = block-diag(J_1, J_2, ..., J_k)
```

Each Jordan block J_i has eigenvalue lambda_i on the diagonal and 1s on the superdiagonal:

```
J_i = | lambda_i   1        0      |
      | 0          lambda_i  1     |
      | 0          0         lambda_i |
```

Jordan blocks introduce polynomial-times-exponential terms in the solution: t^k * e^{lambda * t}. For stability, even with Jordan blocks, Re(lambda) < 0 is still required — but marginally stable systems (Re(lambda) = 0) with Jordan blocks of size > 1 are unstable (the polynomial term grows).

**Practical note:** Jordan form is numerically ill-conditioned to compute. For numerical work, use Schur decomposition (A = QUQ^T where U is upper triangular and Q is orthogonal) instead. Jordan form is valuable for theoretical analysis, not computation.

## Controllable Canonical Form

For a single-input system (m = 1) that is controllable, there exists a transformation T_c that puts the system into controllable canonical form:

```
A_c = | 0    1    0   ...  0   |       B_c = | 0 |
      | 0    0    1   ...  0   |             | 0 |
      | :    :    :   ...  :   |             | : |
      | 0    0    0   ...  1   |             | 0 |
      | -a_0 -a_1 -a_2 ... -a_{n-1} |       | 1 |
```

The last row of A_c contains the negated coefficients of the characteristic polynomial:

```
det(sI - A) = s^n + a_{n-1} s^{n-1} + ... + a_1 s + a_0
```

**Why it matters:** In controllable canonical form, pole placement by state feedback u = -Kx becomes trivial. The feedback gain K directly sets the characteristic polynomial coefficients. Ackermann's formula and direct coefficient matching both exploit this form.

**Construction:** T_c = C_ctrl * W where C_ctrl = [B, AB, ..., A^{n-1}B] is the controllability matrix and W is derived from the characteristic polynomial coefficients.

## Observable Canonical Form

The dual of controllable canonical form. For a single-output (p = 1) observable system:

```
A_o = | 0    0    ...  0    -a_0     |       C_o = | 0  0  ...  0  1 |
      | 1    0    ...  0    -a_1     |
      | 0    1    ...  0    -a_2     |
      | :    :    ...  :     :       |
      | 0    0    ...  1    -a_{n-1} |
```

**Duality:** A system (A, B, C) is observable if and only if (A^T, C^T, B^T) is controllable. Observer design in observable canonical form mirrors pole placement in controllable canonical form.

## Modal Decomposition

When A is diagonalizable, the transformation to diagonal form is called modal decomposition. Each diagonal entry corresponds to a mode of the system:

```
x(t) = T z(t) = sum_{i=1}^{n} z_i(t) * v_i
```

where v_i are the eigenvectors (columns of T) and z_i(t) = e^{lambda_i t} z_i(0) is the i-th modal coordinate.

**Physical meaning for the ball-balancer:**
- Each mode represents an independent pattern of motion
- The unstable mode (positive eigenvalue) corresponds to the ball rolling off due to gravity
- Stable modes represent the controller's restoring dynamics
- Complex conjugate pairs represent oscillatory modes (the ball rocking back and forth)

**Modal participation:** The matrix T^{-1}B shows how the input u excites each mode. If row i of T^{-1}B is near zero, mode i is weakly controllable — the input cannot effectively influence that mode.

## Implementation Notes

### Diagonalization in Eigen

```cpp
#include <Eigen/Eigenvalues>

Eigen::EigenSolver<Eigen::MatrixXd> solver(A);
Eigen::MatrixXcd T = solver.eigenvectors();           // columns = eigenvectors
Eigen::VectorXcd lambdas = solver.eigenvalues();       // eigenvalues

// Verify: T^{-1} A T should be approximately diagonal
Eigen::MatrixXcd D = T.inverse() * A.cast<std::complex<double>>() * T;
// D should be diag(lambdas) up to numerical precision
```

For symmetric A, use `SelfAdjointEigenSolver` to get real-valued results with orthogonal T (see eigenvalues.md).

### Controllable Canonical Form Construction

```cpp
// Build controllability matrix: C_ctrl = [B, AB, A^2B, ..., A^{n-1}B]
int n = A.rows();
Eigen::MatrixXd C_ctrl(n, n);  // single-input: B is n x 1
Eigen::MatrixXd Ak = Eigen::MatrixXd::Identity(n, n);
for (int k = 0; k < n; k++) {
    C_ctrl.col(k) = (Ak * B).eval();
    Ak = (A * Ak).eval();  // .eval() prevents aliasing
}

// Check controllability
Eigen::ColPivHouseholderQR<Eigen::MatrixXd> qr(C_ctrl);
if (qr.rank() < n) {
    // System is NOT controllable — canonical form does not exist
}

// Characteristic polynomial coefficients
// For small systems, compute directly from eigenvalues
Eigen::EigenSolver<Eigen::MatrixXd> solver(A, false);
// Or use Eigen's polynomial utilities
```

### Similarity Transform Application

```cpp
// Apply transformation x = Tz to state-space matrices
// T must be invertible (check condition number first)
Eigen::MatrixXd T_inv = T.inverse();  // or use T.lu().solve(I) for stability

Eigen::MatrixXd A_new = T_inv * A * T;
Eigen::MatrixXd B_new = T_inv * B;
Eigen::MatrixXd C_new = C * T;
// D is unchanged
```

**Numerical caution:** If T is ill-conditioned, the transformation amplifies numerical errors. Check the condition number of T before applying:

```cpp
Eigen::JacobiSVD<Eigen::MatrixXd> svd(T);
double cond = svd.singularValues()(0) / svd.singularValues()(svd.singularValues().size() - 1);
if (cond > 1e10) {
    // T is ill-conditioned — transformed system may be numerically unreliable
}
```

### Schur Decomposition (Numerical Alternative to Jordan Form)

When Jordan form is needed for analysis but numerical stability matters:

```cpp
// Schur decomposition: A = Q U Q^T  (real Schur form)
Eigen::RealSchur<Eigen::MatrixXd> schur(A);
Eigen::MatrixXd Q = schur.matrixU();  // orthogonal
Eigen::MatrixXd U = schur.matrixT();  // quasi-upper triangular
// Diagonal blocks of U are 1x1 (real eigenvalue) or 2x2 (complex conjugate pair)
// Eigenvalues readable from the diagonal/blocks of U
```

Schur decomposition is always numerically stable (orthogonal T, no ill-conditioning) and reveals the same eigenvalue structure as Jordan form without the numerical hazards.
