---
title: Matrix Operations
sources:
  - { book: "Kreyszig - Advanced Engineering Mathematics", chapter: "7" }
  - { book: "Fedorov - Yet Another Introduction to Numerical Methods", chapter: "5", pages: "47-56" }
  - { note: "engineering experience" }
requires: []
related:
  - eigenvalues.md
  - state-space-transforms.md
---

# Matrix Operations

Matrix operations are the computational backbone of every control system, sensor fusion pipeline, and physics simulation in the ball-balancer project. This file covers the operations themselves, the decompositions that make them numerically viable, and the Eigen patterns that implement them.

## Multiplication, Transposition, Inversion

**Matrix multiplication** composes linear maps. Entry (i,j) of AB equals the dot product of row i of A with column j of B. Multiplication is associative but not commutative: AB != BA in general.

**Transposition** swaps rows and columns: (A^T)_{ij} = A_{ji}. Key identities:
- (AB)^T = B^T A^T
- (A^T)^T = A
- (A^{-1})^T = (A^T)^{-1}

**Inversion** exists only for square matrices with det(A) != 0. The inverse satisfies AA^{-1} = A^{-1}A = I. Key identity: (AB)^{-1} = B^{-1}A^{-1}.

**Never compute the inverse explicitly to solve Ax = b.** Use a decomposition instead. Explicit inversion is O(n^3) and numerically unstable compared to factorization-based solves.

## Key Equations

Solving the linear system Ax = b:

```
Ax = b  =>  x = A^{-1} b    (conceptually, never compute A^{-1})
```

Determinant (for invertibility checks only):

```
det(A) = sum over permutations sigma of sgn(sigma) * prod_{i=1}^{n} a_{i,sigma(i)}
```

Effect of elementary row operations on det:

| Operation                        | Effect on det     |
|----------------------------------|-------------------|
| Swap two rows                    | det -> -det       |
| Multiply row by scalar lambda    | det -> lambda*det |
| Add multiple of one row to another | det unchanged   |

For diagonal or triangular matrices: det = product of diagonal entries.

## Decompositions

### LU Decomposition

Factors A = LU where L is lower triangular and U is upper triangular. With partial pivoting: PA = LU.

**When to use:** General square systems, especially when solving Ax = b for multiple right-hand sides b. Factor once, solve many times.

**Solving Ax = b via LU:**
1. Factor A = LU (with pivoting)
2. Solve Lw = Pb by forward substitution (O(n^2))
3. Solve Ux = w by back substitution (O(n^2))

### QR Decomposition

Factors A = QR where Q is orthogonal (Q^T Q = I) and R is upper triangular. Constructed via Gram-Schmidt orthogonalization or Householder reflections.

**When to use:** Least-squares problems, rank determination, ill-conditioned systems where LU may lose accuracy. Also the basis of the QR algorithm for eigenvalue computation.

**Solving Ax = b via QR:**
```
Ax = QRx = b  =>  Rx = Q^T b    (triangular solve)
```

**Rank determination:** The number of non-zero diagonal entries of R equals rank(A). Use column-pivoted QR for reliable numerical rank.

### Cholesky Decomposition

Factors A = LL^T where L is lower triangular. Only works for symmetric positive definite matrices.

**When to use:** Covariance matrices, Kalman filter update steps, any SPD system. Roughly twice as fast as LU and guaranteed stable for SPD matrices.

### Which Decomposition When

| Situation | Decomposition | Reason |
|---|---|---|
| General square Ax = b | LU (partial pivot) | Fast, stable enough for well-conditioned systems |
| Multiple right-hand sides | LU | Factor once, solve O(n^2) per RHS |
| Ill-conditioned or near-singular | QR (column pivot) | Better numerical stability |
| Symmetric positive definite | Cholesky | 2x faster than LU, guaranteed stable |
| Rank determination | QR (column pivot) | Reliable numerical rank from R diagonal |
| Least-squares (overdetermined) | QR | Natural formulation for min ||Ax - b||^2 |
| Eigenvalue computation | QR algorithm | Iterative QR factorizations converge to eigenvalues |

## Implementation Notes

### Solving Ax = b in Eigen

```cpp
#include <Eigen/Dense>

// LU — general square systems (default choice)
Eigen::PartialPivLU<Eigen::MatrixXd> lu(A);
Eigen::VectorXd x = lu.solve(b);

// QR — more stable for ill-conditioned systems, also gives rank
Eigen::ColPivHouseholderQR<Eigen::MatrixXd> qr(A);
Eigen::VectorXd x = qr.solve(b);
int rank = qr.rank();

// Cholesky — symmetric positive definite only (covariance, K filter)
Eigen::LLT<Eigen::MatrixXd> llt(A);
Eigen::VectorXd x = llt.solve(b);
```

### Never Compute the Inverse

```cpp
// WRONG — numerically unstable, wasteful
Eigen::MatrixXd x = A.inverse() * b;

// CORRECT — use a decomposition
Eigen::VectorXd x = A.lu().solve(b);
```

### The auto Pitfall

Never use `auto` with Eigen expressions. Eigen uses expression templates (lazy evaluation), so `auto` captures an unevaluated expression tree, not the result matrix. This causes aliasing bugs and dangling references.

```cpp
// WRONG — expr is a lazy expression template, not a matrix
auto expr = A * B;
A = expr;  // reads A while writing A — undefined behavior

// CORRECT — force evaluation into a concrete matrix
Eigen::MatrixXd result = A * B;
```

### Aliasing and .eval()

When the output matrix appears on the right-hand side, call `.eval()` to force evaluation into a temporary before assignment:

```cpp
// WRONG — undefined behavior (reads and writes A simultaneously)
A = A * B;

// CORRECT — .eval() forces a temporary
A = (A * B).eval();

// When you KNOW there is no aliasing, skip the temporary for performance:
C.noalias() = A * B;
```

### Determinant — Use Sparingly

```cpp
// Only for invertibility checks, never for solving systems
double d = A.determinant();
bool invertible = std::abs(d) > 1e-12;  // threshold depends on scale
```

### Condition Number Check

For the ball-balancer, sensor noise can produce near-singular matrices. Always check condition before trusting a solve:

```cpp
Eigen::JacobiSVD<Eigen::MatrixXd> svd(A);
double cond = svd.singularValues()(0) / svd.singularValues()(svd.singularValues().size() - 1);
if (cond > 1e12) {
    // Matrix is ill-conditioned — solution may be unreliable
}
```
