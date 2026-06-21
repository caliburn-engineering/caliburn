---
title: Inertia Tensors
sources:
  - { note: "engineering experience" }
requires:
  - ../../math/linear-algebra/matrix-operations.md
  - ../../math/linear-algebra/eigenvalues.md
related:
  - equations-of-motion.md
---

# Inertia Tensors

The moment of inertia tensor I is a 3x3 symmetric positive-definite matrix that characterises how a rigid body's mass is distributed relative to a reference point. It maps angular velocity to angular momentum (L = I * omega) and appears in the rotational equations of motion. Computing I correctly is essential for accurate rigid body simulation.

## Key Equations

### Definition

For a rigid body with mass distribution rho(r), the inertia tensor about the origin is:

```
I_ij = integral( (r^2 * delta_ij - r_i * r_j) dm )
```

### Explicit Components

The diagonal entries are the **moments of inertia** and the off-diagonal entries are the (negated) **products of inertia**:

```
I_xx = integral( (y^2 + z^2) dm )
I_yy = integral( (x^2 + z^2) dm )
I_zz = integral( (x^2 + y^2) dm )

I_xy = I_yx = -integral( x*y dm )
I_xz = I_zx = -integral( x*z dm )
I_yz = I_zy = -integral( y*z dm )
```

The tensor is always symmetric: I = I^T.

### Principal Axes and Principal Moments

Because I is real and symmetric, it can be diagonalised by an orthogonal transformation:

```
I_principal = P^T * I * P = diag(A, B, C)
```

where A <= B <= C are the **principal moments of inertia** and the columns of P are the **principal axes** (eigenvectors of I). In principal axes, all products of inertia vanish.

The principal moments satisfy the triangle inequalities (necessary for a physical body):

```
A + B >= C
B + C >= A
A + C >= B
```

### Parallel Axis Theorem (Steiner)

To shift the reference point from the centre of mass to a parallel frame displaced by d:

```
I'_ij = (I_cm)_ij + M * (|d|^2 * delta_ij - d_i * d_j)
```

In compact form:

```
I' = I_cm + M * (d^T * d * E_3 - d * d^T)
```

where E_3 is the 3x3 identity matrix. This is used when the rotation axis does not pass through the centre of mass.

### Common Shapes

| Shape | I_xx | I_yy | I_zz | Notes |
|-------|------|------|------|-------|
| Uniform sphere (radius R, mass M) | 2/5 MR^2 | 2/5 MR^2 | 2/5 MR^2 | Isotropic |
| Solid cylinder (radius R, height h, about symmetry axis z) | M(3R^2 + h^2)/12 | M(3R^2 + h^2)/12 | MR^2/2 | Symmetric top |
| Thin rectangular plate (a x b, about centre, normal z) | Mb^2/12 | Ma^2/12 | M(a^2 + b^2)/12 | Ball-balancer plate |
| Thin rod (length L, about centre, along x) | 0 | ML^2/12 | ML^2/12 | Simplified link |

For the ball-balancer: the tilting plate is a thin rectangular plate with I_zz = M(a^2 + b^2)/12 about its centre. The ball is a uniform sphere with I = 2/5 MR^2 (giving the 5/7 rolling factor in the equations of motion).

### Composite Bodies

For a body composed of N sub-bodies, the total inertia tensor about a common point is the sum:

```
I_total = sum_{k=1}^{N} I_k
```

where each I_k is the inertia tensor of sub-body k about the common point (use the parallel axis theorem to shift each sub-body's I_cm to the common reference).

### Inertia Ellipsoid

The inertia ellipsoid is defined by:

```
r^T * I * r = 1
```

Its semi-axes are 1/sqrt(A), 1/sqrt(B), 1/sqrt(C) along the principal axes. The angular momentum vector L = I*omega is perpendicular to the ellipsoid surface at the point where omega touches it. This geometric picture (Poinsot's construction) is useful for understanding torque-free motion and stability.

**Stability of rotation about principal axes:**
- Rotation about the axis with largest I (C) is **stable**.
- Rotation about the axis with smallest I (A) is **stable**.
- Rotation about the **intermediate** axis (B) is **unstable** (tennis racket theorem / Dzhanibekov effect).

## Implementation Notes

Representing the inertia tensor in Eigen:

```cpp
// Full 3x3 inertia tensor
Eigen::Matrix3d I;
I << Ixx, Ixy, Ixz,
     Ixy, Iyy, Iyz,
     Ixz, Iyz, Izz;

// Principal axis decomposition
Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> solver(I);
Eigen::Vector3d principal_moments = solver.eigenvalues();  // A, B, C
Eigen::Matrix3d principal_axes = solver.eigenvectors();     // columns = axes
```

For bodies aligned with principal axes (which is the common case in simulation), store I as a `Vector3d` of diagonal entries and use element-wise operations:

```cpp
// Diagonal-only storage (principal axes aligned with body frame)
Eigen::Vector3d I_diag{Ixx, Iyy, Izz};

// Euler's equations with diagonal I
Eigen::Vector3d omega_dot;
omega_dot.x() = (torque.x() + (I_diag.y() - I_diag.z()) * omega.y() * omega.z()) / I_diag.x();
omega_dot.y() = (torque.y() + (I_diag.z() - I_diag.x()) * omega.z() * omega.x()) / I_diag.y();
omega_dot.z() = (torque.z() + (I_diag.x() - I_diag.y()) * omega.x() * omega.y()) / I_diag.z();
```

Parallel axis theorem in code:

```cpp
Eigen::Matrix3d parallel_axis_shift(
    const Eigen::Matrix3d& I_cm,
    double mass,
    const Eigen::Vector3d& d)
{
    return I_cm + mass * (d.squaredNorm() * Eigen::Matrix3d::Identity() - d * d.transpose());
}
```

Numerical considerations:

- Always verify the triangle inequality after computing I: a violation indicates a modelling error.
- For thin or nearly-symmetric bodies, two eigenvalues may be very close. Use `SelfAdjointEigenSolver` (not the general `EigenSolver`) for guaranteed real eigenvalues and numerical stability.
- When composing sub-bodies, accumulate I about a single common reference point to avoid error accumulation from repeated parallel-axis shifts.
