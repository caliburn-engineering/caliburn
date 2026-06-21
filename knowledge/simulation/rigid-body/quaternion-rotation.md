---
title: Quaternion Rotation
sources:
  - { book: "Kuipers - Quaternions and Rotation Sequences" }
  - { note: "engineering experience" }
requires:
  - ../../math/linear-algebra/matrix-operations.md
related:
  - equations-of-motion.md
  - inertia-tensors.md
---

# Quaternion Rotation

Quaternions are a 4-component number system that provide the most numerically stable and singularity-free representation of 3D rotations. They avoid gimbal lock (the singularity in Euler angles at certain orientations), compose efficiently via quaternion multiplication, and integrate smoothly for tracking orientation over time.

In the ball-balancer project, quaternions are needed when tracking the ball's spin orientation as it rolls, when performing sensor fusion via quaternion-based Kalman filters, and for smooth interpolation of table tilt in rendering.

## Key Equations

### Definition

A quaternion is a 4-component hypercomplex number:

```
q = w + xi + yj + zk
```

where w is the **scalar part** and (x, y, z) is the **vector part**. The basis elements satisfy:

```
i^2 = j^2 = k^2 = -1
ij = k,  jk = i,  ki = j     (cyclic)
ji = -k, kj = -i, ik = -j    (anti-cyclic)
```

Quaternion multiplication is **associative** but **non-commutative**.

### Unit Quaternion Constraint

A rotation quaternion must have unit norm:

```
||q|| = sqrt(w^2 + x^2 + y^2 + z^2) = 1
```

### Quaternion from Axis-Angle

For a rotation of angle phi about unit axis u = (u_x, u_y, u_z):

```
q = cos(phi/2) + sin(phi/2) * (u_x*i + u_y*j + u_z*k)
```

Note the half-angle: a rotation by phi uses phi/2 in the quaternion. This means q and -q represent the **same rotation** (double cover of SO(3)).

### Rotation of a Vector

To rotate vector v by quaternion q, embed v as a pure quaternion (0, v) and compute:

```
v' = q * v * q*
```

where q* = (w, -x, -y, -z) is the quaternion conjugate. For unit quaternions, q* = q^{-1}.

Expanded form (Kuipers Eq. 5.9, avoids two full quaternion multiplications):

```
v' = (w^2 - |q_vec|^2) * v + 2*(q_vec . v)*q_vec + 2*w*(q_vec x v)
```

where q_vec = (x, y, z) is the vector part of q.

### Quaternion Multiplication (Composition of Rotations)

To apply rotation p first, then rotation q, the combined rotation is:

```
r = q * p       (right-to-left, like matrix multiplication)
```

Full product formula for p = (p_w, p_vec) and q = (q_w, q_vec):

```
r_w   = p_w * q_w   - p_vec . q_vec
r_vec = p_w * q_vec + q_w * p_vec + p_vec x q_vec
```

### Quaternion to Rotation Matrix

Given unit quaternion q = (w, x, y, z), the equivalent 3x3 rotation matrix is:

```
R = | 1-2(y^2+z^2)    2(xy-wz)        2(xz+wy)     |
    | 2(xy+wz)        1-2(x^2+z^2)    2(yz-wx)      |
    | 2(xz-wy)        2(yz+wx)        1-2(x^2+y^2)  |
```

### Rotation Matrix to Quaternion (Shepperd's Method)

```
w = 0.5 * sqrt(1 + R_11 + R_22 + R_33)
x = (R_32 - R_23) / (4*w)
y = (R_13 - R_31) / (4*w)
z = (R_21 - R_12) / (4*w)
```

When w is near zero, use Shepperd's method: compute the largest diagonal element first, extract that component, then derive the others. This avoids division by near-zero.

### Axis-Angle from Quaternion

```
phi = 2 * acos(w)
u   = (x, y, z) / sin(phi/2)     (undefined when phi = 0; use u = (1,0,0))
```

### Quaternion Derivative (for Integration)

Given angular velocity omega = (omega_x, omega_y, omega_z) in the **body frame**, the time derivative of the orientation quaternion is:

```
q_dot = 0.5 * q * [0, omega]
```

In matrix form:

```
| w_dot |       |  0     -omega_x  -omega_y  -omega_z | | w |
| x_dot | = 0.5 |  omega_x   0      omega_z  -omega_y | | x |
| y_dot |       |  omega_y  -omega_z   0       omega_x | | y |
| z_dot |       |  omega_z   omega_y  -omega_x   0     | | z |
```

This is a **linear ODE** in the quaternion components -- far simpler than the Euler angle kinematic equations, which involve trigonometric functions and are singular at gimbal lock.

### SLERP (Spherical Linear Interpolation)

To interpolate between orientations q_1 and q_2 with parameter t in [0, 1]:

```
SLERP(q_1, q_2; t) = sin((1-t)*Omega) / sin(Omega) * q_1 + sin(t*Omega) / sin(Omega) * q_2
```

where Omega = acos(q_1 . q_2) is the angle between the quaternions on S^3.

**Sign check:** If q_1 . q_2 < 0, negate one quaternion before interpolating to take the shorter arc (since q and -q are the same rotation).

**Small angle fallback:** When Omega is very small (q_1 ~ q_2), SLERP degenerates numerically. Fall back to normalised linear interpolation (NLERP):

```
q = normalize((1-t) * q_1 + t * q_2)
```

### Why Quaternions over Euler Angles

| Property | Euler Angles | Quaternions |
|----------|-------------|-------------|
| Parameters | 3 | 4 (1 constraint) |
| Singularity | Gimbal lock at theta = 0, pi | None |
| Composition | Expensive trigonometry | Single quaternion multiply |
| Interpolation | Non-smooth near singularity | SLERP -- smooth on S^3 |
| Numerical drift | Angles drift freely | Re-normalise: q = q / ||q|| |
| Memory | 3 doubles | 4 doubles |

## Implementation Notes

Eigen provides `Eigen::Quaterniond` with built-in operations:

```cpp
#include <Eigen/Geometry>

// From axis-angle
Eigen::Quaterniond q(Eigen::AngleAxisd(angle, Eigen::Vector3d::UnitZ()));

// Rotate a vector
Eigen::Vector3d v_rotated = q * v;   // operator* overloaded for vector rotation

// Compose rotations (apply q1 first, then q2)
Eigen::Quaterniond combined = q2 * q1;

// To rotation matrix
Eigen::Matrix3d R = q.toRotationMatrix();

// From rotation matrix
Eigen::Quaterniond q_from_R(R);

// SLERP
Eigen::Quaterniond q_interp = q1.slerp(t, q2);
```

Quaternion derivative for RK4 integration:

```cpp
Eigen::Vector4d quat_derivative(
    const Eigen::Quaterniond& q,
    const Eigen::Vector3d& omega)  // body-frame angular velocity
{
    Eigen::Matrix4d Xi;
    Xi <<  0,        -omega.x(), -omega.y(), -omega.z(),
           omega.x(),  0,         omega.z(), -omega.y(),
           omega.y(), -omega.z(),  0,         omega.x(),
           omega.z(),  omega.y(), -omega.x(),  0;
    Eigen::Vector4d q_vec(q.w(), q.x(), q.y(), q.z());
    return 0.5 * Xi * q_vec;
}
```

After every RK4 step, re-normalise to prevent quaternion drift:

```cpp
q.normalize();   // projects back onto the unit sphere S^3
```

Numerical stability considerations:

- Quaternion drift is slow (the derivative is tangent to S^3), so re-normalisation after each step is sufficient. Do not skip it.
- When converting from rotation matrix, always use Shepperd's method (check which diagonal element is largest) rather than the naive formula, to avoid division by near-zero.
- In sensor fusion (IMU quaternion Kalman filter), represent the error state as a 3-vector (rotation vector) rather than a full quaternion to keep the covariance matrix 3x3 (multiplicative extended Kalman filter / MEKF).
- For the ball-balancer, the table tilt quaternion stays near identity (small angles), so NLERP is often sufficient in place of full SLERP.
