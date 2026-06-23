---
title: Homogeneous Transformation Matrices
sources:
  - { book: "Kuipers - Quaternions and Rotation Sequences", chapter: "6" }
  - { book: "Rigid Body Dynamics (Borisov &amp; Mamaev)", chapter: "1" }
  - { note: "engineering experience" }
requires:
  - ../linear-algebra/matrix-operations.md
related:
  - ../../simulation/rigid-body/quaternion-rotation.md
  - ../../simulation/rigid-body/equations-of-motion.md
  - ../../rendering/opengl-pipeline.md
reference: ../../../reference/math/homogeneous_transform.h
---

# Homogeneous Transformation Matrices

A homogeneous transformation matrix is a 4×4 matrix that encodes both rotation and translation in a single operation. This is the standard representation for rigid body pose (position + orientation) in robotics, computer graphics, and simulation.

Distinct from quaternion rotation (which encodes orientation only without translation), homogeneous transforms represent full 6-DOF pose and compose via matrix multiplication.

## The 4×4 Transform Matrix

```
T = | R  t |
    | 0  1 |
```

where:
- R is a 3×3 rotation matrix (orthogonal, det R = 1)
- t is a 3×1 translation vector
- The bottom row [0 0 0 1] enables matrix multiplication to compose transforms

A point p in the local frame maps to the world frame as:

```
p_world = T * [p; 1]
```

The homogeneous coordinate (the trailing 1) is what makes translation expressible as matrix multiplication.

## Elementary Transforms

### Rotation about X-axis

```
Rx(theta) = | 1    0        0      0 |
            | 0    cos(θ)  -sin(θ)  0 |
            | 0    sin(θ)   cos(θ)  0 |
            | 0    0        0      1 |
```

### Rotation about Y-axis

```
Ry(theta) = |  cos(θ)  0  sin(θ)  0 |
            |  0       1  0       0 |
            | -sin(θ)  0  cos(θ)  0 |
            |  0       0  0       1 |
```

### Rotation about Z-axis

```
Rz(theta) = | cos(θ)  -sin(θ)  0  0 |
            | sin(θ)   cos(θ)  0  0 |
            | 0        0       1  0 |
            | 0        0       0  1 |
```

### Pure Translation

```
Trans(tx, ty, tz) = | 1  0  0  tx |
                    | 0  1  0  ty |
                    | 0  0  1  tz |
                    | 0  0  0  1  |
```

## Frame Chains

In the ball-balancer, the coordinate frame chain is:

```
World → Plate → Ball → Sensor
  T_plate     T_ball    T_sensor
```

A point in the sensor frame maps to world coordinates as:

```
p_world = T_plate * T_ball * T_sensor * [p_sensor; 1]
```

### T_plate (world → plate)

```
T_plate = Trans(0, 0, h_pivot) * Rx(alpha) * Ry(beta)
```

where h_pivot is the pivot height above ground, and alpha/beta are the servo-driven tilt angles.

### T_ball (plate → ball)

```
T_ball = Trans(ball_x, ball_y, R_ball)
```

The ball sits on the plate surface at height R_ball (its radius), at position (ball_x, ball_y) in the plate frame.

### Composing the Chain

The ball's world position:

```
p_ball_world = T_plate * T_ball * [0; 0; 0; 1]
             = T_plate * [ball_x; ball_y; R_ball; 1]
```

This is how the renderer computes the ball's world-space position for OpenGL: take the plate's model matrix (T_plate), transform the ball's plate-local position through it.

## Inverse Transform

The inverse of a homogeneous transform is:

```
T^{-1} = | R^T   -R^T * t |
         | 0         1     |
```

This is much cheaper than general 4×4 matrix inversion because R is orthogonal (R^{-1} = R^T).

Use case: given a point in world coordinates, find its coordinates in the plate frame:

```
p_plate = T_plate^{-1} * [p_world; 1]
```

The Kalman filter uses this when converting ball position measurements (from a camera in world frame) to the plate frame where the dynamics are defined.

## Relation to Other Representations

| Representation | Encodes | DOF | Composition |
|---|---|---|---|
| Rotation matrix (3×3) | Orientation only | 3 | Matrix multiply |
| Quaternion | Orientation only | 3 (4 params, 1 constraint) | Quaternion multiply |
| Homogeneous transform (4×4) | Full pose (rotation + translation) | 6 | Matrix multiply |
| Euler angles | Orientation only | 3 | Compose via successive rotations |

Quaternions avoid gimbal lock and interpolate smoothly (SLERP), but don't encode translation. Homogeneous transforms encode full pose but are less efficient for pure rotation interpolation. In practice, store orientation as quaternion and compose full pose as needed using homogeneous transforms.

## Implementation Notes

The reference implementation provides functions to:
- Construct elementary rotation and translation transforms
- Compose transforms via multiplication
- Compute the efficient inverse (using R^T, not general inverse)
- Transform points and vectors (vectors ignore translation)
- Extract rotation and translation from a transform

All functions operate on `Eigen::Matrix4d`.
