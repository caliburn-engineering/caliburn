---
title: LQG — Linear Quadratic Gaussian Control
sources:
  - { book: "Skogestad & Postlethwaite — Multivariable Feedback Control", chapter: "9" }
  - { book: "Ogata — Modern Control Engineering", chapter: "12" }
requires:
  - lqr.md
  - ../observers/kalman-filter.md
  - ../state-space.md
related:
  - pid.md
  - sliding-mode.md
  - comparison.md
---

# LQG — Linear Quadratic Gaussian Control

LQG combines LQR (optimal state-feedback) with a Kalman filter (optimal state estimator) to handle the case where the full state is not directly measurable. The combination is justified by the separation principle: the controller and observer can be designed independently without affecting each other's optimality.

## Core Idea

```
LQG = LQR + Kalman Filter (combined via separation principle)
```

- LQR provides optimal gain K given Q, R (control cost weights)
- Kalman provides optimal observer gain L given Qn, Rn (noise covariances)

## Design Procedure (LTI Systems)

1. **Design K** assuming full state is available (standard LQR: choose Q, R, solve ARE)
2. **Design L** assuming the controller already exists (standard Kalman: choose Qn, Rn, solve dual ARE)
3. **Combine:** `u = -K * x_hat` where x_hat comes from the Kalman filter

The control law uses the estimated state rather than the true state.

## Closed-Loop Poles

```
Closed-loop poles = {eig(A - BK)} union {eig(A - LC)}
```

The controller poles (from LQR) and observer poles (from Kalman) appear independently in the closed-loop characteristic polynomial. This is the mathematical manifestation of the separation principle.

## Separation Principle

The separation principle states that for LTI systems with Gaussian noise:
- The optimal controller gain K does not depend on the observer design
- The optimal observer gain L does not depend on the controller design
- The combined system (LQG) inherits optimality from both subsystems

**WARNING:** The separation principle does NOT hold for nonlinear systems. For nonlinear plants, controller and observer interact and must be co-designed.

## Key Limitation: Robustness

LQG has **no guaranteed robustness margins**.

Standalone LQR has excellent robustness properties:
- Infinite gain margin
- Phase margin >= 60 degrees

These guarantees are **lost** when the Kalman filter is introduced. The LQG closed-loop can have arbitrarily small gain and phase margins depending on the observer design.

This fundamental limitation motivated the development of:
- **H-infinity control** — designs for worst-case robustness explicitly
- **LQG/LTR (Loop Transfer Recovery)** — recovers LQR margins by driving Kalman filter bandwidth high
- **Mu-synthesis** — structured robust control for multiple uncertainty sources

## When to Use LQG

| Use when | Avoid when |
|---|---|
| Plant is linear and well-modelled | Plant is highly nonlinear |
| Not all states are directly measurable | All states are measurable (use LQR directly) |
| Sensor noise characteristics are known | Robustness margins are the primary concern |
| Optimality for nominal model is sufficient | Must guarantee performance under large uncertainty |

## Relationship to Other Methods

- **LQR:** LQG without the observer — use when full state is available
- **Kalman filter alone:** State estimation without control — the observation half of LQG
- **H-infinity:** When LQG's lack of robustness guarantees is unacceptable
- **PID + observer:** A simpler alternative if the plant is SISO and tuning rules suffice

## Ball-Balancer Application

The ball-balancer's LQR design already pairs with a Kalman filter for velocity estimation (velocities are not directly measured by the camera). This combination IS LQG:
- K from the LQR design (weighting position tracking vs servo effort)
- L from the Kalman filter (weighting model trust vs measurement noise)
- Separation principle applies because the linearized model is LTI near the operating point
