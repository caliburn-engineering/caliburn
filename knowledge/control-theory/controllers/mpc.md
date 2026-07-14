---
title: Model Predictive Control (MPC)
sources:
  - { book: "Borrelli, Bemporad, Morari — Predictive Control for Linear and Hybrid Systems", chapter: "1-5" }
  - { book: "Rawlings, Mayne, Diehl — Model Predictive Control", chapter: "1-3" }
requires:
  - lqr.md
  - ../state-space.md
  - ../stability.md
related:
  - gain-scheduling.md
  - comparison.md
  - ../trajectory-planning.md
  - ../design-problems/hybrid-torque-split.md
---

# Model Predictive Control (MPC)

MPC solves an optimisation problem at every timestep:

```
At time k, given current state x(k):

Minimise: J = sum_{i=0}^{N-1} [ x(k+i)^T Q x(k+i) + u(k+i)^T R u(k+i) ]
              + x(k+N)^T P_f x(k+N)

Subject to:
  x(k+i+1) = A*x(k+i) + B*u(k+i)     (model prediction)
  u_min <= u(k+i) <= u_max             (input constraints)
  x_min <= x(k+i) <= x_max             (state constraints)

Apply only u(k|k) (first step), then re-solve at k+1.
```

This is **receding-horizon** control: optimise over a finite window, apply one step, shift forward, repeat.

## Why MPC Exists (What Makes It Different)

| Feature | LQR | MPC |
|---|---|---|
| Constraints | Cannot handle directly | Handles constraints explicitly |
| Preview | No (reacts to current state) | Yes (can use known future reference/disturbance) |
| Computation | Offline (solve Riccati once) | Online (solve QP at every timestep) |
| Optimality | Infinite-horizon optimal (unconstrained) | Finite-horizon optimal (constrained) |

**MPC earns its complexity when constraints are active.** If constraints never bind, unconstrained MPC is equivalent to LQR.

## Key Design Choices

| Parameter | Effect | Trade-off |
|---|---|---|
| Horizon N | Longer means more optimal, sees further ahead | More computation, sensitivity to model error over long predictions |
| Q, R weights | Same as LQR (state penalty vs effort penalty) | Same trade-off: aggressive vs gentle |
| Terminal cost P_f | Ensures stability beyond horizon | Typically set to LQR cost-to-go (infinite-horizon tail) |
| Sample time | Faster means better disturbance rejection | Must solve QP within one sample period |

## Automotive MPC Applications

| System | Why MPC | Key constraints |
|---|---|---|
| **ACC/ADAS** | Safety distance constraint, speed limits, comfort bounds (max accel/jerk) | u: throttle/brake limits. x: minimum headway distance. |
| **Hybrid energy management** | SoC bounds, thermal limits, route preview | u: torque split. x: battery SoC in [20%, 80%], T_battery < T_max. |
| **Trajectory planning** | Lane boundaries, curvature limits, dynamic obstacles | u: steering rate limit. x: stay within lane, respect tyre force limits. |
| **Torque vectoring** | Per-motor torque limits, total force constraint, tyre force circle | u: individual motor torques. x: yaw rate bounds. |
| **Thermal management** | Coolant flow limits, power dissipation constraints | u: pump/fan commands. x: T_component < T_derate. |

## MPC vs Other Methods

| Choose MPC when | Don't choose MPC when |
|---|---|
| Hard constraints on states or inputs are critical | System is simple SISO with no constraints |
| Future reference or disturbance is known (preview) | Computational budget is extremely limited (bare-metal ECU at >1 kHz) |
| MIMO system with cross-coupled constraints | A simpler method (PID, LQR) meets all specs |
| You need explicit constraint satisfaction guarantees | Model is too uncertain for meaningful prediction |

## Relationship to LQR

Unconstrained linear MPC with infinite horizon = LQR. They solve the same optimisation — MPC just does it online with a finite window and constraints.

The terminal cost P_f = solution to discrete Riccati equation (same P as in LQR). This ensures the "tail" beyond the horizon is implicitly optimal.

## Implementation Notes

- Standard MPC for linear systems with quadratic cost produces a **Quadratic Program (QP)**
- QP solvers: OSQP, qpOASES, HPIPM (all open-source, C/C++)
- For the ball-balancer: MPC would let you enforce plate angle limits (plus/minus 15 degrees) and ball position bounds explicitly — LQR cannot do this natively
- Typical automotive MPC runs at 10-100 Hz (10-100ms sample time)
- Nonlinear MPC (NMPC) for nonlinear plants — much heavier (requires NLP solver)

## Ball-Balancer Considerations

The existing ball-balancer project uses LQR. An MPC version could:
- Enforce plate angle limits (plus/minus 15 degrees) explicitly
- Enforce ball position bounds (keep ball on plate)
- Use preview: if reference trajectory is known, optimise ahead
- Compare LQR (fast, no constraints) vs MPC (slower, respects constraints)
