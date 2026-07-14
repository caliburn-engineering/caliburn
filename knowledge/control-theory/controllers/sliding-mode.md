---
title: Sliding Mode Control
sources:
  - { book: "Slotine & Li — Applied Nonlinear Control", chapter: "7", pages: "276-310" }
  - { book: "Khalil — Nonlinear Systems", chapter: "14", pages: "552-580" }
  - { note: "engineering experience — Mercedes interview prep, traction control design" }
requires:
  - ../state-space.md
  - ../stability.md
related:
  - pid.md
  - lqr.md
  - ../design-problems/traction-control.md
reference: ../../../reference/controllers/smc.h
---

# Sliding Mode Control

Sliding Mode Control (SMC) is a nonlinear, variable-structure controller that achieves robustness by forcing the system state onto a user-defined sliding surface and keeping it there despite bounded matched disturbances. Unlike PID and LQR (which are linear), SMC handles nonlinear plants directly without linearization. It is the natural choice when the plant has significant model uncertainty, the disturbance bound is known, and robustness is the primary design goal.

## System Setup

SMC applies to systems of the form:

```
x_dot = f(x) + g(x)*u + d(t)
```

where:
- f(x), g(x) are known (possibly nonlinear) system dynamics
- u is the control input
- d(t) is a bounded matched disturbance: |d(t)| <= d_max

"Matched" means the disturbance enters through the same channel as the control input. SMC provides invariance only to matched disturbances.

## Sliding Surface Design

The sliding surface is a manifold in state space:

```
s(x) = 0
```

### Scalar Case (Tracking)

For a system with tracking error e = x - x_d:

```
s = e_dot + lambda * e       (lambda > 0)
```

When s = 0, the error dynamics reduce to e_dot = -lambda * e, giving guaranteed exponential convergence with time constant 1/lambda.

### General n-th Order Case

For an n-th order system:

```
s = (d/dt + lambda)^(n-1) * e
```

This defines an (n-1)-th order stable differential equation on the sliding surface. The designer chooses lambda to set the convergence rate during the sliding phase.

### Design Freedom

The sliding surface encodes the desired closed-loop dynamics. Once the state reaches s = 0 and stays there, the system behaves as if it were the reduced-order system defined by s = 0 — regardless of disturbances.

## Control Law

The control is split into two components:

```
u = u_eq + u_sw
```

### Equivalent Control (u_eq)

The model-based feedforward term that would keep s_dot = 0 if there were no disturbance:

```
u_eq solves: s_dot|_{d=0} = 0
```

This cancels the known dynamics along the sliding surface.

### Switching Control (u_sw)

The discontinuous term that enforces the reaching condition:

```
u_sw = -K * sign(s)       K > d_max
```

The gain K must exceed the worst-case disturbance magnitude to guarantee reaching.

### Reaching Condition (Lyapunov Proof)

Choose V = 0.5 * s^2 as a Lyapunov candidate:

```
V_dot = s * s_dot < 0    for all s != 0
```

This guarantees finite-time convergence to s = 0. The reaching time is bounded by:

```
t_reach <= |s(0)| / (K - d_max)
```

## Two Phases of Motion

### Reaching Phase

The state is driven toward the sliding surface s = 0. During this phase, the system is NOT invariant to disturbances — performance depends on the initial condition and switching gain K.

### Sliding Phase

Once s = 0 is reached, the state remains on the surface (invariant to matched disturbances). The system dynamics reduce to those defined by the sliding surface equation. This is the robustness guarantee of SMC.

## Chattering Problem and Solutions

The discontinuous sign(s) term causes high-frequency switching (chattering) in the control signal. This excites unmodelled high-frequency dynamics and causes actuator wear.

### 1. Boundary Layer (Saturation Approximation)

Replace sign(s) with a continuous approximation:

```
sat(s/Phi) = { s/Phi   if |s| <= Phi
             { sign(s)  if |s| > Phi
```

where Phi is the boundary layer thickness. This trades exact sliding (invariance) for a smooth control signal. The tracking error is bounded by O(Phi) instead of zero.

### 2. Super-Twisting Algorithm (2nd-Order SMC)

A continuous control law that still achieves finite-time convergence:

```
u = -k1 * |s|^(1/2) * sign(s) + v
v_dot = -k2 * sign(s)
```

Properties:
- Control signal u is continuous (no chattering)
- Finite-time convergence to s = 0 and s_dot = 0 simultaneously
- Requires only s (not s_dot) for implementation
- Gains must satisfy: k1 > 0, k2 > sufficient margin over disturbance derivative bound

### 3. Higher-Order SMC

Drive s, s_dot, s_ddot, ... all to zero simultaneously. Provides smoother control at the cost of increased complexity and stronger observability requirements.

## When to Choose SMC

| Choose when | Avoid when |
|---|---|
| Plant is nonlinear or has large uncertainty | Plant is well-modelled and linear (LQR/PID simpler) |
| Robustness to disturbances is critical | Actuator bandwidth is limited (chattering) |
| Finite-time convergence needed | Optimality needed (SMC is robust, not optimal) |
| Matched disturbances dominate | Unmatched disturbances dominate |
| Disturbance bound is known or estimable | No knowledge of disturbance magnitude |

## Comparison with PID and LQR

| Property | PID | LQR | SMC |
|---|---|---|---|
| Plant type | Linear SISO | Linear MIMO | Nonlinear |
| Model requirement | Minimal | Full state-space | Approximate + disturbance bound |
| Robustness | Limited (fixed gains) | Guaranteed margins (for nominal model) | Invariance to matched disturbances |
| Optimality | None | Quadratic cost optimal | Not optimal |
| Convergence | Asymptotic | Asymptotic | Finite-time |
| Tuning parameters | Kp, Ki, Kd | Q, R matrices | lambda, K, Phi (or k1, k2) |

## Automotive Applications

- **Traction control** — tyre curve is nonlinear, road friction (mu) is uncertain. SMC targets optimal slip ratio despite unknown surface.
- **ABS** — keep wheel slip near the peak of the mu-slip curve despite unknown road surface and load transfer.
- **ESP/ESC** — yaw rate tracking with uncertain tyre parameters and changing load conditions.
- **EV motor control** — current/torque tracking with parameter variation due to temperature and saturation.

## Implementation Considerations

### Disturbance Bound Estimation

K must exceed d_max. In practice:
- Use a conservative estimate with safety margin: K = 1.2 * d_max_estimated
- Too large K increases chattering and control effort
- Too small K violates the reaching condition (system never reaches sliding surface)

### Discrete-Time Implementation

In discrete time, true sliding cannot occur (the state crosses s = 0 between samples). Use:
- Boundary layer with Phi >= control period * max(|s_dot|)
- Or quasi-sliding mode: define a band |s| < delta where the state oscillates

### State Estimation

If the full state is not measured, pair SMC with a robust observer (e.g., sliding mode observer). Standard Kalman filters may not be appropriate since the system is nonlinear.

### Matched vs Unmatched Disturbances

SMC only rejects disturbances entering through the control channel. For unmatched disturbances:
- Use hierarchical/cascaded SMC design
- Or combine SMC with integral action for steady-state rejection
- Or accept bounded tracking error proportional to the unmatched disturbance
