---
title: Controller Selection Guide
sources:
  - { note: "engineering experience — synthesis of methods for design decisions" }
requires:
  - pid.md
  - lqr.md
  - sliding-mode.md
  - mpc.md
  - gain-scheduling.md
related:
  - ../stability.md
  - ../frequency-response.md
  - ../design-framework.md
  - lqg.md
  - h-infinity.md
---

# Controller Selection Guide

A comparison of feedback control methods and a decision tree for choosing the right one. This file is the primary reference for the `design-controller` skill's candidate evaluation step.

## Comparison Table

| Method | Feedback | Design basis | Best for | Limitations |
|---|---|---|---|---|
| PID | Output error | Heuristic / tuning rules | SISO, simple plants | No state awareness, struggles with MIMO |
| Pole placement | Full state | Desired pole locations | When you know exactly where poles should be | Manual, no optimality guarantee |
| LQR | Full state | Optimal (minimise J) | MIMO, multiple objectives, trade-off tuning | Assumes full state access, needs linear model |
| LQG | Estimated state | LQR + Kalman | When states aren't all measured | No guaranteed robustness margins |
| SMC | Full state or output | Lyapunov / sliding surface | Nonlinear, uncertain plants, robustness-critical | Chattering, only rejects matched disturbances |
| H-infinity | Output | Worst-case robust | Uncertain plants, robust performance guarantee | Complex design, conservative |
| MPC | Full state (predicted) | Receding-horizon optimisation | Constraints, preview, MIMO | Computational cost, needs model |

## Selection Decision Tree

```
Is the plant linear and well-modelled?
├── Yes → Is it SISO with simple dynamics?
│   ├── Yes → PID (simplest, widely understood, easy to tune)
│   └── No → Are all states measurable?
│       ├── Yes → LQR (optimal, per-state tuning via Q/R)
│       └── No → LQG (LQR + Kalman observer)
│           └── Need robustness guarantee? → H-infinity
├── No (nonlinear or uncertain) →
│   ├── Robustness to disturbances is primary goal? → SMC
│   ├── Hard constraints on states/inputs? → MPC
│   └── Operating point varies significantly? → Gain-scheduled PID or gain-scheduled LQR
```

## Selection Criteria Questions

Use these questions to navigate the decision tree:

1. **Is the plant linear?** If no, consider SMC or gain-scheduling.
2. **Are all states measurable?** If no, need an observer (LQG) or output feedback (H-infinity).
3. **Are there hard constraints on states/inputs?** If yes, MPC is the natural choice.
4. **Is robustness to model uncertainty the primary concern?** If yes, SMC or H-infinity.
5. **Is the system SISO or MIMO?** SISO with simple dynamics favours PID; MIMO favours LQR/LQG/MPC.

## When Simplicity Wins

The decision tree biases toward complexity. Counter-balance with this heuristic:

> **Always prefer the simplest controller that meets requirements.** If PID works with adequate margins, do not reach for LQR. If LQR works without an observer, do not reach for LQG.

Complexity costs: implementation effort, debugging difficulty, more failure modes, harder certification.

## Full Design Methodology

Controller selection (this file) is Step 4 of the broader [6-step control design framework](../design-framework.md). The framework adds upstream steps (requirements clarification, plant modelling, controllability/observability assessment) and downstream steps (addressing hard parts, validation strategy) that surround the architecture choice.

For worked examples applying the full framework to automotive problems, see:

- [Traction Control](../design-problems/traction-control.md) — nonlinear tyre curve, gain-scheduled/SMC architecture
- [Active Suspension](../design-problems/active-suspension.md) — semi-active constraint, skyhook damping
- [Hybrid Torque Split](../design-problems/hybrid-torque-split.md) — optimisation-based energy management
