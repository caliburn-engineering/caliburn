---
title: H-infinity Robust Control (Overview)
sources:
  - { book: "Skogestad & Postlethwaite — Multivariable Feedback Control", chapter: "9" }
  - { note: "overview only — full design procedure deferred" }
requires:
  - lqr.md
  - ../frequency-response.md
  - ../stability.md
related:
  - lqg.md
  - comparison.md
---

# H-infinity Robust Control (Overview)

H-infinity control designs for worst-case robustness. Where LQR optimises average performance (quadratic cost) and LQG assumes Gaussian statistics, H-infinity minimises the worst-case amplification from disturbances to performance outputs.

## Objective

Minimise the H-infinity norm of a weighted closed-loop transfer function:

```
||T_zw||_inf = max_w sigma_max(T_zw(jw))
```

In plain English: find a controller that keeps the worst-case amplification from disturbances to performance outputs below a threshold gamma.

## Key Properties

- **Guarantees robust performance** under bounded model uncertainty
- **Works with output feedback** — does not need full state measurement
- **More conservative than LQR** — optimises for worst case, not average case
- **Design requires weighting function selection** — this is the art of H-infinity design

## When to Use

- Plant model has significant uncertainty bounds
- Must guarantee performance across an operating range
- LQG robustness margins are insufficient
- The system must be certified against worst-case scenarios

## When NOT to Use

- Plant is well-modelled and uncertainty is small (LQR/LQG is simpler and less conservative)
- Plant is SISO with simple dynamics (PID with adequate margins is sufficient)
- Computational resources for design are limited (H-infinity synthesis requires specialised tools)

## Automotive Applications

- **Active suspension** — uncertain road profiles, varying vehicle load
- **EPS (Electric Power Steering)** — plant changes with vehicle speed
- **Ride comfort vs handling** — multi-objective robust trade-off
- Any system where gain scheduling would require too many operating points

## Relationship to Other Methods

| Method | Optimises for | Robustness guarantee |
|---|---|---|
| LQR | Average cost (quadratic) | Excellent (infinite GM, >=60deg PM) — but needs full state |
| LQG | Average cost + estimation | None (can be arbitrarily bad) |
| H-infinity | Worst-case amplification | Explicit — defined by gamma |

## Design Sketch (Conceptual)

1. Define the generalised plant P(s) with performance outputs z and measured outputs y
2. Choose frequency-dependent weighting functions (W1 for performance, W2 for robustness, W3 for control effort)
3. Solve the H-infinity optimisation: find K(s) such that ||F_l(P, K)||_inf < gamma
4. Iterate on weightings until the achieved gamma and closed-loop response are satisfactory

The weighting functions encode the engineer's intent (similar to Q/R in LQR, but frequency-dependent). Selecting good weightings requires understanding of the plant's frequency characteristics.

## Status in Caliburn

This file provides conceptual grounding for the controller selection guide. Full H-infinity design procedure and reference implementation are deferred until a Caliburn project requires robust output-feedback design.
