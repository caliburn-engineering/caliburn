---
title: Gain Scheduling
sources:
  - { book: "Astrom & Wittenmark — Adaptive Control", chapter: "11" }
  - { book: "Rajamani — Vehicle Dynamics and Control", chapter: "5" }
  - { note: "engineering experience — automotive ECU implementations" }
requires:
  - pid.md
  - ../frequency-response.md
related:
  - lqr.md
  - sliding-mode.md
  - ../stability.md
  - ../design-problems/traction-control.md
  - ../design-problems/active-suspension.md
---

# Gain Scheduling

Gain scheduling is a **practical nonlinear control strategy**: design a family of linear controllers, each tuned for a specific operating point, then interpolate between them based on a scheduling variable.

```
Controller gains K = K(sigma)

where sigma = scheduling variable (e.g., vehicle speed, engine RPM, temperature)
```

It's not a single "gain-scheduled controller" — it's a **collection of linear controllers** with a switching/interpolation rule.

## Why It Exists

Many physical systems are nonlinear but can be approximated as linear **near** a given operating point. If the operating point moves slowly relative to the controller dynamics, you can:

1. Linearise the plant at multiple operating points
2. Design a controller for each linearisation
3. Interpolate controller parameters as the operating point moves

This is simpler than designing a single nonlinear controller, and leverages existing linear design tools (PID tuning, LQR, frequency-domain shaping).

## When to Use

| Use gain scheduling when | Don't use when |
|---|---|
| Plant dynamics change with operating point | Operating point changes faster than closed-loop bandwidth |
| Linear design tools work at each point | Strong coupling between scheduling variable and controlled states |
| Slow variation of operating point | Full nonlinear controller is tractable (e.g., SMC, feedback linearisation) |
| Proven in the domain (automotive, aerospace) | Safety-critical with no stability guarantee needed |

## Implementation Approaches

```
1. Lookup table: K(sigma) stored as discrete values, linear interpolation between
2. Polynomial fit: K(sigma) = a0 + a1*sigma + a2*sigma^2 + ...
3. Fuzzy scheduling: blend multiple controllers with membership functions
```

## Stability Concerns

**No global stability guarantee.** Each operating point is stable individually, but:
- Transitions between points can cause transients
- If scheduling variable changes too fast, stability can be lost
- Formal tools exist (LPV — Linear Parameter-Varying) but rarely used in production automotive

**Practical safeguard:** Ensure phase margin > 30 degrees at ALL scheduled operating points, including transitions.

## Automotive Examples

| System | Scheduling variable | Why gains must change |
|---|---|---|
| EPS (power steering) | Vehicle speed | Plant gain (lateral response) increases with speed. High assist at low speed, low at high. |
| Turbo boost control | Engine RPM | Turbo response time changes with RPM. Low-RPM needs aggressive gains (slow turbo), high-RPM needs detuning. |
| Traction control | Estimated tyre-road friction (mu) | Tyre curve shape changes with surface. Dry to wet to ice all need different slip targets and gains. |
| Suspension damping | Vehicle speed + driving mode | Different trade-offs at highway cruise vs city vs track. |
| Cruise control | Vehicle speed + road grade | Drag force is proportional to v^2 — plant gain changes with speed. |

## Relationship to LPV

Linear Parameter-Varying (LPV) is the formal framework:
- Treat the scheduling variable as a parameter in the state-space matrices: A(sigma), B(sigma)
- Design a single controller that guarantees stability for all sigma in a polytope
- More rigorous than classical gain scheduling but computationally heavier
- Used in aerospace; rare in automotive (classical gain scheduling suffices with margin checks)
