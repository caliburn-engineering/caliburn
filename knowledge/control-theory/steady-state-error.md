---
title: Steady-State Error & System Types
sources:
  - { book: "Ogata — Modern Control Engineering", chapter: "5", pages: "230-260" }
  - { book: "Franklin, Powell, Emami-Naeini — Feedback Control", chapter: "4" }
requires:
  - state-space.md
  - frequency-response.md
related:
  - controllers/pid.md
  - stability.md
---

# Steady-State Error & System Types

Steady-state error is the difference between the desired output and the actual output as time approaches infinity. The Final Value Theorem and system type classification let you predict this error analytically — without simulation — based purely on the loop structure.

## Final Value Theorem

For a stable closed-loop system with input R(s) and error E(s):

```
e_ss = lim(t→∞) e(t) = lim(s→0) s * E(s)

For unity feedback:
E(s) = R(s) / (1 + G(s))

Therefore:
e_ss = lim(s→0) s * R(s) / (1 + G(s))
```

**Requirement:** All closed-loop poles must be in the LHP (system is stable). If not, FVT does not apply.

## System Type

The **type number** of a system = number of free integrators (poles at s=0) in the open-loop transfer function G(s).

```
Type 0:  G(s) = K * N(s) / D(s)         (no integrators)
Type 1:  G(s) = K * N(s) / (s * D(s))   (one integrator)
Type 2:  G(s) = K * N(s) / (s^2 * D(s)) (two integrators)
```

## Error Constants

```
Position constant:      Kp = lim(s→0) G(s)
Velocity constant:      Kv = lim(s→0) s * G(s)
Acceleration constant:  Ka = lim(s→0) s^2 * G(s)
```

## Steady-State Error Table

| Input | R(s) | Type 0 | Type 1 | Type 2 |
|---|---|---|---|---|
| Step (position) | 1/s | 1/(1+Kp) | 0 | 0 |
| Ramp (velocity) | 1/s² | ∞ | 1/Kv | 0 |
| Parabola (acceleration) | 1/s³ | ∞ | ∞ | 1/Ka |

**Key insights:**

- Each integrator in G(s) "eliminates" one class of input from causing steady-state error
- Type 0 + step = finite error (this is why P-only cruise control has offset on a hill)
- Adding an I term to PID adds one integrator → makes loop type-1 → zero e_ss for step inputs
- Type 2 needed for ramp tracking (e.g., trajectory following with constant velocity reference)

## Physical Interpretation

| System type | Physical meaning | Example |
|---|---|---|
| Type 0 | No integrator in plant or controller | P-only control of a spring-mass system |
| Type 1 | One integrator (from I-term or plant) | PI control of any plant, or P control of a motor (position = integral of velocity) |
| Type 2 | Two integrators | PID on a double integrator (position control of a free mass: F = ma, x = ∫∫a dt) |

## Design Implications

Before tuning gains, the system type tells you *structurally* whether zero error is achievable:

1. **Step tracking needed?** Ensure loop is at least type-1 (add I term or verify plant has integrator)
2. **Ramp tracking needed?** (constant-velocity reference) Need type-2 — either plant has two integrators or add them via controller
3. **Higher types** are possible but add phase lag (each integrator = -90° phase at all frequencies) which hurts stability margins

The trade-off: each integrator makes tracking better but stability harder. This is why you can't just "add more I terms."

## Worked Example: Cruise Control

```
Plant: G_car(s) = 1 / (m*s + b)    (1st order, type 0 — speed/force)
Controller: PID = Kp + Ki/s + Kd*s

Open-loop: L(s) = (Kp + Ki/s + Kd*s) * 1/(m*s + b)
                 = (Kd*s^2 + Kp*s + Ki) / (s * (m*s + b))

With I-term: type-1 loop → zero e_ss for step speed command.
Without I-term (P-only): type-0 → finite speed error under disturbance (hill).
```
