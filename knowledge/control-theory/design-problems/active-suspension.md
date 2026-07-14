---
title: "Design Problem: Active Suspension Damping"
sources:
  - { note: "Mercedes interview prep — Lesson 5 applied design problems" }
  - { book: "Rajamani — Vehicle Dynamics and Control", chapter: "6" }
  - { note: "engineering experience — semi-active damper control" }
requires:
  - ../state-space.md
  - ../stability.md
  - ../design-framework.md
related:
  - ../controllers/lqr.md
  - ../controllers/gain-scheduling.md
  - ../examples/quarter-car-model.md
  - ../../vehicle-dynamics/vehicle-control-systems.md
  - ../../vehicle-dynamics/weight-transfer.md
---

# Design Problem: Active Suspension Damping

A worked example applying the [6-step design framework](../design-framework.md) to semi-active suspension control with driver-selectable modes. This problem tests understanding of the passivity constraint (semi-active actuators can only dissipate energy), multi-objective trade-offs (comfort vs handling), and mode-switching strategies.

## Step 1 — Requirements

**Scenario:** Adjustable dampers with Comfort, Sport, and Sport+ modes.

| Requirement | Specification |
|---|---|
| Controlled variables | Body heave acceleration (ride), body roll/pitch (handling) |
| Comfort mode | Minimise body acceleration — isolate cabin from road |
| Sport mode | Reduce body roll and pitch — accept harsher ride |
| Sport+ mode | Maximum body control — accept road harshness |
| Bandwidth | Body modes ~1-3 Hz, wheel hop ~10-15 Hz |
| Actuator type | Semi-active damper — variable damping coefficient within a range |
| Constraint | Damper can only dissipate energy, never inject it (passivity constraint) |
| Failure mode | Default to fixed medium damping on sensor failure |

## Step 2 — Plant Model

**Quarter-car model** (per corner):

```
m_s * z_s_ddot = -k_s * (z_s - z_u) - c(u) * (z_s_dot - z_u_dot)
m_u * z_u_ddot =  k_s * (z_s - z_u) + c(u) * (z_s_dot - z_u_dot) - k_t * (z_u - z_r)
```

**States (per corner):** Body position z_s, body velocity z_s_dot, wheel position z_u, wheel velocity z_u_dot.

**Full vehicle:** 7-DOF minimum — heave, pitch, roll + 4 wheel hops. 14 states total.

**Input:** Damping coefficient command c(u) — variable within [c_min, c_max]. This is not a force command; the damper force depends on both the commanded damping and the relative velocity.

**Output:** Body accelerometer, suspension travel sensor (LVDT), wheel speed sensors.

**Disturbance:** Road surface profile z_r(t) — unknown, treated as a stochastic disturbance.

For the quarter-car state-space derivation, see [quarter-car worked example](../examples/quarter-car-model.md).

## Step 3 — Controllability and Observability

**Controllability:** The damping coefficient is the only control input per corner. Crucially, the damper force is c(u) * v_rel — the force magnitude depends on relative velocity, not just the command. When v_rel is near zero (at stroke reversal), the damper has almost no authority regardless of the damping setting. This is a state-dependent controllability limitation.

**Observability:** Road profile z_r is not directly measured. Body acceleration and suspension travel can reconstruct most states via an observer. The road input is the main unobservable disturbance — the controller must treat it as unknown.

**Observer requirement:** A state observer (e.g., Kalman filter) is needed to estimate z_u_dot and z_r from the available sensors.

## Step 4 — Architecture Choice

**Skyhook damping** is the classic semi-active strategy, well-suited to this problem.

### Skyhook Concept

Damp the body as if it were connected to an inertial reference frame ("the sky") rather than to the wheel. The ideal skyhook force would be F = -c_sky * z_s_dot, but a passive damper can only produce force proportional to relative velocity.

**Implementation rule:**

```
if (z_s_dot * (z_s_dot - z_u_dot)) > 0:
    c = c_max    (damper opposes body motion — desirable)
else:
    c = c_min    (damper would pull body toward wheel — go soft to avoid this)
```

This switching law approximates active suspension performance using only a semi-active actuator.

### Groundhook Addition

For handling modes, add groundhook damping — damp the wheel motion relative to ground to maintain tyre contact:

```
if (z_u_dot * (z_u_dot - z_s_dot)) > 0:
    c_ground = c_max
else:
    c_ground = c_min
```

Blend skyhook and groundhook with mode-dependent weighting.

### Mode Mapping

| Mode | Skyhook gain | Groundhook gain | Additional feedforward |
|---|---|---|---|
| **Comfort** | Low | Minimal | None — prioritise isolation |
| **Sport** | Medium | Medium | Anti-roll from steering angle |
| **Sport+** | Maximum | Maximum | Anti-roll + anti-pitch from throttle/brake signals |

## Step 5 — Hard Parts

### Passivity Constraint

The semi-active damper can only dissipate energy — it cannot push the body upward or pull the wheel downward. At every timestep, the controller must verify that the commanded damping produces a force consistent with energy dissipation:

```
F_damper * v_rel >= 0    (power dissipation — force opposes relative motion)
```

If the desired force would inject energy, the controller must clamp to minimum damping. This is what makes semi-active control fundamentally different from full active control — the controller's authority is state-dependent.

### Damper Bandwidth

Real dampers cannot switch instantaneously between c_min and c_max. Typical response time: 10-20ms. The controller must account for this lag — high-frequency switching commands will not be faithfully followed. A minimum dwell time or rate limiter on the damping command prevents commanding faster than the damper can respond.

### Mode Transition Smoothness

Switching from Comfort to Sport+ must not produce a sudden jerk. Ramp the mode gains over 200-500ms:

```
alpha(t) = clamp((t - t_switch) / T_ramp, 0, 1)
gains = (1 - alpha) * gains_old + alpha * gains_new
```

### Sensor Failure Fallback

If the body accelerometer or suspension travel sensor fails, the controller cannot compute skyhook damping. The safe fallback is fixed medium damping — not optimal for any mode, but safe and predictable. The failure detection and transition to fallback mode must be designed as carefully as the nominal controller.

### Cross-Coupling (Full Vehicle)

The quarter-car model treats each corner independently, but in practice, pitch and roll couple the corners. Anti-roll bar forces distribute load across axles. A full vehicle controller must coordinate all four corners — especially for roll control during cornering and pitch control during braking.

## Step 6 — Validation

| Stage | Method | Key metrics |
|---|---|---|
| **MIL** | Quarter-car + full-vehicle simulation with ISO 8608 road profiles | Body acceleration RMS, suspension travel, tyre contact force |
| **HIL** | Real damper hardware against simulated vehicle | Damper response time verification, command tracking, power consumption |
| **Vehicle testing** | Instrumented vehicle on test tracks | ISO 2631 ride comfort (weighted body acceleration), handling metrics (roll angle in lane change, pitch in braking) |

**Critical test cases:**
- Random road at highway speed (Comfort mode — ride quality)
- Double lane change at 80 km/h (Sport/Sport+ — body roll)
- Emergency braking (pitch control, tyre contact maintenance)
- Single pothole (transient response, damper bandwidth test)
- Mode transition during cornering (smoothness verification)
- Sensor failure during operation (fallback behaviour)

**Subjective evaluation:** Professional test driver assessment is essential. ISO metrics quantify ride but do not capture all aspects of ride feel. A controller can meet all numerical specs and still feel "wrong" to a driver.

## Key Takeaways

1. **Semi-active is not the same as active** — the passivity constraint fundamentally limits what the controller can do, especially at stroke reversal when relative velocity is near zero.
2. **Skyhook damping is elegant because it's simple** — a switching rule that approximates active performance. No observer required for the basic version (only body velocity and relative velocity needed).
3. **Mode switching is a control design problem in itself** — abrupt gain changes cause transient disturbances that the driver will feel.
4. **Failure modes drive architecture decisions** — the fallback strategy (fixed damping) must be designed alongside the nominal controller.
5. **Subjective evaluation cannot be replaced by metrics** — ride feel has dimensions that ISO 2631 does not capture.
