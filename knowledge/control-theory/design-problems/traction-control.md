---
title: "Design Problem: Traction Control System"
sources:
  - { note: "Mercedes interview prep — Lesson 5 applied design problems" }
  - { book: "Rajamani — Vehicle Dynamics and Control", chapter: "6" }
  - { book: "Pacejka — Tire and Vehicle Dynamics", chapter: "4" }
requires:
  - ../state-space.md
  - ../stability.md
  - ../design-framework.md
related:
  - ../controllers/pid.md
  - ../controllers/sliding-mode.md
  - ../controllers/gain-scheduling.md
  - ../../vehicle-dynamics/tyre-dynamics.md
  - ../../vehicle-dynamics/vehicle-control-systems.md
---

# Design Problem: Traction Control System

A worked example applying the [6-step design framework](../design-framework.md) to a rear-wheel-drive high-performance traction control system. This problem is representative of automotive control interviews — it tests understanding of nonlinear dynamics, surface estimation, and the limits of linear control.

## Step 1 — Requirements

**Scenario:** Rear-wheel-drive AMG, 600 hp, launching from standing start on a wet surface.

| Requirement | Specification |
|---|---|
| Controlled variable | Wheel slip ratio: lambda = (v_wheel - v_vehicle) / max(v_wheel, v_vehicle) |
| Target slip | Peak of tyre force-slip curve: ~10-15% on wet, ~8-12% on dry |
| Performance | Maximise longitudinal acceleration without wheelspin |
| Actuator bandwidth | Engine torque reduction ~50ms, brake intervention ~100ms |
| Comfort | No abrupt torque cuts — jerk limits apply |
| Surface adaptability | Must work across dry, wet, snow, ice, and split-mu surfaces |

## Step 2 — Plant Model

**Rotational dynamics of the driven wheel:**

```
J * omega_dot = T_engine * G - T_brake - F_tyre * R
```

where:
- J = wheel+drivetrain inertia
- omega = wheel angular velocity
- T_engine = engine torque (through gear ratio G)
- T_brake = brake torque
- F_tyre = tyre longitudinal force (nonlinear function of slip)
- R = tyre radius

**Key nonlinearity:** The tyre force F_tyre depends on slip through the [Pacejka Magic Formula](../../vehicle-dynamics/tyre-dynamics.md). The force-slip curve has a peak — beyond the peak, more slip produces less force (unstable region). This peak location shifts with surface condition.

**States:** Wheel speed omega, (optionally) tyre force estimate.

**Input:** Engine torque command (primary), brake torque command (secondary).

**Output:** Wheel speed sensors (all four wheels). Front undriven wheels provide vehicle speed reference.

## Step 3 — Controllability and Observability

**Controllability:** Wheel torque is controllable via engine and brakes. Engine torque reduction is faster for initial intervention; brake torque provides wheel-specific control. Controllable.

**Observability:** Slip ratio is computed, not directly measured. Vehicle speed is estimated from front (undriven) wheel speeds or GPS/IMU fusion. Key limitation: at very low speed (standing start), front wheels may not rotate — slip ratio computation becomes singular (division by near-zero). Need a low-speed strategy.

**Low-speed workaround:** Switch from slip-based control to acceleration-based control below a threshold speed (~5 km/h). Limit wheel acceleration directly rather than regulating slip ratio.

## Step 4 — Architecture Choice

**Why not pure PID:** The tyre curve is highly nonlinear. The plant gain (dF_tyre/d_lambda) changes sign at the peak — a fixed-gain PID that works below the peak will be unstable above it. Pure PID is insufficient.

**Candidate architectures:**

| Architecture | Rationale | Trade-off |
|---|---|---|
| **Gain-scheduled PID** | Different gains for below/above peak, indexed by estimated operating point | Practical, production-proven, but requires accurate operating point estimation |
| **Sliding mode control** | Robust to the nonlinearity — forces the system onto the target slip surface | Handles uncertainty well, but chattering must be managed |
| **Rule-based + PID hybrid** | Detect excess slip via thresholds, reduce torque in steps, PID for fine regulation | Simple to calibrate, widely used in production, but less optimal |

**Recommended:** Gain-scheduled PID for production (calibration-friendly, deterministic) with sliding mode insights for robustness analysis. See [gain scheduling](../controllers/gain-scheduling.md) and [sliding mode control](../controllers/sliding-mode.md).

## Step 5 — Hard Parts

### Surface Estimation

The tyre curve parameters (peak slip, peak force) vary dramatically with surface condition. The controller must either:
- **Estimate the surface:** Use the slope of the force-slip curve (dF/d_lambda) at current operating point to infer surface type. Steep slope = high-grip surface. Shallow slope = low grip.
- **Adapt online:** Start conservative and increase torque until slip is detected, then regulate.

### Low-Speed Singularity

At standing start, v_vehicle is near zero. Slip ratio lambda = (v_wheel - v_vehicle) / v_wheel approaches 1 regardless of actual grip. Solution: switch to acceleration-based control below a threshold, then transition to slip-based as speed builds.

### Split-Mu

Left and right wheels may be on different surfaces (e.g., left on asphalt, right on ice). Each wheel needs independent torque control. This requires individual wheel brake intervention — engine torque reduction alone affects both driven wheels equally.

### Engine Torque Response Delay

Combustion cycle introduces ~50-200ms transport delay between torque command and delivered torque. This limits controller bandwidth and eats into phase margin. The controller must account for this delay — either by reducing gains (conservative) or by using a Smith predictor structure.

### Drivetrain Oscillation

Aggressive torque modulation can excite drivetrain torsional resonance (drivetrain shuffle). Low-pass filtering the torque command or adding a notch filter at the shuffle frequency prevents this.

## Step 6 — Validation

| Stage | Method | Key metrics |
|---|---|---|
| **MIL** | Validated tyre model ([Pacejka](../../vehicle-dynamics/tyre-dynamics.md)) + drivetrain model | Slip ratio traces, time-to-peak-acceleration, stability on all surfaces |
| **HIL** | Engine ECU + simulated vehicle dynamics | Actuator response verification, communication latency, ECU timing |
| **Vehicle testing** | Dry, wet, snow, ice, split-mu surfaces | Launch time 0-100 km/h, slip ratio consistency, driver comfort (jerk), tyre wear |

**Critical test cases:**
- Standing start on wet surface (primary design case)
- Standing start on ice (extreme low grip)
- Split-mu launch (asymmetric grip)
- Mid-corner acceleration (combined slip — lateral + longitudinal)
- Rapid surface transition (dry to wet patch)

## Key Takeaways

1. **Nonlinear plant demands nonlinear or scheduled control** — fixed PID fails because the tyre curve gain changes sign at the peak.
2. **Surface estimation is the hardest subproblem** — the controller's target depends on an unknown parameter (surface grip).
3. **Low-speed operation requires a separate strategy** — slip ratio is mathematically undefined at zero speed.
4. **Split-mu needs per-wheel actuation** — engine torque reduction alone cannot handle asymmetric grip.
5. **Production traction control is typically rule-based + PID hybrid** — simpler to calibrate than optimal controllers, with gain scheduling for surface adaptation.
