---
title: Automotive Control Scenarios — Interview-Style Questions
sources:
  - { lesson: "PID Recall Drill", ref: "0001-pid-recall-drill" }
  - { lesson: "Vehicle Dynamics & Powertrain", ref: "0004-vehicle-dynamics-powertrain" }
requires:
  - vehicle-control-systems.md
  - tyre-dynamics.md
  - hybrid-powertrain.md
related:
  - understeer-oversteer.md
  - ../control-theory/controllers/pid.md
  - ../control-theory/controllers/sliding-mode.md
  - ../control-theory/controllers/gain-scheduling.md
  - ../control-theory/controllers/lqr.md
  - ../control-theory/observers/kalman-filter.md
  - ../control-theory/design-problems/traction-control.md
  - ../control-theory/design-problems/active-suspension.md
  - ../control-theory/design-problems/hybrid-torque-split.md
---

# Automotive Control Scenarios — Interview-Style Questions

Scenario-based questions that test understanding of vehicle control systems. Each presents a realistic engineering situation, the correct response, and the reasoning behind it. Useful for interview preparation, self-assessment, and the `/explain-concept` skill.

---

## Scenario 1: Cruise Control Offset on a Hill

**Situation:** A cruise control system uses P-only control. The driver sets 100 km/h on flat road and it works fine. Approaching a long hill, the speed settles at 97 km/h and stays there.

**Question:** Why does the 3 km/h offset persist? How would you fix it?

**Answer:** This is steady-state error — the defining limitation of proportional-only control. The controller output is proportional to the error, so it needs a non-zero error to produce the non-zero output required to climb the hill. On flat ground the required output is small, so the error is small. On a hill, more torque is needed, requiring a larger error to sustain it.

**Fix:** Add an integral term. The integrator accumulates the error over time, building up the extra output needed for the hill until the error is driven to zero. This is the core justification for PI or PID control.

**Cross-references:** [PID](../control-theory/controllers/pid.md) -- steady-state error and integral action

---

## Scenario 2: Temperature Controller with Noisy Derivative

**Situation:** A temperature controller for a manufacturing process uses full PID. When the D-term gain is increased to improve setpoint tracking, the heater starts chattering rapidly on and off.

**Question:** What is causing the chatter? How would you address it?

**Answer:** The derivative term amplifies high-frequency noise in the temperature measurement. Temperature sensors (thermocouples, RTDs) produce noisy signals. The D-term computes dError/dt, which magnifies rapid fluctuations. This produces a rapidly oscillating control output that switches the heater on and off at high frequency.

**Fix options:**
1. Low-pass filter on the derivative term (most common — derivative filter with time constant Td/N, where N = 5-20)
2. Derivative on PV only (not on error) to avoid setpoint kick
3. Reduce Kd — accept slower response for less noise sensitivity
4. Better sensor filtering upstream

**Cross-references:** [PID](../control-theory/controllers/pid.md) -- derivative filtering, noise amplification

---

## Scenario 3: PID with 40% Overshoot

**Situation:** A PID-controlled motor position system reaches the target but overshoots by 40% before settling. Rise time is acceptable.

**Question:** Which gain would you adjust, and in which direction?

**Answer:** Increase Kd (derivative gain). The derivative term provides damping by opposing the rate of change of the error. When overshoot is excessive but rise time is acceptable, the system has insufficient damping. Increasing Kd adds prediction — the controller sees the error decreasing rapidly and begins reducing output before the setpoint is reached.

**Why not reduce Kp?** Reducing Kp would also reduce overshoot, but at the cost of slower response and potentially larger steady-state error. Kd targets overshoot specifically without sacrificing rise time.

**Cross-references:** [PID](../control-theory/controllers/pid.md) -- Kd tuning, [Second-Order Systems](../control-theory/second-order-systems.md)

---

## Scenario 4: "Crank Up All Gains"

**Situation:** A junior engineer suggests maximising Kp, Ki, and Kd simultaneously for the "best performance" of a motor speed controller.

**Question:** What happens? Why is this wrong?

**Answer:** The system goes unstable. High Kp drives the loop gain up, reducing phase margin. High Ki adds phase lag at low frequencies and can cause integrator windup. High Kd amplifies sensor noise and adds high-frequency gain. Together, they erode phase margin from both ends of the frequency spectrum.

The fundamental constraint is that every real system has transport delays and unmodelled dynamics. High gains push the crossover frequency into regions where the plant model is inaccurate, and the phase margin drops below zero — the system oscillates or diverges.

**The correct approach:** Tune gains with respect to the plant dynamics — use Bode plots to verify adequate gain and phase margin (typically >6 dB and >45 degrees).

**Cross-references:** [PID](../control-theory/controllers/pid.md), [Stability](../control-theory/stability.md), [Compensator Design](../control-theory/compensator-design.md)

---

## Scenario 5: PID on Tyre Slip Ratio

**Situation:** A traction control system uses a fixed-gain PID to regulate tyre slip ratio at 0.1 (peak friction). On dry tarmac it works well. On wet road, the system oscillates wildly between wheel spin and lockup.

**Question:** Why does the fixed PID fail? What would you use instead?

**Answer:** The tyre force-slip curve is highly nonlinear and changes shape with road surface. On dry tarmac, the curve has a broad peak around slip = 0.1 with moderate gradient. On wet road, the peak is sharper and occurs at a different slip value, and the post-peak slope (unstable region) is much steeper.

A fixed PID tuned for the dry-road plant gain will have too much gain for the wet-road plant (steeper slope = higher plant gain), causing oscillation. The system is operating on a fundamentally different plant.

**Better approaches:**
- Gain scheduling — adapt PID gains based on estimated surface condition
- Sliding mode control — inherently robust to plant variations, switches between acceleration and braking across a sliding surface
- Adaptive control — online estimation of the tyre curve parameters

**Cross-references:** [Tyre Dynamics](tyre-dynamics.md) -- force-slip curve, [Gain Scheduling](../control-theory/controllers/gain-scheduling.md), [Sliding Mode](../control-theory/controllers/sliding-mode.md), [Traction Control](../control-theory/design-problems/traction-control.md)

---

## Scenario 6: ESP Oversteer Correction

**Situation:** A rear-wheel-drive car enters a corner too fast and the rear begins to slide out (oversteer). The ESP system activates.

**Question:** Which wheel does the ESP brake, and why?

**Answer:** Brake the **outer front** wheel. This creates a yaw moment that opposes the oversteer rotation:

- The outer front wheel has the highest normal force (weight transfers to outside and front during cornering + deceleration)
- Braking it creates a longitudinal force pointing rearward on the outside of the turn
- This generates a stabilising yaw moment that pulls the nose outward, counteracting the tail sliding out

**Why not the inner rear?** The inner rear has the least normal force (weight has transferred away from it), so braking it produces minimal force. The outer front maximises the corrective moment arm.

**Cross-references:** [Understeer & Oversteer](understeer-oversteer.md), [Vehicle Control Systems](vehicle-control-systems.md) -- ESP, [Weight Transfer](weight-transfer.md)

---

## Scenario 7: Hybrid Lift-Off Regen

**Situation:** A hybrid vehicle driver lifts off the accelerator in a corner. The regenerative braking applies significant deceleration torque to the driven wheels, causing unexpected weight transfer and potential instability.

**Question:** What controller is needed to handle this safely?

**Answer:** A **brake blending controller** that coordinates regen torque with vehicle dynamics:

1. Limit regen torque during cornering (lateral acceleration > threshold) to avoid destabilising weight transfer
2. Ramp regen torque gradually on lift-off (rate limiting) rather than applying it as a step
3. Coordinate with ESP — if the stability system detects yaw rate deviation, reduce regen immediately
4. Blend between regen and friction brakes based on stability requirements, not just energy recovery efficiency

The key insight is that regen torque acts on the driven axle only (unlike friction brakes on all four wheels), so it creates an asymmetric deceleration that can upset the car's balance, especially mid-corner.

**Cross-references:** [Hybrid Powertrain](hybrid-powertrain.md) -- regen braking, [Vehicle Control Systems](vehicle-control-systems.md), [Weight Transfer](weight-transfer.md)

---

## Scenario 8: Active Suspension Observer Need

**Situation:** An active suspension system needs to know the tyre deflection and road profile to optimally control the damper force. Neither is directly measurable with standard sensors.

**Question:** Why can't you just use sensors? What do you use instead?

**Answer:** An **observer** (state estimator) is required because:

- Tyre deflection is the compression of the tyre itself (difference between wheel position and road surface) — there is no practical sensor for road surface height at the contact patch
- Road profile is unobservable directly — you cannot mount a sensor under the tyre
- Available sensors are: accelerometers (sprung/unsprung mass), suspension travel (LVDT), and wheel speed

An observer (Kalman filter or Luenberger) estimates the unobservable states from the measurable ones using a dynamic model of the quarter-car:

```
Measurable: sprung mass acceleration, suspension deflection
Estimated:  tyre deflection, road velocity, unsprung mass velocity
```

This is a textbook case of observability — the system is observable from the available measurements, but the states of interest are not directly measured.

**Cross-references:** [Kalman Filter](../control-theory/observers/kalman-filter.md), [Luenberger Observer](../control-theory/observers/luenberger.md), [Active Suspension](../control-theory/design-problems/active-suspension.md), [Vehicle Control Systems](vehicle-control-systems.md)

---

## Scenario 9: Twin-Motor Torque Vectoring

**Situation:** An electric vehicle has independent motors on the left and right rear wheels. The goal is to control both the total traction force AND the yaw moment independently.

**Question:** Why can't a standard SISO controller handle this? What approach is needed?

**Answer:** This is a **MIMO (Multi-Input Multi-Output)** control problem that requires decoupling:

**Inputs:** Left motor torque (T_L), Right motor torque (T_R)
**Outputs:** Total longitudinal force (F_x = F_L + F_R), Yaw moment (M_z = (F_R - F_L) * track/2)

A SISO controller on each motor independently would couple the force and yaw channels — increasing total force would inadvertently affect yaw, and vice versa. The controller must decouple these:

```
T_L = (F_x_demand / r) / 2 - M_z_demand / (track * r)
T_R = (F_x_demand / r) / 2 + M_z_demand / (track * r)
```

where r is the tyre radius. This is a static decoupling (coordinate transform). The actual control uses:
- An outer loop: yaw rate controller (LQR or PID on yaw rate error) generates M_z_demand
- A driver demand interpreter: accelerator pedal maps to F_x_demand
- An allocation layer: transforms (F_x_demand, M_z_demand) into (T_L, T_R)

**Cross-references:** [Vehicle Control Systems](vehicle-control-systems.md) -- torque vectoring, [LQR](../control-theory/controllers/lqr.md), [Bicycle Model](bicycle-model.md)

---

## Usage Notes

These scenarios are designed to test conceptual understanding, not rote recall. In an interview setting, the key is to:

1. **Identify the root cause** before proposing a fix
2. **Name the control principle** at play (steady-state error, noise amplification, plant variation, observability, MIMO coupling)
3. **Connect to the broader system** — how does this interact with other vehicle subsystems?
4. **Propose alternatives** — show you know multiple solutions and their tradeoffs
