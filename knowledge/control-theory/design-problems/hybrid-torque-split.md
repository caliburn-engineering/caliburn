---
title: "Design Problem: Hybrid Torque Split Controller"
sources:
  - { note: "Mercedes interview prep — Lesson 5 applied design problems" }
  - { book: "Guzzella & Sciarretta — Vehicle Propulsion Systems", chapter: "7-9" }
  - { book: "Rajamani — Vehicle Dynamics and Control", chapter: "13" }
requires:
  - ../state-space.md
  - ../design-framework.md
related:
  - ../controllers/mpc.md
  - ../controllers/gain-scheduling.md
  - ../../vehicle-dynamics/hybrid-powertrain.md
  - ../../vehicle-dynamics/powertrain-torque-path.md
  - ../../vehicle-dynamics/vehicle-control-systems.md
---

# Design Problem: Hybrid Torque Split Controller

A worked example applying the [6-step design framework](../design-framework.md) to hybrid powertrain energy management. This problem is fundamentally different from the other two — it is an optimisation problem (how to split torque for minimum fuel / maximum performance), not a tracking problem (regulate a variable to a setpoint). The architecture choice reflects this distinction.

## Step 1 — Requirements

**Scenario:** P3 hybrid — V8 ICE plus rear electric motor. Driver requests 500 Nm at the wheels.

| Requirement | Specification |
|---|---|
| Primary constraint | Meet driver torque demand exactly: T_ice + T_motor = T_demand at wheels |
| Eco/Comfort mode | Minimise fuel consumption — run ICE at most efficient point |
| Sport+ mode | Maximise responsiveness — instant torque delivery, ICE for sustained power |
| EV mode | Motor only — below speed threshold and above SoC threshold |
| SoC management | Keep battery state-of-charge within target band (e.g., 20-80%) |
| Thermal limits | Respect battery and motor temperature limits — derate gracefully |
| Seamlessness | Driver must not feel the torque split or mode transitions |

## Step 2 — Plant Model

**Inputs:** ICE torque command T_ice_cmd, motor torque command T_motor_cmd.

**States:**
- Engine speed omega_ice (coupled through drivetrain)
- Motor speed omega_motor
- Battery state-of-charge SoC
- Battery temperature T_bat
- Motor temperature T_mot

**Key maps:**
- **BSFC map** (Brake-Specific Fuel Consumption): Fuel consumption as a function of ICE speed and torque. Defines the ICE's efficiency landscape — there is a narrow island of minimum BSFC.
- **Motor efficiency map**: Electrical-to-mechanical efficiency as a function of motor speed and torque. Generally flatter than ICE but with losses at high torque/speed.

**SoC dynamics:**

```
SoC_dot = -I_battery / Q_battery = -(T_motor * omega_motor) / (eta_motor * V_battery * Q_battery)
```

The optimal split depends on where each power source operates on its respective efficiency map at the current speed.

For detailed powertrain modelling, see [hybrid powertrain](../../vehicle-dynamics/hybrid-powertrain.md) and [powertrain torque path](../../vehicle-dynamics/powertrain-torque-path.md).

## Step 3 — Controllability and Observability

**Controllability:** Both ICE and motor torque are independently commandable. The system is over-actuated — infinite combinations of (T_ice, T_motor) satisfy T_ice + T_motor = T_demand. This over-actuation is what creates the optimisation opportunity.

**Observability:** Engine speed, motor speed, battery voltage, battery current, temperatures — all directly measurable via existing sensors. SoC is estimated from voltage + current integration (coulomb counting) with periodic recalibration. Observable with standard hybrid powertrain instrumentation.

**No fundamental C/O issues** — the challenge is optimisation, not controllability.

## Step 4 — Architecture Choice

This is an optimisation problem with the constraint T_ice + T_motor = T_demand. The architecture must decide, at each instant, how to split the torque.

### Option A: Rule-Based

Simple lookup tables indexed by speed, torque demand, SoC, and mode:

```
if SoC > 30% and speed < 50 km/h and mode == Eco:
    T_motor = T_demand    (EV mode)
elif T_demand > T_ice_max:
    T_motor = T_demand - T_ice_max    (boost)
else:
    use BSFC map to find optimal ICE operating point
```

**Trade-off:** Easy to calibrate and verify. Deterministic. Not globally optimal — human-designed rules cannot capture all operating conditions.

### Option B: ECMS (Equivalent Consumption Minimisation Strategy)

At each instant, assign an equivalence factor s to electrical energy:

```
J = m_fuel_dot + s * P_electrical
```

Choose the split that minimises J. The equivalence factor s encodes how much you value battery charge in fuel-equivalent terms.

**Trade-off:** Near-optimal if s is well-tuned. The challenge is that the optimal s depends on the future drive cycle (which is unknown). Adaptive ECMS adjusts s based on SoC deviation from target.

### Option C: MPC (Model Predictive Control)

Look ahead over a prediction horizon. If route/navigation data is available, plan the SoC trajectory:

```
minimise sum(fuel_consumption(k)) over k = 0..N
subject to:
    T_ice(k) + T_motor(k) = T_demand(k)
    SoC_min <= SoC(k) <= SoC_max
    T_ice_min <= T_ice(k) <= T_ice_max
    T_motor_min <= T_motor(k) <= T_motor_max
    T_bat(k) <= T_bat_max
```

**Trade-off:** Most optimal with preview information (e.g., charge on motorway, deplete in city). Computationally expensive. Requires a prediction of future driving — navigation-based or learned from driver history.

See [MPC](../controllers/mpc.md) for the general receding-horizon framework.

**Recommended:** ECMS for real-time torque split (computationally light, near-optimal) with MPC for strategic SoC planning if navigation data is available. Rule-based for mode transitions and safety logic.

## Step 5 — Hard Parts

### ICE Response Lag — Torque Fill

ICE torque response is slow (~200ms for torque build due to combustion dynamics, turbo lag). Motor response is fast (~10ms). During transients:

```
T_motor(t) = T_demand - T_ice_actual(t)    (motor fills the gap)
```

The motor provides instant torque while the ICE ramps up. This "torque fill" strategy is critical for driver-perceived responsiveness. Without it, there is a perceptible torque hole on tip-in.

### SoC Management

Using too much motor depletes the battery — the driver loses electric boost when they want it. Charging too aggressively wastes fuel and adds NVH (engine running at non-optimal points to generate charging power). The SoC target band must balance:
- Reserve for boost (Sport+ demands)
- Reserve for EV driving (city approach)
- Buffer for regen braking absorption
- Avoid deep discharge (battery health)

### Thermal Derating

On sustained high-power operation (e.g., track driving), battery and motor temperatures rise. The controller must gracefully reduce electric contribution before hitting thermal limits:

```
if T_motor > T_threshold:
    T_motor_max = T_motor_rated * (T_limit - T_motor) / (T_limit - T_threshold)
```

This derating curve must be smooth — an abrupt torque reduction at the thermal limit feels like a power cut to the driver.

### Mode Transitions

EV to hybrid: requires clutch engagement to connect ICE. The torque interruption during clutch engagement must be imperceptible:
1. Motor reduces torque slightly to create margin
2. ICE cranks and matches speed
3. Clutch engages with motor compensating for any torque dip
4. ICE takes over its share of the torque

This coordination happens in ~300-500ms and must be seamless. NVH (noise, vibration, harshness) during ICE start is a critical quality metric — the driver should not hear or feel the engine starting.

### Mode-Dependent Optimisation Objective

The optimisation objective itself changes with driving mode:

| Mode | Objective function |
|---|---|
| Eco | Minimise fuel consumption (BSFC-optimal ICE operating point) |
| Comfort | Minimise fuel + penalise NVH (avoid ICE speed changes) |
| Sport+ | Minimise response time (motor-first, ICE for sustained) |
| EV | Motor only, minimise battery degradation |

The ECMS equivalence factor s or the MPC cost function weights must switch with mode.

## Step 6 — Validation

| Stage | Method | Key metrics |
|---|---|---|
| **MIL** | Full powertrain model with BSFC/efficiency maps + driver model | Fuel consumption over WLTP, SoC trace, torque delivery accuracy |
| **HIL** | Powertrain ECUs against simulated vehicle + battery model | Mode transition timing, CAN bus latency, safety logic verification |
| **Vehicle testing** | Instrumented vehicle on road and track | Fuel consumption (WLTP), 0-100 km/h acceleration, NVH during transitions, thermal endurance |

**Critical test cases:**
- WLTP drive cycle (emissions certification — fuel consumption)
- Nurburgring lap (sustained high power — thermal management, SoC depletion)
- Urban stop-start driving (frequent EV/hybrid transitions)
- Cold start (battery capacity reduced, motor torque limited)
- Rapid mode switching by driver (Eco to Sport+ and back)
- Hill climb with low SoC (must manage ICE-only operation gracefully)
- NVH during EV-to-hybrid transition (clutch engagement quality)

## Key Takeaways

1. **This is an optimisation problem, not a tracking problem** — the architecture must choose among infinite valid solutions (any split that meets demand), not regulate a single variable.
2. **ECMS is the practical sweet spot** — near-optimal fuel savings without the computational cost of MPC. The equivalence factor is the single most important tuning parameter.
3. **Torque fill is non-negotiable** — the driver must never feel the ICE response lag. The motor's fast response is the key enabler.
4. **Thermal management is the hidden constraint** — on paper the hybrid has ample power, but thermal limits reduce available electric torque during sustained operation.
5. **Mode transitions are the hardest user-facing problem** — every transition is a potential NVH event. The mechanical coordination (clutch, speed matching) must be invisible to the driver.
6. **SoC management requires prediction** — without knowing the future drive profile, the controller must balance competing demands (reserve for boost vs. available for EV vs. battery health).
