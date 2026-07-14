---
title: Hybrid Powertrain — Energy Management and Torque Split
sources:
  - { book: "Guzzella & Sciarretta — Vehicle Propulsion Systems", chapter: "7-9" }
  - { book: "Rajamani — Vehicle Dynamics and Control", chapter: "13" }
requires:
  - powertrain-torque-path.md
  - tyre-dynamics.md
related:
  - vehicle-control-systems.md
  - ../control-theory/controllers/mpc.md
  - ../control-theory/design-problems/hybrid-torque-split.md
---

# Hybrid Powertrain — Energy Management and Torque Split

A hybrid powertrain combines an internal combustion engine (ICE) with one or more electric motors. The central engineering challenge is the energy management strategy: deciding how to split torque between ICE and motor at every instant to minimise fuel consumption while maintaining drivability and respecting battery limits.

## Key Decisions

| Decision | Inputs | Constraints |
|---|---|---|
| Torque split (ICE vs motor) | Driver torque demand, SoC, speed | Battery power limits, ICE operating map |
| Regen vs friction braking | Brake pedal force, speed, SoC | Battery charge rate limit, ABS priority |
| SoC management | Current SoC, predicted route, driving mode | SoC bounds (20%-80% for longevity) |
| Mode switching | Speed, torque demand, thermal state | Drivability (no jerks), NVH |

## Hybrid Architectures

### Series Hybrid

```
ICE → Generator → Battery ←→ Motor → Wheels
```

ICE never directly drives wheels. Runs at optimal efficiency point. Motor provides all traction.
- Advantage: ICE always at best efficiency
- Disadvantage: double energy conversion loss (mechanical → electrical → mechanical)

### Parallel Hybrid

```
ICE ──┐
      ├──→ Gearbox → Wheels
Motor ┘
```

Both ICE and motor can drive wheels simultaneously or independently.
- Advantage: direct mechanical path (fewer conversion losses)
- Disadvantage: ICE speed coupled to wheel speed (limited operating point freedom)

### Series-Parallel (Power-Split)

```
ICE ──→ Planetary Gear Set ──→ Wheels
              ↕
         Motor/Generator
```

Continuously variable ratio between ICE and wheels via electrical path.
- Example: Toyota Hybrid Synergy Drive (THS)
- Advantage: ICE operates near optimal regardless of vehicle speed
- Disadvantage: Mechanical complexity

### P0-P4 Classification

| Position | Location | Typical Power | Function |
|---|---|---|---|
| P0 | Belt-driven (crankshaft) | 10-15 kW | Mild hybrid — start/stop, torque fill |
| P1 | Crankshaft-mounted | 15-25 kW | ISG — regen, launch assist |
| P2 | Between clutch and gearbox | 30-80 kW | Can decouple ICE for EV driving |
| P3 | Gearbox output | 50-120 kW | Post-transmission — high torque |
| P4 | Rear axle (separate) | 50-150 kW | AWD hybrid — independent rear drive |

## Energy Management Strategies

### Rule-Based (Simple)

```
if SoC < 20%:       charge mode (ICE runs generator)
elif SoC > 80%:     EV mode (motor only)
elif torque_demand > ICE_efficient_threshold:
                     boost mode (ICE + motor)
else:               ICE only at efficient operating point
```

Advantages: easy to implement, deterministic, fast execution.
Disadvantages: suboptimal — rules don't adapt to driving conditions.

### ECMS (Equivalent Consumption Minimisation Strategy)

At each instant, minimise the equivalent fuel consumption:

```
J = m_dot_fuel + s * P_battery
```

where s is an equivalence factor converting electrical power to equivalent fuel rate.

- s converts Watts of electrical power to grams/second of fuel
- Higher s → prefer ICE (save battery for later)
- Lower s → prefer motor (use battery now)
- Adaptive ECMS adjusts s online to maintain SoC around target

This is a real-time implementable approximation of the globally optimal solution.

### Dynamic Programming (DP)

Given a known driving cycle, find the globally optimal torque split at every timestep:

```
Minimise: total_fuel = sum(m_dot_fuel(t) * dt)
Subject to: SoC_final = SoC_initial (charge-sustaining)
            SoC_min <= SoC(t) <= SoC_max
            P_motor <= P_motor_max
            T_ICE within feasible map
```

DP gives the theoretical optimum but requires the entire drive cycle in advance — used for benchmarking, not real-time control. ECMS with a well-tuned s approaches DP optimality within 1-3%.

### MPC (Model Predictive Control)

Rolling-horizon optimisation:
- Predict future driving conditions (speed profile) over ~10-30 seconds
- Optimise torque split over the prediction horizon
- Apply first timestep, re-plan at next step

Advantages: handles constraints naturally, adapts to changing conditions.
Disadvantages: computational cost — requires efficient solver for real-time ECU implementation.

## Regenerative Braking

### Blending Strategy

During braking, both the electric motor (regen) and friction brakes can decelerate:

```
T_brake_total = T_regen + T_friction

Objective: maximise T_regen (energy recovery) while maintaining:
  1. Driver-requested deceleration (pedal feel)
  2. ABS functionality (friction brakes respond faster)
  3. Battery charge limits (temperature, SoC)
  4. Motor torque limits (speed-dependent)
```

**Priority stack:**
1. ABS overrides everything (safety)
2. Requested deceleration must be met (driver expectation)
3. Regen fills as much of the demand as possible (efficiency)
4. Friction brakes cover the remainder

### Regen Limitations

| Constraint | Effect |
|---|---|
| Low speed (< 5 km/h) | Motor inefficient — switch to full friction |
| High SoC (> 90%) | Cannot charge further — friction only |
| High battery temperature | Reduce charge rate — partial friction |
| ABS active | Friction needed for rapid modulation |
| Motor power limit | Regen limited at high speed (power = torque * omega) |

## F1 ERS Reference (Context)

Modern F1 Energy Recovery Systems as an extreme example of hybrid energy management:

| Component | Energy Limit | Power | Notes |
|---|---|---|---|
| MGU-K recovery | 2 MJ/lap | 120 kW | Braking energy → electrical |
| MGU-K deployment | 4 MJ/lap | 120 kW (~160 hp, ~33s/lap) | Electrical → rear wheels |
| MGU-H | Unlimited | Unlimited | Exhaust energy, eliminates turbo lag |
| Energy Store | 4 MJ usable | — | Battery buffer (~20 kg) |

The extra 2 MJ deployed vs recovered comes from MGU-H harvesting exhaust energy. The strategy layer decides WHEN to deploy the 120 kW boost (corner exits, overtaking zones) — a constrained optimisation problem solved lap-by-lap.

### Control Problem Framing

The ERS is an optimal energy management problem under real-time constraints:

**Harvest decisions:** when to recover energy from braking (MGU-K) and exhaust heat (MGU-H), balancing energy capture against:
- Tyre/brake interaction during MGU-K harvesting (regen torque affects brake balance)
- Battery thermal limits (charge rate bounded by cell temperature)

**Deployment decisions:** when to release the 120 kW boost, bounded by:
- 4 MJ per-lap deployment limit (budget ~33 seconds of boost per lap)
- Battery SoC constraints (cannot deploy below minimum charge)
- Lap-by-lap strategy optimisation (different circuits have different optimal deployment maps)

This maps directly to the same ECMS / MPC frameworks used in road-car hybrids (see above), but with much tighter constraints and higher stakes.

> **Note:** MGU-H removed from 2026 F1 regulations. The 2026 power unit increases MGU-K power to 350 kW and removes the MGU-H entirely, shifting the harvesting burden fully to braking energy recovery.

## Mode Switching and Drivability

### Mode Transitions

| From → To | Challenge | Solution |
|---|---|---|
| EV → ICE | Engine start vibration, torque discontinuity | Torque fill from motor during cranking |
| ICE → EV | Engine shutdown felt as torque drop | Motor pre-fills before ICE cuts |
| Regen → Friction | Transition felt in pedal | Smooth blending over 100-200 ms |
| Gear shift (ICE) | Torque hole during shift | Motor fills gap ("torque fill") |

**Drivability constraint:** Any mode transition must be imperceptible to the driver. This means:
- Torque continuity: total wheel torque must not have step changes > ~5 Nm
- Rate limiting: torque ramp rate < ~500 Nm/s
- Jerk limiting: da/dt < 10 m/s^3

## Implementation Notes

### Simplified ECMS

```cpp
struct ECMS {
    double s;              // equivalence factor [g/s per W]
    double SoC_target;     // target state of charge
    double k_adapt;        // adaptation rate

    struct Split {
        double T_ice;
        double T_motor;
    };

    Split optimal_split(double T_demand, double omega, double SoC,
                        const EngineMap& ice, const MotorMap& mot) const {
        Split best{T_demand, 0.0};
        double best_cost = 1e9;

        // Search over feasible motor torque range
        double T_mot_max = mot.max_torque(omega);
        double T_mot_min = mot.min_torque(omega);  // negative = regen

        for (double T_mot = T_mot_min; T_mot <= T_mot_max; T_mot += 1.0) {
            double T_ice = T_demand - T_mot;
            if (!ice.is_feasible(T_ice, omega)) continue;

            double fuel_rate = ice.fuel_rate(T_ice, omega);
            double P_elec = mot.electrical_power(T_mot, omega);
            double cost = fuel_rate + s * P_elec;

            if (cost < best_cost) {
                best_cost = cost;
                best = {T_ice, T_mot};
            }
        }
        return best;
    }

    void adapt_s(double SoC) {
        // Simple proportional adaptation
        s += k_adapt * (SoC - SoC_target);
    }
};
```

### SoC Management

```cpp
// Enforce SoC bounds with soft constraints
double soc_penalty(double SoC) {
    if (SoC < 0.2) return 1000.0 * (0.2 - SoC);  // heavily penalise low SoC
    if (SoC > 0.8) return 1000.0 * (SoC - 0.8);  // heavily penalise high SoC
    return 0.0;
}
```
