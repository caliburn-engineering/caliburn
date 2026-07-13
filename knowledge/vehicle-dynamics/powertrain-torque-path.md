---
title: Powertrain Torque Path — Engine to Road
sources:
  - { book: "Genta — Motor Vehicle Dynamics", chapter: "6" }
  - { note: "engineering experience" }
requires:
  - tyre-dynamics.md
related:
  - vehicle-control-systems.md
  - hybrid-powertrain.md
---

# Powertrain Torque Path — Engine to Road

The powertrain converts fuel/electrical energy into tractive force at the tyre contact patches. Understanding the complete torque path is essential for traction control, launch control, and hybrid energy management.

## The Torque Chain

```
Engine/Motor → Clutch/Torque Converter → Gearbox → Prop Shaft → Differential → Half Shafts → Wheels → Tyres → Road
```

Each component transforms and/or limits the torque flowing through it.

## Component Functions

### Engine / Electric Motor

**Source of torque.** Characterised by a torque-speed map:

- ICE: torque available is a function of RPM and throttle position
- Electric motor: peak torque from 0 RPM (constant torque region), power-limited above base speed (constant power region)

```
ICE:     T_engine = f(RPM, throttle)          — nonlinear map from dyno
E-motor: T_motor  = min(T_peak, P_max / omega) — torque-speed envelope
```

### Clutch / Torque Converter

**Couples engine to transmission.** Two operating modes:

| Mode | Description | Torque Transfer |
|---|---|---|
| Slipping | Engine and gearbox at different speeds | T = f(slip_speed) — controlled engagement |
| Locked | Same speed (1:1 coupling) | Passes full engine torque |

The clutch is a controllable disconnect — enables gear changes and launch. Torque converters (automatics) add torque multiplication at low speed ratios.

### Gearbox

**Torque multiplication and speed reduction:**

```
T_out = T_in * i_gear * eta_gear
omega_out = omega_in / i_gear
```

where i_gear = gear ratio (>1 in lower gears), eta_gear = mechanical efficiency (~0.95-0.98 per stage).

Total transmission ratio: i_total = i_gear * i_final_drive

### Differential

**Distributes torque between left and right wheels (or front and rear for AWD):**

| Type | Torque Split | Behaviour |
|---|---|---|
| Open diff | 50:50 always | Speed difference allowed — one wheel can spin freely |
| Limited slip (LSD) | Biases toward slower wheel | Clutch pack or viscous coupling limits slip |
| Electronic diff (eLSD) | Actively controlled | Brake or clutch actuator — software-defined split |
| Torque vectoring | Variable L/R | Active motor or clutch per side — yaw moment control |

**Open diff problem:** If one wheel loses grip (mu ~ 0), maximum tractive force = 2 * F_low_mu_wheel. The high-grip wheel is limited by the low-grip side. This is why LSD/torque vectoring exists.

### Half Shafts / Drive Shafts

Torsional compliance — acts as a spring between gearbox output and wheel. This creates drivetrain shuffle (oscillation) during sudden torque application:

```
J_engine * theta_e_ddot + c * (theta_e_dot - theta_w_dot) + k * (theta_e - theta_w) = T_engine
J_wheel * theta_w_ddot - c * (theta_e_dot - theta_w_dot) - k * (theta_e - theta_w) = -T_tyre
```

Natural frequency: f_shuffle = (1/2*pi) * sqrt(k * (1/J_engine + 1/J_wheel))
Typically 2-10 Hz — felt as a "shudder" or "jerk" during tip-in.

### Tyre-Road Interface

The final conversion from rotational to translational force. See [Tyre Dynamics](tyre-dynamics.md) for the full force-slip relationship.

```
F_traction = f(slip, Fz, mu_road)
```

The tyre is the ultimate bottleneck — no matter how much torque the engine produces, the road can only accept mu * Fz.

## Complete Torque Flow

```
F_wheel = T_engine * i_total * eta_total / R_wheel
```

where:
- i_total = i_gear * i_final
- eta_total = product of all efficiencies (~0.85-0.92 overall)
- R_wheel = tyre rolling radius

**Maximum acceleration** (traction limited, not power limited):

```
a_max = mu * g * (weight_on_drive_wheels / total_weight)
```

For RWD at launch with weight transfer helping: a_max can reach ~0.5-0.7g (road car), ~1.0-1.5g (race car with aero and slicks).

## Control Implications

### Traction Control

Must operate faster than the wheel inertia timescale. The wheel can spin up in ~50-100 ms under excess torque. Control bandwidth needs to be >10 Hz to catch slip before it reaches the unstable region.

Actuators:
- Engine torque reduction (spark retard: ~10 ms, fuel cut: ~50 ms, throttle: ~200 ms)
- Brake intervention: ~50 ms (hydraulic), ~20 ms (electric)
- Motor torque (EV): <5 ms — fastest actuator available

### Launch Control

Objective: maximum acceleration from standstill. Must track the peak of the force-slip curve:

1. Hold RPM at target (engine management)
2. Release clutch at controlled rate
3. Modulate torque to maintain lambda at peak (~10-15%)
4. Transition to full traction control once clutch is locked

### Drivetrain Anti-Shuffle

Active damping of drivetrain oscillations during tip-in:

```
T_correction = -K_d * (omega_engine/i_gear - omega_wheel)
```

Subtracts a torque proportional to the speed difference across the half-shaft — equivalent to adding damping to the torsional spring.

## Implementation Notes

### Simplified Powertrain for Simulation

```cpp
struct Powertrain {
    double i_gear;      // current gear ratio
    double i_final;     // final drive ratio
    double eta;         // overall efficiency
    double R_wheel;     // rolling radius

    double wheel_torque(double T_engine) const {
        return T_engine * i_gear * i_final * eta;
    }

    double traction_force(double T_engine) const {
        return wheel_torque(T_engine) / R_wheel;
    }

    double engine_rpm(double vehicle_speed) const {
        double omega_wheel = vehicle_speed / R_wheel;
        return omega_wheel * i_gear * i_final * 60.0 / (2.0 * M_PI);
    }
};
```
