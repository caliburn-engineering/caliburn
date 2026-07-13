---
title: Weight Transfer — Load Redistribution Under Acceleration
sources:
  - { book: "Milliken & Milliken — Race Car Vehicle Dynamics", chapter: "5" }
  - { book: "Rajamani — Vehicle Dynamics and Control", chapter: "4" }
requires:
  - tyre-dynamics.md
related:
  - bicycle-model.md
  - understeer-oversteer.md
  - vehicle-control-systems.md
---

# Weight Transfer — Load Redistribution Under Acceleration

When a vehicle accelerates, brakes, or corners, the normal load on each tyre changes. Because tyre force generation is load-dependent (and nonlinearly so), weight transfer fundamentally affects vehicle handling and limits.

## Fundamental Equations

### Longitudinal Weight Transfer (Braking / Acceleration)

```
dFz_long = m * a_x * h / L
```

where:
- m = vehicle mass [kg]
- a_x = longitudinal acceleration [m/s^2] (positive = forward)
- h = CG height [m]
- L = wheelbase [m]

During braking (a_x < 0): load transfers forward — front tyres gain load, rear tyres lose load.
During acceleration (a_x > 0): load transfers rearward — rear tyres gain load.

### Lateral Weight Transfer (Cornering)

```
dFz_lat = m * a_y * h / t
```

where:
- a_y = lateral acceleration [m/s^2]
- t = track width [m]

During cornering: load transfers to the outside wheels — outside tyres gain, inside tyres lose.

### Individual Wheel Loads (Combined)

For a 4-wheeled vehicle cornering and braking simultaneously:

```
Fz_FL = (m*g*Lr)/(2*L) + dFz_long/2 + dFz_lat_front/2
Fz_FR = (m*g*Lr)/(2*L) + dFz_long/2 - dFz_lat_front/2
Fz_RL = (m*g*Lf)/(2*L) - dFz_long/2 + dFz_lat_rear/2
Fz_RR = (m*g*Lf)/(2*L) - dFz_long/2 - dFz_lat_rear/2
```

(Signs assume right-hand turn, FL = front-left, etc.)

## The Concavity Effect

This is the key insight that makes weight transfer always detrimental to total grip:

**Tyre force vs. normal load is a concave function (diminishing returns).**

```
F(Fz) = mu(Fz) * Fz,   where mu(Fz) decreases with Fz
```

Graphically:

```
Force
  ^
  |           .-------- (concave — slope decreases)
  |         /
  |       /
  |     /
  |   /
  | /
  +-------------------> Normal Load (Fz)
```

**Consequence:** When you transfer load dFz from the inside tyre to the outside:
- Outside gains: F(Fz + dFz) - F(Fz) = small gain (shallow slope)
- Inside loses: F(Fz) - F(Fz - dFz) = large loss (steep slope)
- Net: total force DECREASES

This is why:
- Lower CG height is always better for total grip
- Wider track width reduces lateral weight transfer
- Anti-roll bars redistribute weight transfer front/rear but cannot eliminate it
- Race cars are as low and wide as regulations allow

## Roll Stiffness Distribution

Total lateral weight transfer is fixed by physics (m * a_y * h / t). But the front/rear DISTRIBUTION of this transfer is controlled by relative roll stiffness:

```
dFz_lat_front = dFz_lat_total * (K_f / (K_f + K_r))
dFz_lat_rear  = dFz_lat_total * (K_r / (K_f + K_r))
```

where K_f, K_r = front and rear roll stiffness (spring + anti-roll bar).

### Tuning Implications

| Change | Effect on handling |
|---|---|
| Stiffer front ARB | More front weight transfer, less rear | Adds understeer |
| Stiffer rear ARB | More rear weight transfer, less front | Adds oversteer |
| Lower CG | Less total transfer | More total grip |
| Wider track | Less lateral transfer | More cornering force |

This is the primary setup tool for race engineers: adjusting the balance between understeer and oversteer without changing total grip.

## Dynamic Weight Transfer

The equations above describe steady-state weight transfer. In reality, weight transfer has dynamics:

1. **Geometric (instant):** Due to lateral force at tyre contact patches acting through suspension geometry. Appears within one timestep.
2. **Elastic (roll-rate limited):** Due to roll angle building up through springs and dampers. Time constant ~ 0.1-0.5 s depending on roll damping.

For vehicle control purposes, the steady-state model is usually sufficient (ESP operates on a timescale much slower than roll dynamics). For ride/handling simulation, the roll dynamics matter.

## Implementation Notes

### Weight Transfer Calculation

```cpp
struct WeightTransfer {
    double m, h, L, t, Lf, Lr;
    double Kf_ratio;  // front roll stiffness / total roll stiffness

    // Returns [FL, FR, RL, RR] normal loads
    std::array<double, 4> compute(double ax, double ay) const {
        double g = 9.81;
        double Fz_static_front = m * g * Lr / L / 2.0;
        double Fz_static_rear  = m * g * Lf / L / 2.0;

        double dFz_long = m * ax * h / L;
        double dFz_lat  = m * ay * h / t;

        double dFz_lat_f = dFz_lat * Kf_ratio;
        double dFz_lat_r = dFz_lat * (1.0 - Kf_ratio);

        return {
            Fz_static_front + dFz_long / 2.0 + dFz_lat_f / 2.0,  // FL
            Fz_static_front + dFz_long / 2.0 - dFz_lat_f / 2.0,  // FR
            Fz_static_rear  - dFz_long / 2.0 + dFz_lat_r / 2.0,  // RL
            Fz_static_rear  - dFz_long / 2.0 - dFz_lat_r / 2.0,  // RR
        };
    }
};
```

### Numerical Considerations

- Clamp individual wheel loads to >= 0 (wheel lift occurs when the inside rear reaches zero load during aggressive cornering)
- At wheel lift, the tyre generates zero force — this is a discontinuity that can cause solver issues
- For simulation stability, use a small minimum load (e.g., 10 N) rather than true zero
