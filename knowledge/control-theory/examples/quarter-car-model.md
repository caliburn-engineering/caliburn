---
title: "Worked Example: Quarter-Car Model (2 DOF)"
sources:
  - { book: "Rajamani — Vehicle Dynamics and Control", chapter: "6" }
  - { note: "engineering experience" }
requires:
  - ../state-space.md
  - ../first-principles-modelling.md
related:
  - double-mass-spring-damper.md
  - inverted-pendulum-cart.md
  - ../../vehicle-dynamics/vehicle-control-systems.md
  - ../controllers/lqr.md
  - ../design-problems/active-suspension.md
reference: ../../../reference/models/quarter_car.h
---

# Worked Example: Quarter-Car Model (2 DOF)

The quarter-car model is the fundamental model for vehicle ride dynamics and active suspension design. It captures the essential physics of sprung mass (body) and unsprung mass (wheel/tyre) connected by suspension spring + damper, with the tyre modelled as a spring connected to the road surface. This is a direct application of the double mass-spring-damper pattern with automotive framing.

## System Diagram

```
         ┌──────────┐
         │  Body    │  m_b (sprung mass)
         │  (m_b)   │
         └────┬─────┘
              │
        k_s /\/\  c_s ││     <- suspension spring + damper
              │
         ┌────┴─────┐
         │  Wheel   │  m_w (unsprung mass)
         │  (m_w)   │
         └────┬─────┘
              │
        k_t /\/\              <- tyre stiffness (no damping, or small)
              │
     ═════════╧═══════════     road surface: z_r(t)

     Coordinates: z_b (body), z_w (wheel), z_r (road input)
     Positive direction: upward
```

**DOF = 2.** Coordinates: z_b (body/sprung mass vertical displacement), z_w (wheel/unsprung mass vertical displacement). Both measured from static equilibrium. Road profile z_r(t) is a known disturbance input.

Optional: active force F_a between body and wheel (actuator for active suspension).

## Step 1: Newton on Body (m_b)

Forces on the sprung mass:
- Suspension spring: -k_s * (z_b - z_w) (opposes relative displacement)
- Suspension damper: -c_s * (z_dot_b - z_dot_w) (opposes relative velocity)
- Active force: +F_a (actuator, if present)

```
m_b * z_ddot_b = -k_s * (z_b - z_w) - c_s * (z_dot_b - z_dot_w) + F_a
```

## Step 2: Newton on Wheel (m_w)

Forces on the unsprung mass:
- Suspension spring reaction: +k_s * (z_b - z_w) (Newton's 3rd law)
- Suspension damper reaction: +c_s * (z_dot_b - z_dot_w)
- Tyre spring: -k_t * (z_w - z_r) (opposes deflection from road)
- Active force reaction: -F_a

```
m_w * z_ddot_w = k_s * (z_b - z_w) + c_s * (z_dot_b - z_dot_w) - k_t * (z_w - z_r) - F_a
```

## Step 3: Isolate Accelerations

```
z_ddot_b = [-k_s*(z_b - z_w) - c_s*(z_dot_b - z_dot_w) + F_a] / m_b

z_ddot_w = [k_s*(z_b - z_w) + c_s*(z_dot_b - z_dot_w) - k_t*(z_w - z_r) - F_a] / m_w
```

## Step 4: State-Space Assembly

States: **x = [z_b, z_dot_b, z_w, z_dot_w]^T**

Control input: u = F_a (active suspension force)
Disturbance input: w = z_r (road profile)

```
A = |  0         1          0               0        |
    | -k_s/m_b  -c_s/m_b   k_s/m_b         c_s/m_b  |
    |  0         0          0               1        |
    |  k_s/m_w   c_s/m_w  -(k_s+k_t)/m_w  -c_s/m_w  |

B_u = [0, 1/m_b, 0, -1/m_w]^T        (active force input)

B_w = [0, 0, 0, k_t/m_w]^T            (road disturbance input)
```

**Pattern check:** Identical structure to the double mass-spring-damper. Odd rows are trivial. Even rows are Newton divided by mass. Coupling terms have opposite signs (Newton's 3rd law). The only difference: the tyre spring k_t adds to the wheel diagonal but has no coupling to the body (it connects to the road, not to m_b).

## Separate B Matrices

This system has two types of inputs that should be tracked separately:

- **B_u** (control input): The active suspension force F_a acts on both masses with opposite signs (pushes body up, pushes wheel down, or vice versa). This is what LQR/H-infinity designs.
- **B_w** (disturbance input): The road profile z_r enters only through the tyre spring k_t, affecting only the wheel. This is the disturbance the suspension must reject.

The full state equation is:

```
x_dot = A*x + B_u*u + B_w*w
```

## C Matrix (Typical Outputs)

For ride comfort, the key outputs are body acceleration and suspension travel:

```
C_comfort = | 0  0  0  0 |    (body acceleration = row 2 of A*x + B*u)
            ...

C_travel = | 1  0  -1  0 |    (suspension travel = z_b - z_w)
           | 0  0   1  0 |    (tyre deflection proxy = z_w)
```

In practice, the output matrix depends on what sensors are available and what performance metrics matter.

## Numerical Example

Typical passenger car values:

```
m_b = 300 kg     (quarter of body mass)
m_w = 40 kg      (wheel + tyre + hub assembly)
k_s = 20000 N/m  (suspension spring rate)
c_s = 1500 N*s/m (damper coefficient)
k_t = 200000 N/m (tyre stiffness — much stiffer than suspension)
```

Substituting:

```
A = |  0       1        0           0       |
    | -66.67  -5.0     66.67       5.0      |
    |  0       0        0           1       |
    |  500     37.5    -5500       -37.5    |
```

## Natural Frequencies

Two natural frequencies correspond to the two vibration modes:

**Body bounce (ride mode):** ~1-1.5 Hz
```
f_body ~ (1/2*pi) * sqrt(k_s/m_b) = (1/2*pi) * sqrt(20000/300) ~ 1.3 Hz
```

**Wheel hop:** ~10-12 Hz
```
f_wheel ~ (1/2*pi) * sqrt((k_s+k_t)/m_w) = (1/2*pi) * sqrt(220000/40) ~ 11.8 Hz
```

The large frequency separation (~10x) means the two modes are weakly coupled. The body mode dominates ride comfort; the wheel hop mode dominates road holding.

## Cross-Reference: Active Suspension

The quarter-car model is the foundation for active suspension control. See [vehicle-control-systems.md](../../vehicle-dynamics/vehicle-control-systems.md) for how this model connects to:
- **Skyhook damping:** semi-active control law that sets damper force proportional to absolute body velocity
- **LQR optimal ride:** minimise body acceleration (comfort) subject to suspension travel limits
- **H-infinity robust control:** handle model uncertainty in tyre stiffness and road input spectrum

## Key Insights

1. **Two B matrices:** Always separate control input (F_a) from disturbance (z_r). They enter the system differently and are handled differently in controller design.
2. **Tyre stiffness dominates:** k_t >> k_s means the wheel mode is much faster than the body mode. This frequency separation simplifies control design — you can often design for the body mode and treat the wheel mode as a fast inner loop.
3. **Passive vs active:** Without F_a (passive suspension), the system is parameterised only by k_s, c_s, k_t. The damper c_s trades ride comfort against handling. Active suspension breaks this trade-off by adding a force degree of freedom.
4. **Same pattern as double mass-spring-damper:** The quarter-car is structurally identical to the wall-spring-mass-spring-mass system. The wall is replaced by the road, and the coupling is through the tyre spring instead of a wall spring.
