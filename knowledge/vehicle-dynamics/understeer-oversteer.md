---
title: Understeer & Oversteer — Vehicle Stability Characteristics
sources:
  - { book: "Rajamani — Vehicle Dynamics and Control", chapter: "3" }
  - { book: "Milliken & Milliken — Race Car Vehicle Dynamics", chapter: "8" }
requires:
  - bicycle-model.md
  - tyre-dynamics.md
  - weight-transfer.md
related:
  - vehicle-control-systems.md
  - ../control-theory/controllers/pid.md
---

# Understeer & Oversteer — Vehicle Stability Characteristics

The understeer/oversteer characteristic determines how a vehicle responds when cornering limits are approached. It is the single most important handling quality for safety and is directly controlled by ESP systems.

## Definitions

### From the Bicycle Model

The understeer gradient K_us determines the steady-state yaw rate response:

```
r_ss = V * delta / (L + K_us * V^2)
```

| K_us | Characteristic | Physical Meaning |
|---|---|---|
| K_us > 0 | Understeer | Front tyres saturate first — car ploughs wide |
| K_us = 0 | Neutral | Ackermann-like response — yaw rate proportional to V |
| K_us < 0 | Oversteer | Rear tyres saturate first — tail slides out |

### Intuitive Description

**Understeer:** Driver turns the wheel more, car turns less. The car's path radius is LARGER than the driver commands. Feels "pushy" or "tight." Safe — driver naturally backs off when they feel resistance.

**Oversteer:** Car turns MORE than the driver commands. The path radius is SMALLER than intended. The rear steps out. Dangerous at the limit because the instability is self-reinforcing — a small yaw perturbation grows exponentially above V_crit.

### Critical Speed (Oversteer Vehicles)

```
V_crit = sqrt(-L / K_us)       (only defined for K_us < 0)
```

Above this speed, the open-loop vehicle is dynamically unstable. Any disturbance (wind gust, road camber, asymmetric braking) triggers a spin without electronic intervention.

Production cars are always designed with K_us > 0 (understeer). Race cars may be set up neutral or slightly oversteering for turn-in response, relying on driver skill and ESP for recovery.

## Causes of Understeer/Oversteer

### From Tyre Properties

```
K_us = m/L * (Lr/Cf - Lf/Cr)
```

| Factor | Increases Understeer | Increases Oversteer |
|---|---|---|
| Front cornering stiffness Cf | Decrease Cf | Increase Cf |
| Rear cornering stiffness Cr | Increase Cr | Decrease Cr |
| CG position | Forward (large Lf/Lr) | Rearward (small Lf/Lr) |

### From Weight Transfer

Lateral weight transfer reduces an axle's total cornering force (concavity effect). The axle with MORE weight transfer saturates first:

- More front weight transfer (stiff front ARB) → front saturates first → understeer
- More rear weight transfer (stiff rear ARB) → rear saturates first → oversteer

### From Longitudinal Forces

- Braking: shifts load forward, increases front grip, decreases rear grip → promotes oversteer
- Acceleration: shifts load rearward, increases rear grip, decreases front grip → promotes understeer (FWD) or oversteer (RWD at high power)

This is why lift-off oversteer occurs: driver lifts throttle mid-corner, load shifts forward, rear grip drops suddenly, car snaps into oversteer.

## Transient vs. Steady-State

The understeer gradient K_us describes steady-state behavior. Transient behavior can differ:

- **Yaw rate overshoot:** Even an understeer vehicle can exhibit transient oversteer (yaw rate overshoots its steady-state value) if the yaw damping is low
- **Sideslip build-up:** Large sideslip angles during transients indicate the rear is sliding even if the steady-state is understeer

ESP monitors BOTH yaw rate AND sideslip (estimated) to catch transient instabilities that the steady-state model would miss.

## ESP Correction Strategy

### Detection

```
Error = r_measured - r_desired

r_desired = V * delta / (L + K_us_ref * V^2)    (reference model)
            clamped to: |r_des| <= a_y_max / V   (friction limit)
```

### Correction Logic

| Condition | Diagnosis | Action |
|---|---|---|
| r_measured >> r_desired | Oversteer (spin) | Brake outer front wheel |
| r_measured << r_desired | Understeer (plough) | Brake inner rear wheel |
| |beta| > threshold | Large sideslip | Reduce speed (all-wheel braking) |

### Why These Specific Wheels?

**Oversteer correction — brake outer front:**
- Creates a yaw moment OPPOSING the spin
- Front wheel has more load (weight transfer during cornering to outside)
- Maximum braking force available where it's needed
- Also decelerates the vehicle, reducing lateral demand

**Understeer correction — brake inner rear:**
- Creates a yaw moment INTO the corner (tightens path)
- Inner wheel has less load — braking it doesn't sacrifice much lateral force
- The moment arm is large (track width / 2)

## Handling Diagram

The handling diagram plots front and rear slip angles against lateral acceleration:

```
Slip Angle
    ^
    |      Front slip (alpha_f)
    |     /
    |    /    Rear slip (alpha_r)
    |   /   /
    |  /   /
    | /  /
    |/ /
    +---------> Lateral Acceleration (a_y)
    |
    understeer if alpha_f grows faster than alpha_r
```

The DIFFERENCE (alpha_f - alpha_r) at a given a_y is the understeer angle. When this difference is positive and growing, the vehicle understeers. When it reverses sign, the vehicle transitions to oversteer (limit behavior — the "break point").

## Implementation Notes

### Stability Detection

```cpp
struct StabilityMonitor {
    double K_us_ref;  // reference understeer gradient
    double L;         // wheelbase

    double desired_yaw_rate(double V, double delta) const {
        double r_des = V * delta / (L + K_us_ref * V * V);
        double r_max = 9.81 * 0.9 / V;  // friction-limited (mu=0.9)
        return std::clamp(r_des, -r_max, r_max);
    }

    enum class State { Stable, Oversteer, Understeer };

    State diagnose(double r_measured, double r_desired, double threshold) const {
        double error = r_measured - r_desired;
        if (std::abs(error) < threshold) return State::Stable;
        // Same sign as desired but larger magnitude = oversteer
        if (error * r_desired > 0) return State::Oversteer;
        // Opposite effect = understeer
        return State::Understeer;
    }
};
```

### Test Scenarios

1. **Constant radius, increasing speed:** Measure steering angle vs. speed — slope is K_us
2. **Step steer at various speeds:** Verify yaw rate overshoot increases with speed
3. **Power-off mid-corner:** Verify transient oversteer occurs with rear-biased weight distribution
4. **ESP intervention:** Verify correct wheel is braked and yaw error converges
