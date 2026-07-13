---
title: Vehicle Control Systems — A Map of Automotive Control Engineering
sources:
  - { book: "Rajamani — Vehicle Dynamics and Control", chapter: "1, 6-12" }
  - { book: "Isermann — Automotive Control", chapter: "overview" }
requires:
  - tyre-dynamics.md
  - bicycle-model.md
related:
  - understeer-oversteer.md
  - hybrid-powertrain.md
  - ../control-theory/controllers/pid.md
  - ../control-theory/controllers/lqr.md
  - ../control-theory/controllers/sliding-mode.md
---

# Vehicle Control Systems — A Map of Automotive Control Engineering

Every modern vehicle contains 30-100 ECUs running control algorithms. This file maps the major vehicle control systems, showing where control engineering disciplines apply in practice.

## System Overview Table

| System | Controlled Variable | Actuator | Control Approach | Bandwidth |
|---|---|---|---|---|
| ABS | Wheel slip ratio | Brake pressure valves | Rule-based switching (threshold) | ~50 Hz |
| TCS | Drive slip ratio | Engine torque + brakes | Slip threshold + gain scheduling | ~20 Hz |
| ESP/ESC | Yaw rate, sideslip | Individual brake calipers | Model-based (bicycle model reference) | ~25 Hz |
| Active Suspension | Body heave, pitch, roll | Dampers or hydraulic actuators | Skyhook / LQR / H-infinity | ~10-30 Hz |
| EPS | Assist torque | Steering column motor | Gain-scheduled with vehicle speed | ~100 Hz |
| Torque Vectoring | Yaw moment via L/R torque | Twin motors or active differential | MIMO torque allocation | ~50 Hz |
| Hybrid Energy Mgmt | ICE/motor torque split, SoC | ICE + electric motor(s) | ECMS / DP / MPC | ~1 Hz (strategy) |
| Launch Control | Slip at maximum traction | Engine + clutch | Peak slip tracking | ~50 Hz |
| ACC / ADAS | Speed, headway distance | Throttle + brake | Cascaded PID / MPC | ~10 Hz |

## System Details

### ABS (Anti-lock Braking System)

**Problem:** Under heavy braking, wheels lock (lambda = -1). A locked wheel has LESS braking force than one at peak slip (~10-15%), and ZERO lateral force (no steering control).

**Solution:** Modulate brake pressure to keep slip near the peak of the force-slip curve.

**Algorithm (simplified):**
```
if wheel_deceleration > threshold_1:
    REDUCE pressure (wheel about to lock)
elif wheel_acceleration > threshold_2:
    HOLD pressure (recovering)
elif slip < target:
    INCREASE pressure (more braking available)
```

**Key challenge:** No direct slip measurement — inferred from wheel speed derivatives and vehicle speed estimation. The algorithm must be robust to sensor noise and road surface changes.

### TCS (Traction Control System)

**Problem:** Excess drive torque causes wheel spin (lambda > peak). Beyond the peak, more torque = LESS traction.

**Solution:** Limit drive torque to keep slip below peak.

**Actuators (fastest to slowest):**
1. Motor torque reduction (EV): < 5 ms
2. Spark retard: ~10 ms (reduces torque ~30%)
3. Fuel injection cut: ~50 ms (cylinder-by-cylinder)
4. Throttle closure: ~200 ms
5. Brake application to spinning wheel: ~50 ms

**Control structure:** Often a cascaded loop — outer loop sets target slip, inner loop modulates torque to achieve it. Gain scheduling based on estimated road friction.

### ESP/ESC (Electronic Stability Program)

**Problem:** Vehicle yaw rate deviates from driver's intended path — spin (oversteer) or plough (understeer).

**Solution:** Compare measured yaw rate to reference model, apply differential braking to correct.

**Architecture:**
```
                    ┌─────────────────┐
delta ──────────────┤ Bicycle Model   ├──── r_desired
                    │ (reference)     │
                    └─────────────────┘
                              │
                              v
              ┌───────────────────────────────┐
r_measured ──►│ Error = r_meas - r_des        │──► Brake which wheel?
              │ + sideslip observer (beta_est) │    How much pressure?
              └───────────────────────────────┘
```

**Sideslip estimation:** beta is not directly measured (no production sensor). Estimated via observer combining:
- Gyroscope (yaw rate) — integrating gives heading, difference from velocity vector gives beta
- Accelerometer (lateral acceleration)
- Wheel speeds (vehicle speed and slip)
- Steering angle

Observer design: typically EKF or Luenberger with nonlinear tyre model.

### Active Suspension

**Problem:** Passive springs/dampers compromise between ride comfort (soft) and handling (stiff).

**Solution:** Actively controlled force elements that adapt in real-time.

**Skyhook damping concept:**
```
F_damper = -c_sky * v_body    (damp body velocity relative to inertial frame)
```

Rather than damping relative wheel-body velocity (conventional shock), damp the absolute body velocity. This is physically impossible with passive elements but achievable with semi-active or active actuators.

**Semi-active (adjustable damper):** Can only dissipate energy (force in same direction as relative velocity). Implements skyhook when possible, zero force otherwise:
```
if v_body * v_relative > 0:   F = c_sky * v_body  (skyhook achievable)
else:                          F = 0               (minimum damping)
```

**Fully active:** Hydraulic or electric actuator — can inject energy. Enables perfect isolation but requires significant power.

### EPS (Electric Power Steering)

**Problem:** Provide steering assistance that feels natural across all speeds.

**Solution:** Speed-dependent gain scheduling — heavy assist at low speed (parking), light assist at high speed (stability).

```
T_assist = K(V) * T_driver + T_damping + T_return
```

- K(V): gain that decreases with vehicle speed
- T_damping: electronic damping to prevent oscillation
- T_return: active return-to-center torque

Modern EPS also enables:
- Lane-keeping assist (adds steering torque toward lane center)
- Active steering for ESP (applies counter-steer automatically)
- Variable ratio steering (software gear ratio change)

### Torque Vectoring

**Problem:** In cornering, a conventional differential distributes torque equally — no yaw moment contribution.

**Solution:** Actively control left/right torque distribution to create a yaw moment.

```
M_yaw = (T_right - T_left) * t / (2 * R_wheel)
```

**Implementation options:**
- Twin electric motors (one per wheel) — direct and fast
- Active differential with clutch packs — mechanical, adds to existing diff
- Brake-based (poor man's torque vectoring) — wastes energy but requires no additional hardware

**Control allocation problem:** Given a desired total force Fx and yaw moment Mz, find individual wheel torques [T_FL, T_FR, T_RL, T_RR] subject to traction circle constraints on each wheel.

### ACC / ADAS

**Problem:** Maintain desired speed and safe following distance automatically.

**Solution:** Cascaded control — outer loop controls gap/speed, inner loop controls acceleration via throttle and brake.

```
Outer loop:  a_desired = f(distance_error, relative_speed)     [~5 Hz]
Inner loop:  throttle/brake = g(a_desired, a_measured)          [~50 Hz]
```

**MPC approach (modern):** Plan a trajectory over a prediction horizon considering:
- Speed limits, curvature constraints
- Preceding vehicle predicted motion
- Comfort constraints (jerk limits)
- Fuel efficiency

## Cross-Cutting Themes

### Sensor Fusion

Every system relies on multiple sensors combined via observers/filters:
- Wheel speed sensors (4x): fundamental for ABS, TCS, ESP, speed estimation
- IMU (gyro + accelerometer): yaw rate, lateral acceleration, body motion
- Steering angle sensor: driver intent
- Brake pressure sensor: actual braking force
- GPS + camera + radar/lidar: ADAS-level perception

### Actuator Coordination

Multiple systems share the same actuators (especially brakes and engine torque). A coordination layer prioritises:

```
Priority: ABS > ESP > TCS > ACC > driver request
```

If ESP demands brake pressure on the front-left, it overrides the driver's brake pedal distribution. Safety-critical systems always win.

### Fail-Safe Design

All safety-critical systems must handle sensor failure gracefully:
- ABS: if wheel speed sensor fails, disable ABS on that corner (revert to manual braking)
- ESP: if gyro fails, disable ESP entirely (warn driver)
- EPS: if motor fails, mechanical connection preserved (heavy steering but still functional)

## Implementation Notes

### System Architecture Pattern

Most vehicle control systems follow this pattern:

```cpp
struct VehicleController {
    // 1. State estimation (observer)
    virtual void estimate_state(const SensorData& sensors) = 0;

    // 2. Reference generation (what the system should do)
    virtual void compute_reference(const DriverInput& input) = 0;

    // 3. Error computation
    virtual void compute_error() = 0;

    // 4. Control law (how to achieve it)
    virtual ActuatorCommand compute_command() = 0;

    // 5. Safety limits and coordination
    virtual ActuatorCommand apply_limits(ActuatorCommand cmd) = 0;
};
```

This maps directly to the standard control architecture: observer + reference + controller + saturation.
