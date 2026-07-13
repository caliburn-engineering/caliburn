# Observers

State estimation algorithms — reconstructing system state from noisy measurements.

## Topics

| Topic | Description |
|---|---|
| [Luenberger Observer](luenberger.md) | Deterministic state estimation via pole placement — the foundation that Kalman builds on |
| [Discrete-Time Kalman Filter](kalman-filter.md) | Predict-update cycle for linear state estimation |
| [Extended Kalman Filter](extended-kalman.md) | Kalman filter for nonlinear systems via Jacobian linearization |

## Automotive Observer Applications

| Application | What's estimated | Sensors used | Observer type |
|---|---|---|---|
| ESP/ESC sideslip | Lateral velocity (sideslip angle) | IMU (yaw rate, lateral accel) + wheel speeds | EKF on bicycle model |
| Battery SoC | State of charge, internal resistance | Terminal voltage, current | EKF on equivalent circuit model |
| Active suspension | Tyre deflection, road profile | Body accelerometer, suspension travel | Luenberger or Kalman on quarter-car |
| Tyre force estimation | Longitudinal/lateral tyre forces | Wheel speeds, IMU | UKF or sliding mode observer |
| Motor temperature | Rotor temperature (can't measure directly) | Stator temperature, current, voltage | Luenberger on thermal model |
