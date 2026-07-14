# Controllers

Feedback control algorithms — from classical PID to optimal LQR, observer-based LQG, robust H-infinity, nonlinear SMC, gain-scheduled designs, and constrained MPC.

## Topics

| Topic | Description |
|---|---|
| [PID Control](pid.md) | Proportional-Integral-Derivative control law, anti-windup, tuning diagnostics, automotive applications, derivative filtering |
| [Linear-Quadratic Regulator](lqr.md) | Optimal state-feedback control via algebraic Riccati equation, Q/R tuning |
| [LQG](lqg.md) | LQR + Kalman combined via separation principle — for when states aren't all measured |
| [H-infinity](h-infinity.md) | Worst-case robust control overview — guaranteed performance under model uncertainty |
| [Sliding Mode Control](sliding-mode.md) | Nonlinear variable-structure control, robustness via sliding surfaces, chattering solutions |
| [Gain Scheduling](gain-scheduling.md) | Practical nonlinear control via interpolated linear designs |
| [MPC](mpc.md) | Constrained receding-horizon optimal control |
| [Controller Selection Guide](comparison.md) | Comparison table and decision tree for choosing between all methods |

## Design Methodology

Controller selection is Step 4 of the [6-step control design framework](../design-framework.md). For worked examples applying the full framework to automotive problems, see the [design problems](../design-problems/) directory.
