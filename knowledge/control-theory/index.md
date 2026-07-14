# Control Theory

Feedback control, state estimation, and stability analysis.

## Subcategories

| Subcategory | Path | Description |
|---|---|---|
| Controllers | [controllers/](controllers/index.md) | PID, LQR, SMC, MPC, Gain Scheduling — feedback control algorithms |
| Observers | [observers/](observers/index.md) | Luenberger, Kalman filter, EKF — state estimation from noisy measurements |
| Worked Examples | [examples/](examples/) | Step-by-step derivations: double mass-spring-damper, inverted pendulum, quarter-car |
| Design Problems | [design-problems/](design-problems/) | Automotive design walkthroughs applying the 6-step framework: traction control, active suspension, hybrid torque split |

## Topics (Root Level)

| Topic | Description |
|---|---|
| [State-Space Representation](state-space.md) | ABCD form, controllability, observability, discretization methods |
| [Stability Analysis](stability.md) | Eigenvalue criterion, Lyapunov theory, Routh-Hurwitz, stability margins |
| [Frequency Response & Bode Plots](frequency-response.md) | Transfer functions, factored pole-zero form, Bode magnitude/phase plots, gain/phase margins, ODE-to-G(s) via Laplace |
| [Trajectory Planning](trajectory-planning.md) | Polynomial, minimum-jerk, and trapezoidal velocity profiles for smooth setpoint transitions |
| [Second-Order Systems](second-order-systems.md) | Standard form, overshoot/settling/rise time formulas, pole location to behaviour mapping, dominant poles |
| [Steady-State Error & System Types](steady-state-error.md) | Final Value Theorem, error constants, system type classification, tracking error prediction |
| [Compensator Design](compensator-design.md) | Lead, lag, lead-lag, notch compensators for loop shaping |
| [Nyquist Stability Criterion](nyquist.md) | Encirclement criterion, gain margin from Nyquist plot, conditional stability |
| [Root Locus Method](root-locus.md) | How closed-loop poles move with gain, essential construction rules |
| [First-Principles Modelling](first-principles-modelling.md) | Newton & Euler-Lagrange derivation pipelines, 3-minute checklist, state-space assembly pattern |
| [Control Design Framework](design-framework.md) | 6-step methodology for any control design problem: requirements, modelling, C/O assessment, architecture, hard parts, validation |
