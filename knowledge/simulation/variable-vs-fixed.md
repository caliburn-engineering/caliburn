---
title: Fixed vs. Variable Timestep
sources:
  - { note: "engineering experience" }
  - { book: "Fedorov - Yet Another Introduction to Numerical Methods", chapter: "4", pages: "43-46" }
requires:
  - rk4.md
related:
  - fixed-timestep.md
  - sim-loop.md
---

# Fixed vs. Variable Timestep

The timestep strategy is one of the first architectural decisions in any simulation. It determines whether physics advances in uniform increments (fixed) or adapts step size based on error estimates (variable/adaptive). The choice cascades into reproducibility, debugging, control system design, and performance profiling.

**Recommendation:** use fixed timestep for real-time control systems (ball-balancer). Use adaptive for offline simulation, parameter sweeps, or systems with widely varying timescales.

## Key Equations

### Fixed Timestep

```
t_{n+1} = t_n + dt          (dt constant, chosen at design time)
y_{n+1} = integrator(y_n, t_n, dt)
```

The total number of steps to simulate interval [0, T] is exactly N = T / dt. Execution time is predictable and bounded.

### Variable (Adaptive) Timestep

Embedded RK methods produce two solutions of different orders from the same function evaluations. The difference estimates the local error:

```
e_n = y_{high} - y_{low}
```

Step acceptance criterion:

```
if ||e_n|| < tol:
    accept step, advance t by h
else:
    reject step, retry with smaller h
```

Step size adjustment (PI controller for stability):

```
h_{next} = h * safety * (tol / ||e_n||)^(1/(p+1))
```

where `p` is the order of the lower method and `safety` is typically 0.8-0.95 to avoid oscillation between acceptance and rejection.

### Common Embedded Pairs

| Method | Orders | Evals/step | Notes |
|---|---|---|---|
| Bogacki-Shampine (RK23) | 3(2) | 3 (FSAL) | Cheap, good for smooth problems |
| Runge-Kutta-Fehlberg (RKF45) | 5(4) | 6 | scipy/GSL default |
| Dormand-Prince (DOPRI54) | 5(4) | 7 (FSAL) | MATLAB ode45 default |
| Cash-Karp | 5(4) | 6 | Alternative to RKF45 |

FSAL = First Same As Last: the final evaluation of step n is reused as the first evaluation of step n+1, saving one function call per accepted step.

## Implementation Notes

### Fixed Timestep: Strengths

- **Deterministic:** identical inputs always produce identical outputs. Essential for replay-based debugging, networked synchronization, and regression testing.
- **Predictable CPU cost:** each frame has a bounded number of substeps. No risk of adaptive refinement consuming the entire frame budget.
- **Simple to implement:** no error estimation, no step rejection logic, no PI controller.
- **Control system compatibility:** PID controllers, Kalman filters, and discrete-time control laws assume a constant sample period. Variable dt requires re-deriving the discrete controller for each step.
- **Debuggable:** step through simulation tick-by-tick, compare state at tick N across runs.

### Fixed Timestep: Weaknesses

- **Wastes computation on smooth intervals:** the system may be nearly static for long periods but still takes full-cost steps.
- **Accuracy limited by worst case:** dt must be small enough for the stiffest/fastest mode, even when that mode is inactive.
- **No error feedback:** if dt is too large, results silently degrade. No built-in warning.

### Variable Timestep: Strengths

- **Accuracy-driven:** the integrator automatically takes small steps where the solution changes rapidly and large steps where it is smooth.
- **Efficiency for offline simulation:** a parameter sweep over 10,000 seconds of sim time benefits enormously from large steps during quiescent periods.
- **Built-in error estimate:** the embedded pair provides a local error bound at every step. No silent degradation.

### Variable Timestep: Weaknesses

- **Non-deterministic execution time:** a stiff transient can trigger thousands of rejected steps mid-frame, blowing the real-time budget.
- **Not reproducible across platforms:** different floating-point behavior can cause different step acceptance decisions, producing different trajectories from identical initial conditions.
- **Control system headaches:** discrete-time controllers derived assuming constant dt must be re-formulated for variable dt (continuous-time equivalent + discretize at each step, or use a continuous-time controller).
- **More complex code:** step acceptance, rejection, error estimation, safety factors, minimum/maximum step bounds.

### Adaptive RK4 (When Fixed Is Not Enough)

If a fixed-step RK4 simulation shows energy drift or inaccuracy in specific regimes, the cheapest upgrade is Richardson extrapolation (step doubling):

```cpp
// Take one full step of size h
State y_full = rk4_step(y, t, h, f);

// Take two half-steps of size h/2
State y_half = rk4_step(y, t, h/2, f);
y_half = rk4_step(y_half, t + h/2, h/2, f);

// Error estimate (difference between the two approaches)
double error = (y_half - y_full).norm();

// Richardson-extrapolated result (5th-order accurate)
State y_improved = y_half + (y_half - y_full) / 15.0;
```

This costs 12 function evaluations per step (vs. 4 for plain RK4) but provides an error estimate without switching to a different integrator.

### Hybrid Approach: Fixed Outer, Adaptive Inner

For the ball-balancer, a pragmatic hybrid:

- **Outer loop:** fixed dt = 0.01 s (100 Hz), drives the control system at constant rate
- **Inner substeps:** if contact detection or bounce events need finer resolution, subdivide the dt interval adaptively within the physics step

This preserves control-loop determinism while allowing the contact model to handle fast transients. In practice, the ball-balancer's small-angle dynamics are smooth enough that plain fixed-step RK4 suffices.

### Decision Matrix

| Criterion | Fixed | Variable | Winner for ball-balancer |
|---|---|---|---|
| Real-time guarantee | Yes | No | Fixed |
| Determinism / replay | Yes | Platform-dependent | Fixed |
| Control system integration | Native | Requires adaptation | Fixed |
| Accuracy per CPU cycle | Lower | Higher | Variable (but irrelevant at 100 Hz) |
| Implementation complexity | Low | Medium-High | Fixed |
| Offline parameter sweeps | Wasteful | Efficient | Variable |

For the ball-balancer simulator: **fixed timestep, no question.** For an offline dynamics analysis tool that sweeps plate angles or ball masses: adaptive.
