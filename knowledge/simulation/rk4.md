---
title: Runge-Kutta 4th Order Integration
sources:
  - { book: "Fedorov - Yet Another Introduction to Numerical Methods", chapter: "4", pages: "35-46" }
  - { book: "Kreyszig - Advanced Engineering Mathematics", chapter: "21" }
  - { note: "engineering experience" }
requires:
  - ../math/linear-algebra/matrix-operations.md
related:
  - fixed-timestep.md
  - sim-loop.md
reference: ../../reference/integrators/rk4.h
---

# Runge-Kutta 4th Order Integration

RK4 is the standard numerical integrator for physics simulation. It solves initial value problems of the form **y'(t) = f(t, y)** by evaluating the derivative at four carefully chosen points within each timestep, then combining them with Simpson's-rule-like weights. Self-starting (no history needed), explicit (no implicit solve), and the best accuracy-to-cost ratio for smooth ODE systems.

In the ball-balancer, RK4 integrates a 6D state vector (x, y, z, vx, vy, vz) at 100 Hz. It is the computational core of every physics tick.

## Key Equations

### The Four Stages

Given current state y_n at time t_n with step size h:

```
k1 = f(t_n,       y_n)
k2 = f(t_n + h/2, y_n + (h/2) * k1)
k3 = f(t_n + h/2, y_n + (h/2) * k2)
k4 = f(t_n + h,   y_n + h * k3)

y_{n+1} = y_n + (h/6)(k1 + 2*k2 + 2*k3 + k4)
```

### Butcher Tableau

```
  0  |
 1/2 | 1/2
 1/2 |  0   1/2
  1  |  0    0    1
     | 1/6  1/3  1/3  1/6
```

The weights 1/6, 1/3, 1/3, 1/6 are the signature pattern -- they produce a weighted average of the four slope estimates equivalent to Simpson's rule.

### Error Order

- **Local truncation error:** O(h^5) per step
- **Global accumulated error:** O(h^4) over the integration interval
- 4 function evaluations per step (vs. 1 for Euler, 6 for RKF45)

### Applying to Systems of ODEs

Any higher-order ODE reduces to a first-order system. For a second-order ODE u'' = g(t, u, u'), introduce y1 = u, y2 = u':

```
y1' = y2
y2' = g(t, y1, y2)
```

Each k-stage evaluates ALL components simultaneously. For an N-dimensional state vector, each k_i is itself an N-vector. The full update applies the RK4 formula component-wise.

### When to Use RK4 vs. Other Integrators

| Scenario | Integrator | Reason |
|---|---|---|
| Real-time physics (ball-balancer) | **RK4 fixed-step** | Predictable execution time, sufficient accuracy |
| Offline high-accuracy simulation | RKF45 / Dormand-Prince | Adaptive step size, embedded error estimate |
| Quick prototyping | Euler-Cromer | 1 eval/step, symplectic (conserves energy for oscillators) |
| Stiff systems (chemical kinetics) | Implicit methods (BDF) | Explicit methods require impractically small h |
| Long-duration orbital mechanics | Symplectic (Stormer-Verlet) | Energy conservation over millions of steps |

RK4 is NOT adaptive -- it has no built-in error estimate. If you need step-size control, use an embedded pair like Bogacki-Shampine (RK23, 3 evals with FSAL) or RKF45 (6 evals).

## Implementation Notes

### Eigen-Based State Vector Pattern

```cpp
using State = Eigen::VectorXd;  // or Eigen::Vector<double, 6> for ball-balancer

State rk4_step(
    const State& y,
    double t,
    double h,
    std::function<State(double, const State&)> f)
{
    State k1 = f(t,         y);
    State k2 = f(t + h/2.0, y + (h/2.0) * k1);
    State k3 = f(t + h/2.0, y + (h/2.0) * k2);
    State k4 = f(t + h,     y + h * k3);

    return y + (h / 6.0) * (k1 + 2.0*k2 + 2.0*k3 + k4);
}
```

Using Eigen vectors eliminates manual component-by-component code. The arithmetic operators (+, *, scalar multiplication) apply element-wise automatically.

### Ball-Balancer 6D State Pattern

State: `[x, y, z_ball, vx, vy, vz_ball]`

```cpp
// Derivatives function: given state + table orientation, return d(state)/dt
State ball_derivatives(double t, const State& s, const TableState& table) {
    State dsdt(6);
    dsdt[0] = s[3];  // dx/dt = vx
    dsdt[1] = s[4];  // dy/dt = vy
    dsdt[2] = s[5];  // dz/dt = vz
    dsdt[3] = compute_ax(s, table);  // dvx/dt = ax
    dsdt[4] = compute_ay(s, table);  // dvy/dt = ay
    dsdt[5] = compute_az(s, table);  // dvz/dt = az
    return dsdt;
}
```

The dynamics function encodes the rolling physics (5/7 factor for solid sphere), contact detection, viscous friction, and bounce model.

### Numerical Stability Considerations

- **Timestep selection rule:** h < T_fast / 10, where T_fast is the period of the fastest mode in the system. Ball-balancer at 100 Hz (dt = 0.01 s) satisfies this for typical ball dynamics.
- **Stability bound:** for Euler, h < 2/|lambda_max| where lambda_max is the largest eigenvalue of the system Jacobian. RK4 has a wider stability region but the same qualitative constraint applies.
- **Energy drift check:** for conservative (undamped) systems, track E(t) - E(0). Growing energy drift signals h is too large or there is a bug in the derivatives function.
- **Use `double` throughout.** Single-precision `float` causes visible energy drift and position errors within seconds at typical simulation rates.
- **Avoid heap allocation in the hot loop.** Use fixed-size `Eigen::Vector<double, N>` rather than dynamic `Eigen::VectorXd` if the state dimension is known at compile time.
