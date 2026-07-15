# Linearization Tooling & Ball-Balancer Linear Model

**Date:** 2026-07-15
**Status:** Draft
**Project:** Caliburn

## Purpose

Provide tooling to linearize nonlinear plant models and validate the resulting linear state-space representation. The first application is the ball-balancer: derive its linear plant model, validate it numerically, and compare linear vs nonlinear time responses in the existing visualizer.

This is the prerequisite step before the Linear System Analyzer (next project), which will visualize pole-zero maps, Bode plots from state-space, controllability/observability, and time responses — all of which require a LinearSystem as input.

## Approach

Analytical-first, numerical validation (Approach C from brainstorming). The user hand-derives A, B from first principles. A numerical `Linearizer` computes them independently via central finite differences. A `validate()` function checks element-wise agreement. Both models are then simulated with identical inputs and their responses overlaid for visual comparison.

## Deliverable 1: LinearSystem Struct

**Path:** `reference/models/linear_system.h`

The universal container for a linearized model. Used by all downstream analysis tools.

```cpp
namespace caliburn {

struct LinearSystem {
    Eigen::MatrixXd A;  // n x n state matrix
    Eigen::MatrixXd B;  // n x m input matrix
    Eigen::MatrixXd C;  // p x n output matrix
    Eigen::MatrixXd D;  // p x m feedthrough matrix

    int states() const { return A.rows(); }
    int inputs() const { return B.cols(); }
    int outputs() const { return C.rows(); }
};

}  // namespace caliburn
```

Header-only. No dependencies beyond Eigen.

## Deliverable 2: Numerical Linearizer

**Path:** `reference/models/linearizer.h`, `reference/models/linearizer.cpp`

### Function signature

```cpp
namespace caliburn {

using NonlinearFn = std::function<Eigen::VectorXd(
    const Eigen::VectorXd& state,
    const Eigen::VectorXd& input)>;

// Compute A = df/dx, B = df/du at (x0, u0) via central finite differences.
// Returns LinearSystem with A, B computed numerically and C = I, D = 0 (full state output).
LinearSystem linearize(
    const NonlinearFn& f,
    const Eigen::VectorXd& x0,
    const Eigen::VectorXd& u0,
    double epsilon = 1e-6);

// Overload: caller provides C and D (when only a subset of states is measured).
LinearSystem linearize(
    const NonlinearFn& f,
    const Eigen::VectorXd& x0,
    const Eigen::VectorXd& u0,
    const Eigen::MatrixXd& C,
    const Eigen::MatrixXd& D,
    double epsilon = 1e-6);
```

### Algorithm

For each state dimension i:
```
A(:, i) = (f(x0 + eps*e_i, u0) - f(x0 - eps*e_i, u0)) / (2 * eps)
```

For each input dimension j:
```
B(:, j) = (f(x0, u0 + eps*e_j) - f(x0, u0 - eps*e_j)) / (2 * eps)
```

Central differences give O(eps^2) accuracy.

### Validation

```cpp
struct ValidationResult {
    Eigen::MatrixXd A_error;  // element-wise |analytical - numerical|
    Eigen::MatrixXd B_error;
    double max_A_error;       // A_error.maxCoeff()
    double max_B_error;       // B_error.maxCoeff()
    bool pass;                // both maxes < tolerance
};

ValidationResult validate(
    const LinearSystem& analytical,
    const NonlinearFn& f,
    const Eigen::VectorXd& x0,
    const Eigen::VectorXd& u0,
    double tol = 1e-4);
```

## Deliverable 3: Ball-Balancer Linear Plant Model

**Path:** `projects/ball-balancer/src/ball_plant_linear.h`, `projects/ball-balancer/src/ball_plant_linear.cpp`

### Hand-derived linearization

Operating point: x0 = [0, 0, 0, 0] (ball at centre, zero velocity), u0 = [0, 0] (plate level).

Linearization of `RollingBallDynamics::derivatives`:
- sin(alpha) -> alpha, sin(beta) -> beta (small angle)
- Rolling friction drops out at zero velocity
- Rolling factor k = 5/7 (solid sphere)

```
State: x = [x, y, vx, vy]
Input: u = [alpha, beta]   (plate tilt angles)

A = | 0  0  1  0 |    B = | 0    0  |
    | 0  0  0  1 |        | 0    0  |
    | 0  0  0  0 |        | k*g  0  |
    | 0  0  0  0 |        | 0  k*g  |

C = | 1  0  0  0 |    D = | 0  0 |
    | 0  1  0  0 |        | 0  0 |
```

Where k*g = (5/7) * 9.81 = 7.007 m/s^2.

Axis mapping: beta (Y-axis tilt) drives x-acceleration, alpha (X-axis tilt) drives y-acceleration. This matches the existing `RollingBallDynamics` implementation.

### API

```cpp
namespace caliburn {

// Returns the hand-derived linear model for ball-on-plate
LinearSystem ballBalancerLinearModel(double gravity = 9.81);

// Wraps RollingBallDynamics::derivatives as a NonlinearFn for comparison
NonlinearFn ballBalancerNonlinearFn(const BallParams& ball, const PlateParams& plate);

}  // namespace caliburn
```

### Validation

Construction-time assertion in debug builds:
```cpp
auto result = validate(ballBalancerLinearModel(), ballBalancerNonlinearFn(ball, plate), x0, u0);
assert(result.pass);  // expect match to ~1e-8
```

## Deliverable 4: Comparison Simulation

**Path:** `projects/ball-balancer/src/comparison_sim.h`, `projects/ball-balancer/src/comparison_sim.cpp`

### Input signals

```cpp
namespace caliburn {

struct StepInput {
    Eigen::VectorXd amplitude;  // step size per input channel
    double start_time;          // when the step occurs
};

struct RampInput {
    Eigen::VectorXd slope;      // slope per input channel
    double start_time;
};

using CustomInput = std::function<Eigen::VectorXd(double t)>;

using InputSignal = std::variant<StepInput, RampInput, CustomInput>;
```

### Comparison runner

```cpp
struct ComparisonResult {
    std::vector<double> time;
    std::vector<Eigen::VectorXd> nonlinear_states;  // x_nl(t)
    std::vector<Eigen::VectorXd> linear_states;      // x_lin(t)
    std::vector<Eigen::VectorXd> inputs;              // u(t) at each step
};

ComparisonResult compareModels(
    const NonlinearFn& f_nl,
    const LinearSystem& lin,
    const Eigen::VectorXd& x0,
    const InputSignal& input,
    double dt,
    double duration);
```

Both models are integrated with the same RK4 integrator, same dt, same initial condition, same input signal. The nonlinear model uses the full `f_nl`. The linear model uses `x_dot = A*x + B*u`.

## Deliverable 5: Comparison Panel (Visualizer Integration)

**Path:** `projects/ball-balancer/src/comparison_panel.h`, `projects/ball-balancer/src/comparison_panel.cpp`

A new dockable ImGui window integrated into the existing ball-balancer visualizer. Not a separate executable.

### Controls

- **Input type selector:** Step / Ramp / Custom dropdown
- **Amplitude or slope sliders** per input channel (alpha, beta)
- **Duration slider** (1-30 seconds)
- **"Run Comparison" button** — executes `compareModels()` and populates plots
- **"Reset" button** — clears results

### Plots

Follow all `scaffold-sim-viewer` plot invariants, including the updated grouping rule:

**One plot per state type:**

| Plot | Traces | Legend entries |
|---|---|---|
| Position [m] | x_nl, y_nl, x_lin, y_lin | xN, yN, xL, yL |
| Velocity [m/s] | vx_nl, vy_nl, vx_lin, vy_lin | vxN, vyN, vxL, vyL |
| Input [rad] | alpha, beta | al, be |
| Error [m] (collapsible) | \|x_nl - x_lin\|, \|y_nl - y_lin\| | ex, ey |

**Model differentiation:** Nonlinear traces use solid lines, linear traces use dashed lines (or distinct color families: blue/orange for NL, green/red for LIN).

**Divergence shading:** Where any state error exceeds a configurable threshold, lightly shade the plot background. This makes it immediately visible where the linearization breaks down.

**All standard plot features apply:**
- Synchronized cursor across all plots
- Click-to-mark persistent markers
- Y-axis auto-fit to visible time window
- Legend outside, right-aligned, max 2-character labels
- Pause and time-zoom for inspection

### Validation result display

Below the plots, show the element-wise validation result:
- A_error and B_error matrices displayed in a small table
- Green/red pass/fail indicator
- Max error values

## Deliverable 6: Knowledge File

**Path:** `knowledge/control-theory/linearization.md`

Covers:
- Jacobian linearization theory (Taylor expansion, dropping higher-order terms)
- Small-signal assumption and validity region
- Numerical linearization via central differences
- When linearization fails (discontinuities, non-smooth dynamics like Coulomb friction)
- Worked example: ball-on-plate linearization (pointing to ball_plant_linear.h)
- Connection to system type and transfer function derivation (links to steady-state-error.md, frequency-response.md)

Frontmatter:
```yaml
---
title: Linearization
sources:
  - { book: "Ogata — Modern Control Engineering", chapter: "3" }
  - { book: "Slotine & Li — Applied Nonlinear Control", chapter: "3" }
requires:
  - state-space.md
  - first-principles-modelling.md
related:
  - stability.md
  - frequency-response.md
  - controllers/gain-scheduling.md
---
```

Added to `knowledge/control-theory/index.md` topics table.

## Deliverable 7: Tests

**Path:** `reference/models/test_linearizer.cpp`

| Test | What it validates |
|---|---|
| Mass-spring-damper | Linearizer on a known linear system returns exact A, B |
| Ball-balancer analytical vs numerical | `validate()` passes with tolerance 1e-6 |
| Ball-balancer step response | Linear and nonlinear agree within 1% for small step (0.01 rad, 1s) |
| Ball-balancer large step divergence | Linear and nonlinear diverge for large step (0.5 rad) — confirms nonlinearity matters |
| Zero perturbation | Edge case: x0 = 0, u0 = 0, verify no division by zero |

## Out of Scope

- Generalized model-loading UI (future — when building the Linear System Analyzer)
- Controller or observer design (next project after Linear System Analyzer)
- Transfer function extraction from state-space (Linear System Analyzer scope)
- Automated operating-point sweep or gain scheduling (future)

## Build Order

1. `linear_system.h` — the struct everything depends on
2. `linearizer.h/.cpp` + `test_linearizer.cpp` — reusable utility, tested standalone
3. `ball_plant_linear.h/.cpp` — hand-derived model, validated against linearizer
4. `linearization.md` — knowledge file documenting the theory
5. `comparison_sim.h/.cpp` — simulation runner
6. `comparison_panel.h/.cpp` — visualizer integration
7. Update ball-balancer CMakeLists.txt and AGENTS.md
