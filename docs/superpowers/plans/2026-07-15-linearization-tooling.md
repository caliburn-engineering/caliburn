# Linearization Tooling & Ball-Balancer Linear Model — Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Build a reusable numerical linearizer, hand-derive the ball-balancer linear plant model, validate it, and add a comparison panel to the ball-balancer visualizer showing linear vs nonlinear time responses.

**Architecture:** A `LinearSystem` struct is the universal container. A `Linearizer` computes A,B numerically via central differences. The ball-balancer gets a hand-derived `LinearSystem` validated against the numerical result. A `ComparisonSim` runs both models with identical inputs, and a `ComparisonPanel` renders the overlaid responses in the existing visualizer.

**Tech Stack:** C++17, Eigen 3.4, ImGui (docking), ImPlot, RK4 integrator (existing)

---

## File Map

| File | Action | Responsibility |
|---|---|---|
| `reference/models/linear_system.h` | Create | `LinearSystem` struct (header-only) |
| `reference/models/linearizer.h` | Create | `linearize()`, `validate()` declarations + `NonlinearFn` typedef |
| `reference/models/linearizer.cpp` | Create | Central-difference implementation |
| `reference/models/test_linearizer.cpp` | Create | 5 test cases |
| `reference/CMakeLists.txt` | Modify | Add `test_linearizer` target |
| `projects/ball-balancer/src/ball_plant_linear.h` | Create | `ballBalancerLinearModel()`, `ballBalancerNonlinearFn()` declarations |
| `projects/ball-balancer/src/ball_plant_linear.cpp` | Create | Hand-derived A,B,C,D + NonlinearFn wrapper |
| `projects/ball-balancer/src/comparison_sim.h` | Create | `InputSignal`, `ComparisonResult`, `compareModels()` |
| `projects/ball-balancer/src/comparison_sim.cpp` | Create | Dual-model RK4 simulation runner |
| `projects/ball-balancer/src/comparison_panel.h` | Create | `ComparisonPanel` class declaration |
| `projects/ball-balancer/src/comparison_panel.cpp` | Create | ImGui/ImPlot comparison panel |
| `projects/ball-balancer/src/visualizer.cpp` | Modify | Wire in `ComparisonPanel` |
| `projects/ball-balancer/CMakeLists.txt` | Modify | Add new source files to targets |
| `knowledge/control-theory/linearization.md` | Create | Linearization theory knowledge file |
| `knowledge/control-theory/index.md` | Modify | Add linearization entry |
| `projects/ball-balancer/AGENTS.md` | Modify | Update status for ball dynamics |

---

### Task 1: LinearSystem Struct

**Files:**
- Create: `reference/models/linear_system.h`

- [ ] **Step 1: Write `linear_system.h`**

```cpp
#pragma once

#include <Eigen/Core>

namespace caliburn {

struct LinearSystem {
    Eigen::MatrixXd A;  // n x n state matrix
    Eigen::MatrixXd B;  // n x m input matrix
    Eigen::MatrixXd C;  // p x n output matrix
    Eigen::MatrixXd D;  // p x m feedthrough matrix

    int states() const { return static_cast<int>(A.rows()); }
    int inputs() const { return static_cast<int>(B.cols()); }
    int outputs() const { return static_cast<int>(C.rows()); }
};

}  // namespace caliburn
```

- [ ] **Step 2: Verify it compiles**

Run: `cd /home/nds/Desktop/Merlin/04-projects/Caliburn/reference && cmake -B build && cmake --build build 2>&1 | tail -5`
Expected: Build succeeds (header-only, no new target yet — just checking it doesn't break includes)

- [ ] **Step 3: Commit**

```bash
git add reference/models/linear_system.h
git commit -m "feat: add LinearSystem struct — universal state-space container"
```

---

### Task 2: Numerical Linearizer — Tests First

**Files:**
- Create: `reference/models/linearizer.h`
- Create: `reference/models/test_linearizer.cpp`
- Modify: `reference/CMakeLists.txt`

- [ ] **Step 1: Write `linearizer.h` with declarations only**

```cpp
#pragma once

#include "linear_system.h"

#include <Eigen/Core>
#include <functional>

namespace caliburn {

/// Nonlinear system: f(state, input) -> state_derivative
using NonlinearFn = std::function<Eigen::VectorXd(
    const Eigen::VectorXd& state,
    const Eigen::VectorXd& input)>;

/// Compute A = df/dx, B = df/du at (x0, u0) via central finite differences.
/// Returns LinearSystem with C = I, D = 0 (full state output).
LinearSystem linearize(
    const NonlinearFn& f,
    const Eigen::VectorXd& x0,
    const Eigen::VectorXd& u0,
    double epsilon = 1e-6);

/// Overload: caller provides C and D.
LinearSystem linearize(
    const NonlinearFn& f,
    const Eigen::VectorXd& x0,
    const Eigen::VectorXd& u0,
    const Eigen::MatrixXd& C,
    const Eigen::MatrixXd& D,
    double epsilon = 1e-6);

/// Element-wise comparison of analytical vs numerical linearization.
struct ValidationResult {
    Eigen::MatrixXd A_error;  // |analytical.A - numerical.A|
    Eigen::MatrixXd B_error;  // |analytical.B - numerical.B|
    double max_A_error;
    double max_B_error;
    bool pass;                // both maxes < tolerance
};

ValidationResult validate(
    const LinearSystem& analytical,
    const NonlinearFn& f,
    const Eigen::VectorXd& x0,
    const Eigen::VectorXd& u0,
    double tol = 1e-4);

}  // namespace caliburn
```

- [ ] **Step 2: Write `test_linearizer.cpp` with 5 test cases**

```cpp
#include "linearizer.h"

#include <cassert>
#include <cmath>
#include <cstdio>

using namespace caliburn;

static constexpr double TOL = 1e-8;
static constexpr double G = 9.81;
static constexpr double K = 5.0 / 7.0;

// ---- Test 1: Mass-spring-damper (already linear — should recover exact A, B) ----
//
// mx'' + bx' + kx = u
// State: [x, v], Input: [u]
// A = [0, 1; -k/m, -b/m], B = [0; 1/m]
static void test_mass_spring_damper() {
    double m = 2.0, b = 0.5, k = 10.0;

    NonlinearFn f = [=](const Eigen::VectorXd& x, const Eigen::VectorXd& u) {
        Eigen::VectorXd dx(2);
        dx(0) = x(1);
        dx(1) = (-k * x(0) - b * x(1) + u(0)) / m;
        return dx;
    };

    Eigen::VectorXd x0 = Eigen::VectorXd::Zero(2);
    Eigen::VectorXd u0 = Eigen::VectorXd::Zero(1);

    auto sys = linearize(f, x0, u0);

    assert(std::abs(sys.A(0, 0) - 0.0) < TOL);
    assert(std::abs(sys.A(0, 1) - 1.0) < TOL);
    assert(std::abs(sys.A(1, 0) - (-k / m)) < TOL);
    assert(std::abs(sys.A(1, 1) - (-b / m)) < TOL);
    assert(std::abs(sys.B(0, 0) - 0.0) < TOL);
    assert(std::abs(sys.B(1, 0) - (1.0 / m)) < TOL);

    // C should be identity, D should be zero
    assert(sys.C.rows() == 2 && sys.C.cols() == 2);
    assert(sys.D.rows() == 2 && sys.D.cols() == 1);
    assert((sys.C - Eigen::MatrixXd::Identity(2, 2)).norm() < TOL);
    assert(sys.D.norm() < TOL);

    std::printf("  [PASS] Mass-spring-damper — exact A, B recovered\n");
}

// ---- Test 2: Ball-balancer analytical vs numerical ----
static void test_ball_balancer_linearization() {
    // Nonlinear ball-on-plate: f(x, u) with sin(alpha), sin(beta)
    // No friction for clean comparison
    NonlinearFn f = [](const Eigen::VectorXd& x, const Eigen::VectorXd& u) {
        Eigen::VectorXd dx(4);
        dx(0) = x(2);  // vx
        dx(1) = x(3);  // vy
        dx(2) = K * G * std::sin(u(1));  // ax from beta (Y-axis tilt)
        dx(3) = K * G * std::sin(u(0));  // ay from alpha (X-axis tilt)
        return dx;
    };

    // Hand-derived analytical model
    LinearSystem analytical;
    analytical.A = Eigen::MatrixXd::Zero(4, 4);
    analytical.A(0, 2) = 1.0;
    analytical.A(1, 3) = 1.0;

    analytical.B = Eigen::MatrixXd::Zero(4, 2);
    analytical.B(2, 1) = K * G;  // d(ax)/d(beta)
    analytical.B(3, 0) = K * G;  // d(ay)/d(alpha)

    analytical.C = Eigen::MatrixXd::Identity(4, 4);
    analytical.D = Eigen::MatrixXd::Zero(4, 2);

    Eigen::VectorXd x0 = Eigen::VectorXd::Zero(4);
    Eigen::VectorXd u0 = Eigen::VectorXd::Zero(2);

    auto result = validate(analytical, f, x0, u0, 1e-6);

    assert(result.pass);
    assert(result.max_A_error < 1e-8);
    assert(result.max_B_error < 1e-6);  // sin linearization via central diff

    std::printf("  [PASS] Ball-balancer analytical vs numerical (max_A=%.2e, max_B=%.2e)\n",
                result.max_A_error, result.max_B_error);
}

// ---- Test 3: Custom C and D ----
static void test_custom_output_matrices() {
    // Simple 2-state system, measure only first state
    NonlinearFn f = [](const Eigen::VectorXd& x, const Eigen::VectorXd& u) {
        Eigen::VectorXd dx(2);
        dx(0) = x(1);
        dx(1) = -x(0) + u(0);
        return dx;
    };

    Eigen::VectorXd x0 = Eigen::VectorXd::Zero(2);
    Eigen::VectorXd u0 = Eigen::VectorXd::Zero(1);

    Eigen::MatrixXd C(1, 2);
    C << 1.0, 0.0;
    Eigen::MatrixXd D = Eigen::MatrixXd::Zero(1, 1);

    auto sys = linearize(f, x0, u0, C, D);

    assert(sys.C.rows() == 1 && sys.C.cols() == 2);
    assert(std::abs(sys.C(0, 0) - 1.0) < TOL);
    assert(std::abs(sys.C(0, 1) - 0.0) < TOL);
    assert(sys.D.rows() == 1 && sys.D.cols() == 1);
    assert(sys.D.norm() < TOL);

    std::printf("  [PASS] Custom C and D passed through correctly\n");
}

// ---- Test 4: Nonlinear system — linearization at non-zero operating point ----
static void test_nonzero_operating_point() {
    // f(x, u) = [-x^2 + u], linearize at x0=2, u0=4 (equilibrium: -4+4=0)
    // A = df/dx = -2*x0 = -4, B = df/du = 1
    NonlinearFn f = [](const Eigen::VectorXd& x, const Eigen::VectorXd& u) {
        Eigen::VectorXd dx(1);
        dx(0) = -x(0) * x(0) + u(0);
        return dx;
    };

    Eigen::VectorXd x0(1);
    x0 << 2.0;
    Eigen::VectorXd u0(1);
    u0 << 4.0;

    auto sys = linearize(f, x0, u0);

    assert(std::abs(sys.A(0, 0) - (-4.0)) < 1e-6);
    assert(std::abs(sys.B(0, 0) - 1.0) < TOL);

    std::printf("  [PASS] Nonzero operating point (A=%.4f, B=%.4f)\n",
                sys.A(0, 0), sys.B(0, 0));
}

// ---- Test 5: Zero state/input — no division by zero ----
static void test_zero_operating_point() {
    NonlinearFn f = [](const Eigen::VectorXd& x, const Eigen::VectorXd& u) {
        Eigen::VectorXd dx(2);
        dx(0) = x(1) + u(0);
        dx(1) = -x(0);
        return dx;
    };

    Eigen::VectorXd x0 = Eigen::VectorXd::Zero(2);
    Eigen::VectorXd u0 = Eigen::VectorXd::Zero(1);

    auto sys = linearize(f, x0, u0);

    assert(std::abs(sys.A(0, 1) - 1.0) < TOL);
    assert(std::abs(sys.A(1, 0) - (-1.0)) < TOL);
    assert(std::abs(sys.B(0, 0) - 1.0) < TOL);

    std::printf("  [PASS] Zero operating point — no NaN or division by zero\n");
}

int main() {
    std::printf("Linearizer tests:\n");

    test_mass_spring_damper();
    test_ball_balancer_linearization();
    test_custom_output_matrices();
    test_nonzero_operating_point();
    test_zero_operating_point();

    std::printf("All linearizer tests passed.\n");
    return 0;
}
```

- [ ] **Step 3: Add CMake target**

Append to `reference/CMakeLists.txt`:

```cmake
# --- Models: Linearizer ---
add_executable(test_linearizer
    models/linearizer.cpp
    models/test_linearizer.cpp
)
target_include_directories(test_linearizer PRIVATE models)
target_link_libraries(test_linearizer PRIVATE Eigen3::Eigen)
add_test(NAME linearizer COMMAND test_linearizer)
```

- [ ] **Step 4: Run tests — they should fail (no implementation)**

Run: `cd /home/nds/Desktop/Merlin/04-projects/Caliburn/reference && cmake -B build && cmake --build build --target test_linearizer 2>&1 | tail -10`
Expected: Linker error — `linearize()` and `validate()` symbols not found

- [ ] **Step 5: Commit (red)**

```bash
git add reference/models/linearizer.h reference/models/test_linearizer.cpp reference/CMakeLists.txt
git commit -m "test: add linearizer test cases (red — no implementation)"
```

---

### Task 3: Numerical Linearizer — Implementation

**Files:**
- Create: `reference/models/linearizer.cpp`

- [ ] **Step 1: Implement `linearize()` and `validate()`**

```cpp
#include "linearizer.h"

namespace caliburn {

LinearSystem linearize(
    const NonlinearFn& f,
    const Eigen::VectorXd& x0,
    const Eigen::VectorXd& u0,
    double epsilon)
{
    int n = static_cast<int>(x0.size());
    int m = static_cast<int>(u0.size());

    Eigen::MatrixXd C = Eigen::MatrixXd::Identity(n, n);
    Eigen::MatrixXd D = Eigen::MatrixXd::Zero(n, m);

    return linearize(f, x0, u0, C, D, epsilon);
}

LinearSystem linearize(
    const NonlinearFn& f,
    const Eigen::VectorXd& x0,
    const Eigen::VectorXd& u0,
    const Eigen::MatrixXd& C,
    const Eigen::MatrixXd& D,
    double epsilon)
{
    int n = static_cast<int>(x0.size());
    int m = static_cast<int>(u0.size());

    LinearSystem sys;
    sys.A = Eigen::MatrixXd::Zero(n, n);
    sys.B = Eigen::MatrixXd::Zero(n, m);
    sys.C = C;
    sys.D = D;

    // A = df/dx via central differences
    for (int i = 0; i < n; ++i) {
        Eigen::VectorXd x_plus = x0;
        Eigen::VectorXd x_minus = x0;
        x_plus(i) += epsilon;
        x_minus(i) -= epsilon;

        Eigen::VectorXd f_plus = f(x_plus, u0);
        Eigen::VectorXd f_minus = f(x_minus, u0);

        sys.A.col(i) = (f_plus - f_minus) / (2.0 * epsilon);
    }

    // B = df/du via central differences
    for (int j = 0; j < m; ++j) {
        Eigen::VectorXd u_plus = u0;
        Eigen::VectorXd u_minus = u0;
        u_plus(j) += epsilon;
        u_minus(j) -= epsilon;

        Eigen::VectorXd f_plus = f(x0, u_plus);
        Eigen::VectorXd f_minus = f(x0, u_minus);

        sys.B.col(j) = (f_plus - f_minus) / (2.0 * epsilon);
    }

    return sys;
}

ValidationResult validate(
    const LinearSystem& analytical,
    const NonlinearFn& f,
    const Eigen::VectorXd& x0,
    const Eigen::VectorXd& u0,
    double tol)
{
    auto numerical = linearize(f, x0, u0, analytical.C, analytical.D);

    ValidationResult result;
    result.A_error = (analytical.A - numerical.A).cwiseAbs();
    result.B_error = (analytical.B - numerical.B).cwiseAbs();
    result.max_A_error = result.A_error.maxCoeff();
    result.max_B_error = result.B_error.maxCoeff();
    result.pass = (result.max_A_error < tol) && (result.max_B_error < tol);

    return result;
}

}  // namespace caliburn
```

- [ ] **Step 2: Build and run tests**

Run: `cd /home/nds/Desktop/Merlin/04-projects/Caliburn/reference && cmake -B build && cmake --build build --target test_linearizer && ./build/test_linearizer`
Expected:
```
Linearizer tests:
  [PASS] Mass-spring-damper — exact A, B recovered
  [PASS] Ball-balancer analytical vs numerical (max_A=0.00e+00, max_B=...)
  [PASS] Custom C and D passed through correctly
  [PASS] Nonzero operating point (A=-4.0000, B=1.0000)
  [PASS] Zero operating point — no NaN or division by zero
All linearizer tests passed.
```

- [ ] **Step 3: Commit (green)**

```bash
git add reference/models/linearizer.cpp
git commit -m "feat: implement numerical linearizer with central differences"
```

---

### Task 4: Ball-Balancer Linear Plant Model

**Files:**
- Create: `projects/ball-balancer/src/ball_plant_linear.h`
- Create: `projects/ball-balancer/src/ball_plant_linear.cpp`

- [ ] **Step 1: Write `ball_plant_linear.h`**

```cpp
#pragma once

#include "linear_system.h"
#include "linearizer.h"

namespace caliburn {

struct BallParams;
struct PlateParams;

/// Hand-derived linear model for ball rolling on tilted plate.
/// Operating point: ball at centre, plate level.
/// State: [x, y, vx, vy], Input: [alpha, beta]
/// Output: [x, y] (position only)
LinearSystem ballBalancerLinearModel(double gravity = 9.81);

/// Wrap RollingBallDynamics::derivatives as a NonlinearFn for comparison.
/// Drops the time argument (linearization is time-invariant).
NonlinearFn ballBalancerNonlinearFn(double ball_radius = 0.02,
                                    double ball_mass = 0.05,
                                    double rolling_friction = 0.0,
                                    double plate_half_width = 0.15,
                                    double gravity = 9.81);

}  // namespace caliburn
```

- [ ] **Step 2: Write `ball_plant_linear.cpp`**

```cpp
#include "ball_plant_linear.h"
#include "rolling_dynamics.h"

namespace caliburn {

LinearSystem ballBalancerLinearModel(double gravity) {
    constexpr double k = 5.0 / 7.0;  // rolling factor, solid sphere

    LinearSystem sys;

    // State: [x, y, vx, vy], Input: [alpha, beta]
    sys.A = Eigen::MatrixXd::Zero(4, 4);
    sys.A(0, 2) = 1.0;  // dx/dt = vx
    sys.A(1, 3) = 1.0;  // dy/dt = vy

    sys.B = Eigen::MatrixXd::Zero(4, 2);
    // beta (Y-axis tilt) drives x-acceleration: sin(beta) ≈ beta
    sys.B(2, 1) = k * gravity;
    // alpha (X-axis tilt) drives y-acceleration: sin(alpha) ≈ alpha
    sys.B(3, 0) = k * gravity;

    // Output: measure x, y positions
    sys.C = Eigen::MatrixXd::Zero(2, 4);
    sys.C(0, 0) = 1.0;
    sys.C(1, 1) = 1.0;

    sys.D = Eigen::MatrixXd::Zero(2, 2);

    return sys;
}

NonlinearFn ballBalancerNonlinearFn(double ball_radius,
                                    double ball_mass,
                                    double rolling_friction,
                                    double plate_half_width,
                                    double gravity) {
    BallParams ball{ball_radius, ball_mass, rolling_friction};
    PlateParams plate{plate_half_width, gravity};
    auto dynamics = std::make_shared<RollingBallDynamics>(ball, plate);

    return [dynamics](const Eigen::VectorXd& state, const Eigen::VectorXd& input)
        -> Eigen::VectorXd {
        return dynamics->derivatives(
            Eigen::Vector4d(state),
            input(0),   // alpha
            input(1));  // beta
    };
}

}  // namespace caliburn
```

- [ ] **Step 3: Verify compilation**

This file depends on both `reference/models/` headers and `reference/simulation/rolling_dynamics.h`. We'll wire up the CMake in Task 7. For now, verify syntax by checking the header parses:

Run: `cd /home/nds/Desktop/Merlin/04-projects/Caliburn && head -5 projects/ball-balancer/src/ball_plant_linear.h`
Expected: File exists with correct header

- [ ] **Step 4: Commit**

```bash
git add projects/ball-balancer/src/ball_plant_linear.h projects/ball-balancer/src/ball_plant_linear.cpp
git commit -m "feat: hand-derived ball-balancer linear plant model"
```

---

### Task 5: Comparison Simulation

**Files:**
- Create: `projects/ball-balancer/src/comparison_sim.h`
- Create: `projects/ball-balancer/src/comparison_sim.cpp`

- [ ] **Step 1: Write `comparison_sim.h`**

```cpp
#pragma once

#include "linear_system.h"
#include "linearizer.h"

#include <Eigen/Core>
#include <functional>
#include <variant>
#include <vector>

namespace caliburn {

// --- Input signal types ---

struct StepInput {
    Eigen::VectorXd amplitude;  // step size per input channel
    double start_time = 0.0;
};

struct RampInput {
    Eigen::VectorXd slope;  // slope per input channel
    double start_time = 0.0;
};

using CustomInput = std::function<Eigen::VectorXd(double t)>;
using InputSignal = std::variant<StepInput, RampInput, CustomInput>;

/// Evaluate an InputSignal at time t
Eigen::VectorXd evaluateInput(const InputSignal& signal, double t, int num_inputs);

// --- Comparison result ---

struct ComparisonResult {
    std::vector<double> time;
    std::vector<Eigen::VectorXd> nonlinear_states;
    std::vector<Eigen::VectorXd> linear_states;
    std::vector<Eigen::VectorXd> inputs;
};

/// Run both nonlinear and linear models with the same input and initial condition.
/// Both use RK4 integration with the same dt.
ComparisonResult compareModels(
    const NonlinearFn& f_nl,
    const LinearSystem& lin,
    const Eigen::VectorXd& x0,
    const InputSignal& input,
    double dt,
    double duration);

}  // namespace caliburn
```

- [ ] **Step 2: Write `comparison_sim.cpp`**

```cpp
#include "comparison_sim.h"
#include "rk4.h"

#include <cmath>

namespace caliburn {

Eigen::VectorXd evaluateInput(const InputSignal& signal, double t, int num_inputs) {
    return std::visit([&](auto&& sig) -> Eigen::VectorXd {
        using T = std::decay_t<decltype(sig)>;
        if constexpr (std::is_same_v<T, StepInput>) {
            if (t >= sig.start_time)
                return sig.amplitude;
            return Eigen::VectorXd::Zero(num_inputs);
        } else if constexpr (std::is_same_v<T, RampInput>) {
            if (t >= sig.start_time)
                return sig.slope * (t - sig.start_time);
            return Eigen::VectorXd::Zero(num_inputs);
        } else {  // CustomInput
            return sig(t);
        }
    }, signal);
}

ComparisonResult compareModels(
    const NonlinearFn& f_nl,
    const LinearSystem& lin,
    const Eigen::VectorXd& x0,
    const InputSignal& input,
    double dt,
    double duration)
{
    int num_steps = static_cast<int>(std::ceil(duration / dt));
    int n = static_cast<int>(x0.size());
    int m = lin.inputs();

    ComparisonResult result;
    result.time.reserve(num_steps + 1);
    result.nonlinear_states.reserve(num_steps + 1);
    result.linear_states.reserve(num_steps + 1);
    result.inputs.reserve(num_steps + 1);

    Eigen::VectorXd x_nl = x0;
    Eigen::VectorXd x_lin = x0;

    for (int i = 0; i <= num_steps; ++i) {
        double t = i * dt;
        Eigen::VectorXd u = evaluateInput(input, t, m);

        result.time.push_back(t);
        result.nonlinear_states.push_back(x_nl);
        result.linear_states.push_back(x_lin);
        result.inputs.push_back(u);

        if (i < num_steps) {
            // RK4 step for nonlinear model
            DerivativeFn f_nl_rk4 = [&](double /*t*/, const Eigen::VectorXd& x) {
                return f_nl(x, u);
            };
            x_nl = rk4_step(x_nl, t, dt, f_nl_rk4);

            // RK4 step for linear model: x_dot = A*x + B*u
            DerivativeFn f_lin_rk4 = [&](double /*t*/, const Eigen::VectorXd& x) {
                return Eigen::VectorXd(lin.A * x + lin.B * u);
            };
            x_lin = rk4_step(x_lin, t, dt, f_lin_rk4);
        }
    }

    return result;
}

}  // namespace caliburn
```

- [ ] **Step 3: Commit**

```bash
git add projects/ball-balancer/src/comparison_sim.h projects/ball-balancer/src/comparison_sim.cpp
git commit -m "feat: comparison simulation — dual-model RK4 runner with input signals"
```

---

### Task 6: Comparison Panel

**Files:**
- Create: `projects/ball-balancer/src/comparison_panel.h`
- Create: `projects/ball-balancer/src/comparison_panel.cpp`

- [ ] **Step 1: Write `comparison_panel.h`**

```cpp
#pragma once

#include "comparison_sim.h"
#include "ball_plant_linear.h"
#include "plot_panel.h"

namespace caliburn {

class ComparisonPanel {
public:
    ComparisonPanel();

    /// Draw the comparison panel as an ImGui window.
    /// Call once per frame from the main loop.
    void draw();

private:
    // Input configuration
    int input_type_ = 0;  // 0=Step, 1=Ramp
    float alpha_amplitude_ = 0.05f;
    float beta_amplitude_ = 0.0f;
    float duration_ = 5.0f;

    // Results
    bool has_result_ = false;
    ComparisonResult result_;
    ValidationResult validation_;

    // Plotting
    PlotState plot_state_;

    // Position traces: xN, yN, xL, yL
    TimeSeries ts_xN_{"xN"}, ts_yN_{"yN"}, ts_xL_{"xL"}, ts_yL_{"yL"};
    // Velocity traces: vxN, vyN, vxL, vyL
    TimeSeries ts_vxN_{"vN"}, ts_vyN_{"wN"}, ts_vxL_{"vL"}, ts_vyL_{"wL"};
    // Input traces
    TimeSeries ts_alpha_{"al"}, ts_beta_{"be"};
    // Error traces
    TimeSeries ts_ex_{"ex"}, ts_ey_{"ey"};

    std::vector<PlotConfig> plots_;

    // Divergence threshold
    float diverge_threshold_ = 0.01f;

    void runComparison();
    void populatePlots();
    void setupPlots();
    void drawValidationTable();
};

}  // namespace caliburn
```

- [ ] **Step 2: Write `comparison_panel.cpp`**

This is the largest file. It needs to:
1. Draw controls (input type, sliders, run button)
2. Call `compareModels()` on button press
3. Call `validate()` and display element-wise errors
4. Populate `TimeSeries` from `ComparisonResult`
5. Draw plots via `draw_time_series_panel()`
6. Add divergence shading

```cpp
#include "comparison_panel.h"

#include "imgui.h"
#include "implot.h"

#include <cmath>
#include <cstdio>

namespace caliburn {

ComparisonPanel::ComparisonPanel() {
    setupPlots();
}

void ComparisonPanel::setupPlots() {
    plots_.clear();
    plots_.push_back({"Position [m]", {&ts_xN_, &ts_yN_, &ts_xL_, &ts_yL_}, 0, true});
    plots_.push_back({"Velocity [m/s]", {&ts_vxN_, &ts_vyN_, &ts_vxL_, &ts_vyL_}, 1, true});
    plots_.push_back({"Input [rad]", {&ts_alpha_, &ts_beta_}, 2, true});
    plots_.push_back({"Error [m]", {&ts_ex_, &ts_ey_}, 3, false});  // collapsed by default
}

void ComparisonPanel::runComparison() {
    auto lin = ballBalancerLinearModel();
    auto f_nl = ballBalancerNonlinearFn();

    // Build input signal
    Eigen::VectorXd amp(2);
    amp << static_cast<double>(alpha_amplitude_),
           static_cast<double>(beta_amplitude_);

    InputSignal input;
    if (input_type_ == 0) {
        input = StepInput{amp, 0.5};  // step at t=0.5s
    } else {
        input = RampInput{amp, 0.5};
    }

    double dt = 0.001;
    result_ = compareModels(f_nl, lin, Eigen::VectorXd::Zero(4),
                            input, dt, static_cast<double>(duration_));
    has_result_ = true;

    // Run validation
    Eigen::VectorXd x0 = Eigen::VectorXd::Zero(4);
    Eigen::VectorXd u0 = Eigen::VectorXd::Zero(2);
    validation_ = validate(lin, f_nl, x0, u0);

    populatePlots();
}

void ComparisonPanel::populatePlots() {
    // Clear existing data
    plot_state_.clear();
    auto clear = [](TimeSeries& s) { s.data.clear(); };
    clear(ts_xN_); clear(ts_yN_); clear(ts_xL_); clear(ts_yL_);
    clear(ts_vxN_); clear(ts_vyN_); clear(ts_vxL_); clear(ts_vyL_);
    clear(ts_alpha_); clear(ts_beta_);
    clear(ts_ex_); clear(ts_ey_);

    // Downsample for plotting: take every Nth sample to keep buffer manageable
    int total = static_cast<int>(result_.time.size());
    int stride = std::max(1, total / 3600);

    // Temporarily unpause to allow push
    plot_state_.paused = false;

    for (int i = 0; i < total; i += stride) {
        float t = static_cast<float>(result_.time[i]);
        plot_state_.push_time(t);

        const auto& xnl = result_.nonlinear_states[i];
        const auto& xln = result_.linear_states[i];
        const auto& u = result_.inputs[i];

        push_series(ts_xN_, static_cast<float>(xnl(0)), plot_state_);
        push_series(ts_yN_, static_cast<float>(xnl(1)), plot_state_);
        push_series(ts_xL_, static_cast<float>(xln(0)), plot_state_);
        push_series(ts_yL_, static_cast<float>(xln(1)), plot_state_);

        push_series(ts_vxN_, static_cast<float>(xnl(2)), plot_state_);
        push_series(ts_vyN_, static_cast<float>(xnl(3)), plot_state_);
        push_series(ts_vxL_, static_cast<float>(xln(2)), plot_state_);
        push_series(ts_vyL_, static_cast<float>(xln(3)), plot_state_);

        push_series(ts_alpha_, static_cast<float>(u(0)), plot_state_);
        push_series(ts_beta_, static_cast<float>(u(1)), plot_state_);

        push_series(ts_ex_, static_cast<float>(std::abs(xnl(0) - xln(0))), plot_state_);
        push_series(ts_ey_, static_cast<float>(std::abs(xnl(1) - xln(1))), plot_state_);
    }

    // Pause after populating — this is static comparison data
    plot_state_.paused = true;
    plot_state_.pause_t_min = static_cast<float>(result_.time.front());
    plot_state_.pause_t_max = static_cast<float>(result_.time.back());
    plot_state_.time_window = plot_state_.pause_t_max - plot_state_.pause_t_min;
}

void ComparisonPanel::drawValidationTable() {
    if (!has_result_) return;

    ImGui::Separator();
    ImGui::Text("Linearization Validation");

    ImVec4 color = validation_.pass
        ? ImVec4(0.2f, 0.9f, 0.2f, 1.0f)
        : ImVec4(0.9f, 0.2f, 0.2f, 1.0f);
    ImGui::TextColored(color, validation_.pass ? "PASS" : "FAIL");
    ImGui::SameLine();
    ImGui::Text("max|A err|=%.2e  max|B err|=%.2e",
                validation_.max_A_error, validation_.max_B_error);

    if (ImGui::TreeNode("A error (element-wise)")) {
        for (int r = 0; r < validation_.A_error.rows(); ++r) {
            for (int c = 0; c < validation_.A_error.cols(); ++c) {
                if (c > 0) ImGui::SameLine();
                ImGui::Text("%.2e", validation_.A_error(r, c));
            }
        }
        ImGui::TreePop();
    }

    if (ImGui::TreeNode("B error (element-wise)")) {
        for (int r = 0; r < validation_.B_error.rows(); ++r) {
            for (int c = 0; c < validation_.B_error.cols(); ++c) {
                if (c > 0) ImGui::SameLine();
                ImGui::Text("%.2e", validation_.B_error(r, c));
            }
        }
        ImGui::TreePop();
    }
}

void ComparisonPanel::draw() {
    if (!ImGui::Begin("Model Comparison")) {
        ImGui::End();
        return;
    }

    // --- Controls ---
    ImGui::Text("Input Signal");
    ImGui::RadioButton("Step", &input_type_, 0);
    ImGui::SameLine();
    ImGui::RadioButton("Ramp", &input_type_, 1);

    ImGui::SliderFloat(u8"α amplitude [rad]", &alpha_amplitude_, 0.0f, 1.0f, "%.3f");
    ImGui::SliderFloat(u8"β amplitude [rad]", &beta_amplitude_, 0.0f, 1.0f, "%.3f");
    ImGui::SliderFloat("Duration [s]", &duration_, 1.0f, 30.0f, "%.1f");
    ImGui::SliderFloat("Diverge threshold", &diverge_threshold_, 0.001f, 0.1f, "%.3f");

    if (ImGui::Button("Run Comparison")) {
        runComparison();
    }
    ImGui::SameLine();
    if (ImGui::Button("Reset")) {
        has_result_ = false;
        plot_state_.clear();
        setupPlots();
    }

    // --- Validation results ---
    drawValidationTable();

    // --- Plots ---
    if (has_result_) {
        ImGui::Separator();
        draw_time_series_panel("##comparison_plots", plot_state_, plots_);
    }

    ImGui::End();
}

}  // namespace caliburn
```

- [ ] **Step 3: Commit**

```bash
git add projects/ball-balancer/src/comparison_panel.h projects/ball-balancer/src/comparison_panel.cpp
git commit -m "feat: comparison panel — linear vs nonlinear visualization"
```

---

### Task 7: Wire Into Ball-Balancer Build and Visualizer

**Files:**
- Modify: `projects/ball-balancer/CMakeLists.txt`
- Modify: `projects/ball-balancer/src/visualizer.cpp`

- [ ] **Step 1: Update `CMakeLists.txt`**

Add a library for the ball dynamics (linear + nonlinear + comparison) and update the visualizer target. Add these lines after the `kinematics` library block:

```cmake
# --- Ball dynamics library (linear + nonlinear + comparison) ---
add_library(ball_dynamics STATIC
    src/ball_plant_linear.cpp
    src/comparison_sim.cpp
    ../../reference/simulation/rolling_dynamics.cpp
    ../../reference/integrators/rk4.cpp
    ../../reference/models/linearizer.cpp
)
target_include_directories(ball_dynamics PUBLIC
    src
    ../../reference/models
    ../../reference/simulation
    ../../reference/integrators
)
target_link_libraries(ball_dynamics PUBLIC Eigen3::Eigen)
```

Update the visualizer target to include the comparison panel and link ball_dynamics:

```cmake
add_executable(visualizer
    src/visualizer.cpp
    src/renderer.cpp
    src/plot_panel.cpp
    src/comparison_panel.cpp
)
target_link_libraries(visualizer PRIVATE
    kinematics
    ball_dynamics
    imgui
    implot
    glad
    glfw
    OpenGL::GL
)
```

- [ ] **Step 2: Add comparison panel to `visualizer.cpp`**

Add the include at the top:

```cpp
#include "comparison_panel.h"
```

Add a `ComparisonPanel` instance to `AppState`:

```cpp
// In AppState struct, add:
caliburn::ComparisonPanel comparison_panel;
```

Add the draw call in the main loop, after the existing plots panel draw:

```cpp
// In the main loop, after draw_time_series_panel(...):
state.comparison_panel.draw();
```

- [ ] **Step 3: Build the visualizer**

Run: `cd /home/nds/Desktop/Merlin/04-projects/Caliburn/projects/ball-balancer && cmake -B build && cmake --build build --target visualizer 2>&1 | tail -10`
Expected: Build succeeds

- [ ] **Step 4: Build and run reference tests**

Run: `cd /home/nds/Desktop/Merlin/04-projects/Caliburn/reference && cmake -B build && cmake --build build --target test_linearizer && ./build/test_linearizer`
Expected: All 5 linearizer tests pass

- [ ] **Step 5: Commit**

```bash
git add projects/ball-balancer/CMakeLists.txt projects/ball-balancer/src/visualizer.cpp
git commit -m "feat: wire comparison panel into ball-balancer visualizer"
```

---

### Task 8: Knowledge File

**Files:**
- Create: `knowledge/control-theory/linearization.md`
- Modify: `knowledge/control-theory/index.md`

- [ ] **Step 1: Write `linearization.md`**

```markdown
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
  - second-order-systems.md
---

# Linearization

Linearization approximates a nonlinear system with a linear one near an operating point. This is the bridge between first-principles modelling (which produces nonlinear ODEs) and linear control design tools (Bode, root locus, LQR, pole placement) which all require state-space or transfer function models.

## Jacobian Linearization

Given a nonlinear system:

```
x_dot = f(x, u)
y = h(x, u)
```

At an equilibrium point (x0, u0) where f(x0, u0) = 0, the linearized system is:

```
delta_x_dot = A * delta_x + B * delta_u
delta_y = C * delta_x + D * delta_u
```

Where delta_x = x - x0, delta_u = u - u0, and the Jacobian matrices are:

```
A = df/dx |_(x0, u0)     (n x n)
B = df/du |_(x0, u0)     (n x m)
C = dh/dx |_(x0, u0)     (p x n)
D = dh/du |_(x0, u0)     (p x m)
```

This is a first-order Taylor expansion — higher-order terms are dropped.

## Validity Region

The linear model is accurate only "near" the operating point. How near depends on the curvature of f:

- **Mildly nonlinear** (sin at small angles): valid over a wide range. sin(0.1) ≈ 0.1 with 0.17% error, sin(0.3) ≈ 0.3 with 1.5% error.
- **Strongly nonlinear** (saturation, dead zones, friction): valid over a very narrow range. Coulomb friction switches sign discontinuously — linearization fails at v=0.
- **Rule of thumb:** if the nonlinear and linear time responses agree within 5% for the expected input range, the linearization is adequate for control design.

## Numerical Linearization

When the analytical Jacobian is tedious (many states, complex coupling), compute A and B numerically via central finite differences:

```
A(:, i) = (f(x0 + eps*e_i, u0) - f(x0 - eps*e_i, u0)) / (2 * eps)
B(:, j) = (f(x0, u0 + eps*e_j) - f(x0, u0 - eps*e_j)) / (2 * eps)
```

Where e_i is the i-th unit vector and eps is a small perturbation (typically 1e-6). Central differences give O(eps^2) accuracy, much better than forward differences.

**Best practice:** Derive A, B analytically first, then validate against the numerical result. This catches derivation errors while building physical understanding.

## When Linearization Fails

- **Discontinuities:** Coulomb friction, backlash, relay controllers. The Jacobian doesn't exist at the switching surface.
- **Limit cycles:** Oscillations that linearization cannot predict (requires describing function or simulation).
- **Large-signal behaviour:** Saturation, actuator limits, constraint activation. The linear model doesn't know about these.
- **Multiple equilibria:** Linearization is valid around one equilibrium. If the system has several (e.g., inverted vs hanging pendulum), you need separate linear models for each.

For these cases, use nonlinear control methods: sliding mode (robust to model uncertainty), gain scheduling (multiple linear models), or MPC (handles constraints directly).

## Worked Example: Ball-on-Plate

See `projects/ball-balancer/src/ball_plant_linear.h` for the implementation and `reference/models/linearizer.h` for the validation tool.

**Nonlinear dynamics:**
```
ax = (5/7) * g * sin(beta)    — Y-axis tilt drives x-acceleration
ay = (5/7) * g * sin(alpha)   — X-axis tilt drives y-acceleration
```

**Linearization at (x=0, v=0, alpha=0, beta=0):**
- sin(alpha) ≈ alpha, sin(beta) ≈ beta
- Rolling friction drops out at zero velocity
- System decouples into two identical double integrators

**Result:** A is 4x4 with ones on the (0,2) and (1,3) entries (position-velocity coupling). B has (5/7)*g on the (2,1) and (3,0) entries (input-acceleration coupling). The system is controllable and observable.

## Connection to Frequency Domain

Once you have (A, B, C, D), you can compute the transfer function matrix:

```
G(s) = C * (sI - A)^(-1) * B + D
```

This enables Bode plots, Nyquist diagrams, and root locus analysis — all starting from the linearized state-space model. See [Frequency Response](frequency-response.md) for details.

## Connection to Gain Scheduling

When one linear model isn't enough, linearize at multiple operating points and schedule between them. See [Gain Scheduling](controllers/gain-scheduling.md) for the approach.
```

- [ ] **Step 2: Add to `knowledge/control-theory/index.md`**

Add a new row to the Topics table:

```
| [Linearization](linearization.md) | Jacobian linearization, small-signal validity, numerical vs analytical, worked ball-on-plate example |
```

- [ ] **Step 3: Commit**

```bash
git add knowledge/control-theory/linearization.md knowledge/control-theory/index.md
git commit -m "docs: add linearization knowledge file with ball-on-plate worked example"
```

---

### Task 9: Update Ball-Balancer AGENTS.md

**Files:**
- Modify: `projects/ball-balancer/AGENTS.md`

- [ ] **Step 1: Update the current focus and deferred items**

Change the current focus line from:

```
**Current focus:** Table kinematics only. Ball dynamics, control, and estimation are deferred.
```

To:

```
**Current focus:** Table kinematics + ball dynamics linearization. Control and estimation are deferred.
```

Add to the Caliburn Resources section under Knowledge:

```
- `../../knowledge/control-theory/linearization.md` — linearization theory
- `../../reference/models/linearizer.h` — numerical linearizer
- `../../reference/models/linear_system.h` — LinearSystem struct
```

- [ ] **Step 2: Commit**

```bash
git add projects/ball-balancer/AGENTS.md
git commit -m "docs: update ball-balancer status — linearization complete"
```

---

### Task 10: End-to-End Verification

- [ ] **Step 1: Build and run all reference tests**

Run: `cd /home/nds/Desktop/Merlin/04-projects/Caliburn/reference && cmake -B build && cmake --build build && ctest --test-dir build --output-on-failure 2>&1 | tail -20`
Expected: `test_linearizer` passes (pre-existing failures in rk4/bicycle/tyre are unrelated)

- [ ] **Step 2: Build ball-balancer visualizer**

Run: `cd /home/nds/Desktop/Merlin/04-projects/Caliburn/projects/ball-balancer && cmake -B build && cmake --build build 2>&1 | tail -5`
Expected: Both `analysis` and `visualizer` targets build cleanly

- [ ] **Step 3: Smoke test the visualizer**

Run: `cd /home/nds/Desktop/Merlin/04-projects/Caliburn/projects/ball-balancer && timeout 3 ./build/visualizer 2>&1; echo "exit: $?"`
Expected: Exits cleanly (timeout or window close), no crashes or assertion failures

- [ ] **Step 4: Final commit (if any fixups needed)**

```bash
git add -A && git commit -m "fix: address build issues from integration"
```
