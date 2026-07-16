# Linear System Analyzer

**Date:** 2026-07-16
**Status:** Draft
**Project:** Caliburn

## Purpose

An interactive desktop tool for analyzing linear state-space models. The user loads or constructs a plant G(s) as state-space (A, B, C, D), optionally defines a controller, and the tool visualizes pole-zero maps, Bode plots, Nyquist plots, step/impulse/ramp responses, and controllability/observability checks — for the plant alone, the controller alone, the open-loop combination, and the closed-loop feedback system.

This is the next project after the linearization tooling. It consumes `LinearSystem` structs produced by the linearizer or hand-derived models.

## Architecture

**Approach B: Analysis Library + Thin Panels.** A headless analysis library contains all computation (pure functions, no GUI dependency, unit-testable). Thin ImGui panels render precomputed results. The visualizer main loop coordinates model changes, recomputation, and panel rendering.

The project lives at `projects/linear-analyzer/` as a standalone project with its own CMakeLists.txt and git repo, following the same pattern as `projects/ball-balancer/` and `projects/bode-explorer/`.

## Project Structure

```
projects/linear-analyzer/
├── CMakeLists.txt
├── vendor/fonts/NotoSans-Regular.ttf
├── src/
│   ├── analysis/                       # Headless analysis library (no ImGui)
│   │   ├── frequency_response.h/.cpp   # G(jω) = C(jωI-A)⁻¹B+D
│   │   ├── pole_zero.h/.cpp            # Eigenvalues, transmission zeros, root locus
│   │   ├── system_properties.h/.cpp    # Controllability/observability rank checks
│   │   ├── time_response.h/.cpp        # Step/impulse/ramp via RK4
│   │   ├── model_library.h/.cpp        # Built-in presets + matrix parsing
│   │   └── system_connect.h/.cpp       # Series, feedback, state-feedback connections
│   ├── panels/                         # Thin ImGui panels (render only)
│   │   ├── bode_panel.h/.cpp
│   │   ├── nyquist_panel.h/.cpp
│   │   ├── pole_zero_panel.h/.cpp
│   │   ├── time_response_panel.h/.cpp
│   │   ├── properties_panel.h/.cpp
│   │   └── model_panel.h/.cpp
│   └── visualizer.cpp                  # Main loop, docking, glue
├── tests/
│   ├── test_frequency_response.cpp
│   ├── test_pole_zero.cpp
│   ├── test_system_properties.cpp
│   ├── test_time_response.cpp
│   └── test_system_connect.cpp
└── .gitignore
```

## Deliverable 1: LinearSystem (existing)

Reuses `reference/models/linear_system.h` — the existing `LinearSystem` struct with A, B, C, D matrices and `states()`, `inputs()`, `outputs()` accessors. Copied into the project (header-only, no cross-directory CMake dependency).

## Deliverable 2: Analysis Library

Six modules, all pure functions on `LinearSystem`. No global state, no ImGui dependency.

### 2a: Frequency Response

**Path:** `src/analysis/frequency_response.h/.cpp`

```cpp
namespace caliburn {

struct BodePoint {
    double freq_hz;
    double magnitude_db;
    double phase_deg;
};

struct FrequencyResponse {
    std::vector<BodePoint> points;
    double gain_margin_db;      // dB above 0 at phase crossover
    double phase_margin_deg;    // degrees above -180 at gain crossover
    double gain_crossover_hz;   // where |G| = 0 dB
    double phase_crossover_hz;  // where ∠G = -180°
};

// Evaluate single SISO channel (input j → output i) over a log-spaced frequency grid.
// Computes G_ij(jω) = C_i (jωI - A)⁻¹ B_j + D_ij using ColPivHouseholderQR.
FrequencyResponse computeBode(
    const LinearSystem& sys, int output_i, int input_j,
    double freq_min_hz, double freq_max_hz, int num_points);

// Raw complex evaluation at a single complex frequency s.
// Used by Nyquist (s = jω along the Nyquist contour) and internal helpers.
std::complex<double> evalTransferFunction(
    const LinearSystem& sys, int output_i, int input_j,
    std::complex<double> s);

}  // namespace caliburn
```

Phase is computed with continuous unwrapping to avoid ±180° jumps.

Gain margin: magnitude (in dB) at the phase crossover frequency (where phase = -180°).
Phase margin: phase (in deg) offset from -180° at the gain crossover frequency (where magnitude = 0 dB).

### 2b: Pole-Zero Analysis

**Path:** `src/analysis/pole_zero.h/.cpp`

```cpp
namespace caliburn {

struct PoleZeroResult {
    std::vector<std::complex<double>> poles;   // eigenvalues of A
    std::vector<std::complex<double>> zeros;   // transmission zeros for selected channel
    bool is_stable;                             // all poles have Re < 0
};

struct RootLocusPoint {
    double gain;
    std::vector<std::complex<double>> poles;
};

// Compute poles (eigenvalues of A) and transmission zeros for channel (i, j).
// Transmission zeros: values of s where the Rosenbrock system matrix loses rank.
PoleZeroResult computePoleZero(
    const LinearSystem& sys, int output_i, int input_j);

// Unity feedback root locus: closed-loop poles of 1 + K·G_ij(s) = 0 as K varies.
// Implemented by forming the closed-loop A matrix for each K and computing eigenvalues.
std::vector<RootLocusPoint> computeRootLocus(
    const LinearSystem& sys, int output_i, int input_j,
    double k_min, double k_max, int num_points);

// State feedback root locus: eigenvalues of (A - α·B·K) as α varies.
// K is the gain matrix (m×n for m inputs, n states).
// The scalar multiplier α scales K.
std::vector<RootLocusPoint> computeStateFeedbackLocus(
    const LinearSystem& sys,
    const Eigen::MatrixXd& K,
    double alpha_min, double alpha_max, int num_points);

}  // namespace caliburn
```

For the unity feedback root locus with state-space: given G_ij(s) from the plant and scalar gain K, the closed-loop transfer function has poles that are the eigenvalues of A_cl = A - K·B_j·C_i (output feedback on the selected channel). Sweep K from k_min to k_max.

For state feedback: A_cl = A - α·B·K where K is the full gain matrix direction. Sweep α from alpha_min to alpha_max.

### 2c: System Properties

**Path:** `src/analysis/system_properties.h/.cpp`

```cpp
namespace caliburn {

struct PropertyResult {
    Eigen::MatrixXd matrix;   // the controllability or observability matrix
    int rank;
    int required_rank;        // = n (number of states)
    bool pass;                // rank == required_rank
};

PropertyResult checkControllability(const LinearSystem& sys);
PropertyResult checkObservability(const LinearSystem& sys);

}  // namespace caliburn
```

Controllability matrix: [B, AB, A²B, ..., A^(n-1)B]. Rank computed via ColPivHouseholderQR.
Observability matrix: [C; CA; CA²; ...; CA^(n-1)]. Same rank check.

### 2d: Time Response

**Path:** `src/analysis/time_response.h/.cpp`

```cpp
namespace caliburn {

struct TimePoint {
    double time;
    Eigen::VectorXd state;   // x(t)
    Eigen::VectorXd output;  // y(t) = Cx + Du
    Eigen::VectorXd input;   // u(t)
};

struct TimeResponse {
    std::vector<TimePoint> points;
};

// Step response: u_j(t) = amplitude for t >= 0.
TimeResponse computeStepResponse(
    const LinearSystem& sys, int input_j,
    double amplitude, double duration, double dt);

// Impulse response: u_j(t) = (amplitude/dt) for one time step, then 0.
TimeResponse computeImpulseResponse(
    const LinearSystem& sys, int input_j,
    double amplitude, double duration, double dt);

// Ramp response: u_j(t) = slope * t for t >= 0.
TimeResponse computeRampResponse(
    const LinearSystem& sys, int input_j,
    double slope, double duration, double dt);

}  // namespace caliburn
```

All three simulate ẋ = Ax + Bu, y = Cx + Du using the existing `rk4_step()` from `reference/integrators/rk4.h` (copied into the project).

### 2e: Model Library

**Path:** `src/analysis/model_library.h/.cpp`

```cpp
namespace caliburn {

struct ModelEntry {
    std::string name;
    std::string description;
    LinearSystem system;
};

// Built-in preset models.
std::vector<ModelEntry> getBuiltinModels();

// Parse a matrix from a MATLAB-style string: "0 1; -2 -3"
// Returns nullopt on parse failure.
std::optional<Eigen::MatrixXd> parseMatrix(const std::string& text);

}  // namespace caliburn
```

Built-in models:
- Ball-Balancer (4 states, 2 inputs, 2 outputs) — from `ball_plant_linear`
- Inverted Pendulum on Cart (4 states, 1 input, 2 outputs)
- Quarter-Car Suspension (4 states, 1 input, 2 outputs)
- Double Mass-Spring-Damper (4 states, 1 input, 2 outputs)
- Simple Second-Order (2 states, 1 input, 1 output) — mass-spring-damper with tunable ωn, ζ

## Deliverable 3: Multi-System Configuration

The core design enhancement: the analyzer doesn't just show one system — it shows up to four system configurations simultaneously, each in a consistent color:

| System | Color | What it is |
|---|---|---|
| Plant G(s) | Blue (#38bdf8) | The loaded/edited LinearSystem |
| Controller C(s) | Purple (#a855f7) | Optional second LinearSystem or gain matrix K |
| Open-Loop L(s) | Green (#34d399) | Series combination: L = C · G (or G · C depending on convention) |
| Closed-Loop T(s) | Orange (#f97316) | Unity negative feedback: T = L(I + L)⁻¹ |

### Controller input modes

The controller can be specified in two ways:

1. **State-space (A_c, B_c, C_c, D_c):** A dynamic controller. The combined open-loop system is formed by series connection of the controller and plant state-space models.

2. **Gain matrix K:** Static state feedback u = -Kx. The closed-loop system is A_cl = A - BK, B_cl = B (or a reference input matrix), C_cl = C, D_cl = D.

When no controller is defined, only the Plant traces appear. When a controller is added, all four systems become available as toggleable traces on every analysis panel.

### State-space series connection

Signal flow: reference r → C(s) → u → G(s) → y. The open-loop transfer function is L(s) = G(s)·C(s).

Given plant G with (A_g, B_g, C_g, D_g) and controller C with (A_c, B_c, C_c, D_c), the series connection L = G·C is:

```
A_L = | A_c       0         |    B_L = | B_c     |
      | B_g·C_c   A_g       |          | B_g·D_c |

C_L = | D_g·C_c   C_g |    D_L = | D_g·D_c |
```

For unity negative feedback: T(s) = L(s)·(I + L(s))⁻¹. In state-space, the closed-loop A matrix is A_L - B_L·(I + D_L)⁻¹·C_L, with corresponding B, C, D adjustments.

For static gain K (state feedback u = -Kx + r):
- Open-loop L: not directly meaningful (K is static, no dynamics)
- Closed-loop: A_cl = A - BK, B_cl = B, C_cl = C, D_cl = D

### Implementation

```cpp
namespace caliburn {

// Series connection: result = sys2 * sys1 (sys1 output feeds sys2 input)
LinearSystem seriesConnect(const LinearSystem& sys1, const LinearSystem& sys2);

// Unity negative feedback: T = G / (I + G)
// For SISO: T(s) = G(s) / (1 + G(s))
// For MIMO: T(s) = G(s) · (I + G(s))⁻¹
LinearSystem feedbackConnect(const LinearSystem& open_loop);

// State feedback: A_cl = A - B*K
LinearSystem stateFeedbackClose(const LinearSystem& plant, const Eigen::MatrixXd& K);

}  // namespace caliburn
```

These live in a new file `src/analysis/system_connect.h/.cpp`.

## Deliverable 4: Visualizer Panels

All panels follow `scaffold-sim-viewer` invariants (docking, font, reset, plot rules). Since this is a pure 2D analysis tool, there is no 3D viewport.

### 4a: Model Panel

Always visible. Contains:

- **Plant section:** Preset dropdown + editable A, B, C, D text fields (MATLAB-style: `"0 1; -2 -3"`). Pre-populated from preset, user can edit. "Apply" button triggers recomputation.
- **Controller section:** Type selector (None / State-Space / Gain Matrix K). When State-Space: four text fields for A_c, B_c, C_c, D_c. When Gain Matrix: one text field for K. "Apply Controller" button.
- **System info:** Dimensions (n, m, p), stability status for plant and closed-loop (green STABLE / red UNSTABLE / yellow MARGINAL for poles on jω axis).
- **Channel selector:** Output i and Input j dropdowns. "Show All Channels" checkbox (enabled only when p, m ≤ 3). All frequency-domain and time-domain panels read the selected channel from here.
- **Reset All** button (red) — restores all state to defaults.

### 4b: Pole-Zero / Root Locus Panel

ImPlot scatter/line plot in the complex plane (σ on x-axis, jω on y-axis).

**Default mode (Pole-Zero):**
- Poles as × markers, zeros as ○ markers
- Color-coded by system: plant poles blue, closed-loop poles orange
- Stable region (Re < 0) lightly shaded green
- Unit circle drawn as dashed reference (relevant for discrete systems, helpful visual anchor)

**Unity feedback locus mode:**
- Gain slider K (log scale, configurable range)
- Locus paths drawn as continuous lines connecting swept poles
- Current K's poles highlighted with larger markers
- As K = 0: shows plant poles. As K increases: poles move along locus paths.

**State feedback locus mode:**
- Text field for gain vector k direction
- Scalar multiplier α slider
- Same locus path rendering

Three mode radio buttons: Pole-Zero / Unity FB / State FB.

Trace toggles: Plant, Controller (if defined), Closed-Loop.

### 4c: Bode Panel

Two stacked ImPlot plots sharing a log-frequency x-axis:
- Top: Magnitude [dB]
- Bottom: Phase [deg]

Trace toggles for all four systems (Plant, Controller, Open-Loop, Closed-Loop). Each system's Bode is computed independently and overlaid with its color.

Gain and phase margin annotations:
- Vertical dashed line at gain crossover frequency (where |L| = 0 dB)
- Vertical dashed line at phase crossover frequency (where ∠L = -180°)
- Margin values displayed as text annotations and in a summary bar below the plot
- Margins are computed for the **open-loop L(s)** specifically (this is the standard convention)

Frequency range controls: min/max Hz text fields, number of points.

In "Show All Channels" mode: p×m grid of Bode plot pairs (max 3×3 = 9 pairs).

### 4d: Nyquist Panel

ImPlot parametric plot: Re{L(jω)} vs Im{L(jω)}.

- Critical point (-1, 0) marked with a red circle
- Frequency direction indicated with arrow markers at regular intervals
- Tooltip shows frequency at hover point
- Both positive and negative frequency contours drawn (symmetric about real axis)

Trace toggles: typically Plant and Open-Loop (Nyquist is most meaningful for the open-loop).

### 4e: Time Response Panel

Input type selector: Step / Impulse / Ramp (radio buttons).

Controls:
- Amplitude slider (for step/impulse) or slope slider (for ramp)
- Duration slider (1–30 s)
- Input channel selector (which input to excite)

Plots follow "one plot per state type" grouping:
- Output [units] — all output channels overlaid
- Input [units] — the applied input signal

Trace toggles: Plant response (blue) and Closed-Loop response (orange) overlaid on the same plots. This directly shows how the controller changes the system's behavior.

Time-series plot invariants apply: synchronized cursor, click-to-mark, Y-axis auto-fit, legend outside right with ≤2-char labels.

### 4f: Properties Panel

Two sections: Controllability and Observability.

Each shows:
- Pass/fail badge (green/red) with rank vs required rank
- Collapsible tree to expand the full controllability/observability matrix

Shown for the plant system. When a controller is defined, also show closed-loop properties.

### Panel Toggle Bar

A row of toggle buttons at the top of the right column, one per panel: Pole-Zero, Bode, Nyquist, Time Response. Clicking hides/shows the panel. The Properties panel is always shown in the left column (it's compact).

## Deliverable 5: Visualizer Main Loop

**Path:** `src/visualizer.cpp`

Follows the ball-balancer pattern:

1. GLFW window init, ImGui/ImPlot context, font loading (NotoSans with Greek/math ranges)
2. `ImGuiConfigFlags_DockingEnable`, `io.IniFilename = "imgui.ini"`
3. Full-viewport dockspace with `PassthruCentralNode`
4. `AppState` struct holding: current plant LinearSystem, controller config, channel selection, all precomputed analysis results, panel visibility flags, UI state
5. Recompute flag: when model or channel changes, set `needs_recompute = true`. Before rendering, recompute all visible analysis results.
6. Panel draw calls (each panel receives its precomputed results by const reference)
7. Reset All: overwrite AppState to initial values

No 3D viewport — the central dockspace node is empty background.

## Deliverable 6: Tests

**Path:** `tests/`

| Test file | What it validates |
|---|---|
| `test_frequency_response.cpp` | Bode of known 1st/2nd order systems matches analytical; margins correct for textbook examples |
| `test_pole_zero.cpp` | Eigenvalues match known systems; transmission zeros correct; root locus endpoints match open/closed-loop poles |
| `test_system_properties.cpp` | Controllable/observable systems pass; uncontrollable system (constructed) fails; rank values correct |
| `test_time_response.cpp` | Step response of 1st-order system matches analytical (1-e^(-t/τ)); impulse of 2nd-order matches; ramp response steady-state error matches system type |
| `test_system_connect.cpp` | Series connection of two known systems matches expected combined TF; feedback connection matches expected closed-loop poles; state feedback close matches A-BK eigenvalues |

## Deliverable 7: CMakeLists.txt

Dependencies via FetchContent:
- GLFW 3.4
- ImGui (docking branch)
- ImPlot (master)
- Eigen 3.4.0
- GLAD vendored from `../ball-balancer/vendor/glad/`

Build targets:
- `analysis_lib` — static library from `src/analysis/*.cpp`
- `visualizer` — executable linking `analysis_lib`, imgui, implot, glad, glfw, OpenGL
- `test_*` — test executables linking `analysis_lib` and Eigen only (no GUI)

## Deliverable 8: Knowledge Update

Add a note to `knowledge/control-theory/frequency-response.md` pointing to the linear-analyzer project as the interactive tool for exploring these concepts.

## Out of Scope

- 3D visualization (this is a 2D analysis tool)
- Discrete-time systems (z-domain) — future extension
- Automated controller synthesis (LQR/pole placement from within the tool) — the tool shows analysis, not design
- Transfer function input mode (factored poles/zeros) — that's what bode-explorer does
- File loading (.json/.yaml) for models — manual entry + presets is sufficient for v1

## Build Order

1. Copy `linear_system.h` and `rk4.h` into project, set up CMakeLists.txt with all dependencies
2. `model_library.h/.cpp` — presets and matrix parsing (+ tests for parsing)
3. `system_properties.h/.cpp` + tests — controllability/observability
4. `frequency_response.h/.cpp` + tests — Bode computation and margins
5. `pole_zero.h/.cpp` + tests — eigenvalues, zeros, root locus
6. `time_response.h/.cpp` + tests — step/impulse/ramp
7. `system_connect.h/.cpp` + tests — series, feedback, state-feedback connections
8. `model_panel.h/.cpp` — model configuration UI
9. `properties_panel.h/.cpp` — controllability/observability display
10. `pole_zero_panel.h/.cpp` — pole-zero map and root locus rendering
11. `bode_panel.h/.cpp` — Bode plot rendering with margins
12. `nyquist_panel.h/.cpp` — Nyquist plot rendering
13. `time_response_panel.h/.cpp` — time response plots
14. `visualizer.cpp` — main loop, docking, wire everything together
15. End-to-end verification
