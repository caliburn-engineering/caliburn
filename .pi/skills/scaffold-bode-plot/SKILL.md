---
name: scaffold-bode-plot
description: Scaffold an interactive Bode plot viewer for exploring transfer function frequency response. Use when the user wants to visualize, plot, or analyze Bode magnitude/phase for a control system.
---

# Scaffold Bode Plot

Generate an interactive ImGui+ImPlot application that lets the user build transfer functions from individual poles and zeros and see the Bode magnitude and phase response update in real time.

## Step 1 — Read the knowledge and reference

Read `knowledge/control-theory/frequency-response.md` for the theory: factored transfer function form, standard element types, evaluation at s=jw, phase unwrapping, and plotting conventions.

Read `projects/bode-explorer/transfer_function.h` for the reference implementation of the pure logic module.

Completion: you understand the six element types (real pole, real zero, complex pole pair, complex zero pair, integrator, differentiator) and how to evaluate H(jw).

## Step 2 — Transfer function logic module

Create a header-only logic module (no rendering dependencies). It must contain:

### Element types

| Type | H(s) | Parameters |
|---|---|---|
| Real Pole | 1 / (1 + s/w_n) | w_n (corner frequency) |
| Real Zero | (1 + s/w_n) | w_n (corner frequency) |
| Complex Pole Pair | 1 / (1 + 2·zeta·s/w_n + s²/w_n²) | w_n, zeta |
| Complex Zero Pair | (1 + 2·zeta·s/w_n + s²/w_n²) | w_n, zeta |
| Integrator | 1/s | none |
| Differentiator | s | none |

### TransferFunction struct

- A gain K (float) and a vector of elements
- `evaluate(float omega)` → `std::complex<float>` — evaluates H(jw) by multiplying K with all element contributions
- `magnitude_db(float omega)` → float — 20·log10(|H(jw)|), clamped to avoid -inf
- `phase_deg(float omega)` → float — arg(H(jw)) in degrees

### BodeData struct

- Logarithmically spaced frequency grid (default: 0.01 to 10000 rad/s, 500 points)
- Vectors for omega, magnitude (dB), and phase (degrees)
- `compute(const TransferFunction&)` — fills all three vectors
- Phase unwrapping post-pass to produce continuous phase curves

### Preset library

Include at minimum:
- Single real pole (1st-order LP)
- 1st-order high-pass: gain = 1/w_c, one differentiator + one real pole at w_c
- Integrator
- 2nd-order underdamped (zeta = 0.2), critically damped (zeta = 1.0), overdamped (zeta = 2.0)
- PID-like: integrator + two real zeros
- Notch filter: complex zero pair + complex pole pair at same w_n, different zeta

**Common mistake — 1st-order high-pass:** The correct factored form is `H(s) = (1/w_c) · s · 1/(1 + s/w_c)`. The gain factor `1/w_c` is essential. Without it the system doesn't settle to unity gain at high frequencies. Do NOT use a RealZero + RealPole at the same frequency (they cancel).

### Margins struct

- `omega_gc`, `phase_at_gc`, `phase_margin` — gain crossover data (PM = 180 + phase_at_gc)
- `omega_pc`, `mag_at_pc`, `gain_margin` — phase crossover data (GM = -mag_at_pc)
- `has_gain_crossover`, `has_phase_crossover` — booleans (not all systems have both crossovers)

### find_margins(BodeData) function

Walk the BodeData arrays and find first downward crossings:
- Gain crossover: where mag_db crosses 0 dB — interpolate in log-frequency to find exact omega, then read phase
- Phase crossover: where phase_deg crosses -180° — interpolate to find exact omega, then read magnitude

### ClosedLoopData struct

Given a plant G(s) and controller C(s):
- Compute L(jw) = G(jw) * C(jw) for each frequency point
- Compute T(jw) = L(jw) / (1 + L(jw)) for each frequency point
- Store as two BodeData structs (open_loop, closed_loop) via `compute_from_complex()`
- Compute margins on the open-loop data

Completion: the logic module compiles standalone with no rendering dependencies.

## Step 3 — ImGui/ImPlot application

Create the main application with dockable panels:

### Controls panel

- **Mode toggle**: "Lock Plant & Add Controller" button. When clicked, the plant TF is frozen and a second builder panel ("Controller C(s)") appears. An "Unlock Plant" button reverses this.
- **Plot Settings** (collapsible) — frequency range sliders (log), point count slider

### Transfer Function Builder panel(s)

Reusable panel drawn once for the plant G(s), and optionally a second time for the controller C(s) when the plant is locked. When locked, the plant panel shows "LOCKED" and is read-only.

Each builder panel contains:
- **Presets section** — buttons for each preset, loads the entire transfer function
- **Gain slider** — logarithmic (10^x), range 10^-2 to 10^3
- **Element list** — each element shows:
  - Color-coded label: blue for poles, green for zeros
  - Type selector (combo box with all 6 types)
  - w_n slider (logarithmic, range 10^-2 to 10^5 rad/s) — hidden for integrator/differentiator
  - zeta slider (0.01 to 3.0) — shown only for complex pairs, with text label for system character (underdamped / critically damped / overdamped)
  - Remove button (X)
- **Add buttons** — one per element type
- **Clear All** button

### Bode Plot panel

- **Transfer function expressions** displayed as text above the plots
- **Margin readout** — colored text showing PM and GM values with crossover frequencies. Green if positive (stable), red if negative (unstable).

**Single-TF mode** (plant only):
- Magnitude plot of |G(jw)| and Phase plot of G(jw)
- Margins computed and displayed on G(s)

**Plant + Controller mode** (plant locked):
- **Magnitude — Open Loop L(s)**: shows |L(jw)| = |G(jw)*C(jw)| as the primary curve, with the plant |G(jw)| dimmed behind it
- **Phase — Open Loop L(s)**: same layout, open-loop phase primary, plant phase dimmed
- **Magnitude — Closed Loop T(s)**: shows |T(jw)| = |L/(1+L)| in green, with a -3 dB bandwidth reference line
- Margins are computed on the open-loop L(s) and visualized there

**Margin visualization on plots:**
- **Phase margin** (yellow): vertical line at gain crossover on both plots. On the phase plot, a thick yellow segment from -180° to the actual phase shows the PM visually. Scatter dot at the crossing point.
- **Gain margin** (red): vertical line at phase crossover on both plots. On the magnitude plot, a thick red segment from the actual magnitude to 0 dB shows the GM visually. Scatter dot at the crossing point.

Use `ImPlotSpec` for line styling (the current ImPlot API does not have `SetNextLineStyle`):
```cpp
ImPlot::PlotLine("label", xs, ys, count, {
    ImPlotProp_LineColor, ImVec4(r, g, b, a),
    ImPlotProp_LineWeight, weight
});
```

For scatter markers:
```cpp
ImPlot::PlotScatter("label", &x, &y, 1, {
    ImPlotProp_MarkerSize, 5.0f,
    ImPlotProp_MarkerFillColor, ImVec4(r, g, b, a),
    ImPlotProp_Flags, (int)ImPlotItemFlags_NoLegend
});
```

### Quick Reference panel

Display Bode plot rules of thumb, margin definitions, and closed-loop formulas as static text.

## Step 4 — CMake

Follow the same FetchContent pattern as the ball-balancer project:

| Dependency | Source | Tag |
|---|---|---|
| GLFW | `github.com/glfw/glfw` | `3.4` |
| ImGui | `github.com/ocornut/imgui` | `docking` |
| ImPlot | `github.com/epezent/implot` | `master` |
| GLAD | Vendored from `projects/ball-balancer/vendor/glad/` | GL 3.3 Core |

Use `gladLoadGLLoader((GLADloadproc)glfwGetProcAddress)` for GL loading.

Enable `ImGuiConfigFlags_DockingEnable` and use `ImGui::DockSpaceOverViewport()` for the full-window dockspace.

Completion: `cmake -B build && cmake --build build` succeeds.

## Step 5 — Verify

1. Build succeeds with no warnings in project code
2. App launches and displays all three panels
3. Preset buttons load correctly and plots respond
4. Sliders update the plots in real time
5. Adding/removing elements works

Completion: all checks pass.

## Step 6 (Future) — Frequency-Domain Design Extensions

Once the base Bode explorer is working, extend it into a full frequency-domain design tool:

### Compensator Mode

- User selects a compensator type (lead, lag, lead-lag, notch) from a dropdown
- Quick-design wizard: input PM deficit → auto-compute lead compensator zero/pole/gain per the procedure in `knowledge/control-theory/compensator-design.md`
- Show before/after Bode overlaid: plant-only (dimmed) vs. plant+compensator (primary)
- Display the compensator transfer function expression and its contribution to phase/magnitude

### Nyquist View

- Toggle between Bode and Nyquist plot of the same open-loop transfer function L(s)
- Plot Re(L(jw)) vs Im(L(jw)) with parametric frequency annotation
- Draw the unit circle and mark the (-1, 0) critical point
- Show gain margin (distance from negative real-axis crossing to -1) and phase margin (angle from crossing on unit circle to negative real axis) visually
- Handle integrators by showing the infinite-radius semicircle as a dashed arc
- Reference: `knowledge/control-theory/nyquist.md`

### Margin Annotations (Enhanced)

- On Bode: mark PM and GM with colored segments (already specified in Step 3)
- On Nyquist: draw a line from origin to the unit-circle crossing (PM) and from the real-axis crossing to (-1, 0) (GM)
- Tooltip on hover showing the numeric values

### Root Locus View (Stretch)

- Given G(s) poles and zeros, compute and display the root locus as K varies
- Interactive K slider: highlight the current closed-loop pole locations
- Mark the jw-axis crossing (critical gain)
- Reference: `knowledge/control-theory/root-locus.md`

## Rules

- Ground all transfer function math in `knowledge/control-theory/frequency-response.md`. Do not invent formulas.
- The logic module must have zero rendering dependencies — it is the portable piece.
- Use `ImPlotSpec` for line styling, not the deprecated `SetNextLineStyle`.
- The high-pass preset must include the 1/w_c gain factor. Verify by checking that magnitude → 0 dB at high frequencies.
- Phase unwrapping is mandatory. Without it, systems with multiple poles show discontinuous phase jumps.
- Compensator design formulas must match `knowledge/control-theory/compensator-design.md` exactly.
- Nyquist plot must handle the indentation for imaginary-axis poles (integrators) per `knowledge/control-theory/nyquist.md`.
