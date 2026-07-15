---
name: scaffold-sim-viewer
description: Scaffold an interactive ImGui+OpenGL visualizer for a simulation project. Use when the user wants visualization, a viewer, a 3D viewport, real-time plots, or an interactive GUI for their simulation.
---

# Scaffold Sim Viewer

Generate a complete interactive visualization for any Caliburn project that already has a simulation or kinematics module. The viewer is a **harness** — it wraps the user's existing code in a GUI without modifying it.

## Step 1 — Identify the harness target

Read the project's source files. Identify:

1. **State struct** — what gets visualized (e.g. `TablePose`, `BallState`)
2. **Tick function** — what advances the simulation one step
3. **Parameters** — what the user tunes (gains, angles, masses)

Completion: you can name the struct, the tick call, and at least 3 tunable parameters.

## Step 2 — Generate CMake

Add to the project's `CMakeLists.txt` (or create a new one) with FetchContent for:

| Dependency | Source | Tag |
|---|---|---|
| GLFW | `github.com/glfw/glfw` | `3.4` |
| ImGui | `github.com/ocornut/imgui` | `docking` (always the docking branch) |
| ImPlot | `github.com/epezent/implot` | `master` |
| GLAD | Vendored (`vendor/glad/`) | GL 3.3 Core |

If GLAD files don't exist in the project, generate them:
```bash
uv tool run glad --api gl=3.3 --profile core --out-path /tmp/glad --generator c
```
Then copy into `vendor/glad/`.

Build targets: the existing simulation as a library, plus a `visualizer` executable.

Completion: `cmake -B build` succeeds with all dependencies resolved.

## Step 3 — Generate renderer

Create `src/renderer.h` and `src/renderer.cpp` from the reference at `reference/rendering/`. The renderer provides:

- **OrbitCamera** — azimuth, elevation, distance, target. Mouse drag to orbit, scroll to zoom.
- **LineRenderer** — batched line drawing with per-vertex color via GL 3.3 shaders.
  - `line()`, `circle()`, `disc()` (filled triangle fan with alpha), `point()`, `axes()`
  - Triangle batch for filled geometry, rendered before lines (depth-correct transparency).
- **Matrix helpers** — `perspective()`, `look_at()` returning `Eigen::Matrix4f`.

Completion: renderer compiles standalone (no project-specific code).

## Step 4 — Generate visualizer

Create `src/visualizer.cpp` with the full main loop. Every viewer must include these **invariants** — they are non-negotiable:

### Font
- Load a Unicode-capable font (Noto Sans, DejaVu Sans, or similar) via `io.Fonts->AddFontFromFileTTF()` with Greek and mathematical symbol ranges merged. This enables proper labels (α, β, θ, φ, ω, etc.) throughout the UI. Bundle the `.ttf` in `vendor/fonts/`.

### Docking & persistence
- `ImGuiConfigFlags_DockingEnable` always set.
- Full-window dockspace (`ImGuiDockNodeFlags_PassthruCentralNode`).
- `io.IniFilename = "imgui.ini"` — window positions persist between sessions.
- All ImGui windows are dockable (no `ImGuiWindowFlags_NoDocking` on content windows).

### Reset
- A red **"Reset All"** button that restores every piece of application state to its initial value: parameters, camera, animation, and any solver warm-start. It must be impossible for the user to reach a state they cannot recover from.

### Controls panel
- Sliders for each tunable parameter identified in Step 1.
- Preset buttons for common configurations.
- A **"Pause"** button that freezes simulation time (see Plots panel for pause behaviour).
- FK/IK/solver status with color-coded text (green = OK, red = FAIL).
- Collapsible sections for advanced info (Jacobian, raw state).

### 3D Viewport
- Rendered behind the docked panels via `PassthruCentralNode`.
- Orbit camera with mouse drag + scroll zoom.
- Ground grid, coordinate axes (RGB = XYZ), display toggles.
- All planar surfaces rendered as filled `disc()` with low alpha (0.10–0.15). Disable depth writes (`glDepthMask(GL_FALSE)`) before drawing filled surfaces so geometry behind them remains visible. Re-enable depth writes after. Edge `circle()` drawn on top with full opacity.

### Plots panel — time series invariants

Every time series plot must follow these rules:

**Grouping: one plot per state type**
- Group signals by physical type, not by individual state. States that share the same unit and meaning go in one plot. Examples: all positions in one plot, all velocities in one plot, all angles in one plot.
- Good: "Position [m]" with traces x, y. "Velocity [m/s]" with traces vx, vy. "Servo Angles [deg]" with traces α₀, α₁, α₂.
- Bad: separate plots for x position and y position. Separate plots for each servo angle.
- When comparing models (e.g. linear vs nonlinear), both model traces for each state appear in the same state-type plot. Use solid lines for one model and dashed for the other, or distinct color families (blue/orange for model A, green/red for model B).

**Layout**
- Plots auto-size their height to divide available vertical space evenly, with a minimum height of 120 pixels per plot. If the available space cannot fit all plots at minimum height, the panel becomes scrollable.
- **Plot visibility toggles**: a row of toggle buttons at the top of the plots panel, one per plot, labelled with the plot title. Clicking a button hides/shows that plot. Hidden plots free their vertical space for visible ones. The height calculation and scrolling logic use only the count of visible plots.
- Legend placed **outside** the plot, to the **right** (`ImPlotLegendFlags_Outside`).
- Legend labels are **max 2 characters**. The plot title carries the full context (e.g. title: "Servo Angles [deg]", legends: "α₀", "α₁", "α₂").

**Data & buffer**
- Scrolling circular buffer sized to at least 60 seconds of data at 60 fps (≥ 3600 samples).
- Use `ImPlotSpec` with `.Offset` for circular buffer rendering.
- A **"Clear Data"** button that flushes the buffer. Plots start refilling from empty.

**Time axis**
- **Time window slider** (2–60s range) controlling the visible x-axis span.

**Y-axis scaling**
- Y-axis auto-fits to the data **within the visible time window only**, not the full buffer. Compute min/max of visible data and set Y-limits with a small margin (5%).
- When a data series is hidden via legend toggle, the Y-limits exclude that series and refit to the remaining visible series only.

**Synchronized cursor**
- When the mouse hovers over any time series plot, draw a **vertical line at the cursor time on all time series plots** simultaneously.
- On the plot being hovered, show a **tooltip window** with the exact values of all visible series at that time instant.
- **Click** on a plot to place a **persistent marker**: a vertical line that remains on all plots plus a pinned tooltip on the clicked plot. Multiple markers can coexist. Provide a way to clear all markers.

**Pause & time-zoom**
- When simulation is paused, the time axis stops advancing and data stops appending.
- While paused, show **time range sliders** (start/end) that let the user zoom into any sub-range of the buffered data for analysis.
- Hover cursor and click-markers remain functional while paused.

### Blending
- `glEnable(GL_BLEND)` + `glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA)` always active.

Completion: `cmake --build build` succeeds. The visualizer launches, displays the 3D scene, and all panels respond to input. The Reset button recovers from any state.

## Step 5 — Verify

1. Build: `cmake -B build && cmake --build build`
2. Launch: `timeout 3 ./build/visualizer` exits without error
3. Check: `imgui.ini` is generated after first run
4. Add `imgui.ini` to `.gitignore`

Completion: all four checks pass. Commit with a message listing the generated files and dependencies.
