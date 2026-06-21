---
title: Real-Time Simulation Loop
sources:
  - { note: "engineering experience" }
requires:
  - fixed-timestep.md
related:
  - rk4.md
  - variable-vs-fixed.md
---

# Real-Time Simulation Loop

The simulation loop is the heartbeat of any real-time physics application. It orchestrates three concerns on every frame: reading input, advancing physics, and rendering the result. For control systems like the ball-balancer, the loop must also guarantee deterministic physics timing -- the control algorithm's correctness depends on a stable, predictable sample period.

## Key Equations

### The Canonical Loop

```
while running:
    1. Read input (sensors, user commands, network)
    2. Step physics (fixed-timestep accumulator + RK4)
    3. Apply constraints and contact resolution
    4. Run control algorithm (PID, state feedback)
    5. Render (interpolated state)
```

Steps 2-4 run at the physics rate (fixed dt). Step 5 runs at the display rate (variable, typically 60 Hz or vsync). The accumulator pattern from `fixed-timestep.md` bridges the two rates.

### Clock Management: Wall Time vs. Sim Time

Two independent clocks run simultaneously:

| Clock | Source | Purpose |
|---|---|---|
| **Wall time** | `std::chrono::steady_clock` | Frame delta measurement, profiling, real-time pacing |
| **Sim time** | Accumulated `dt` increments | Physics state, control loop, event scheduling |

They diverge under load: if physics substeps take longer than `dt` of wall time, sim time falls behind. The frame-time clamp (see `fixed-timestep.md`) prevents runaway divergence by sacrificing real-time fidelity rather than locking up.

```
sim_time += dt;             // incremented per physics substep
wall_time = clock.now();    // measured per frame
drift = wall_time - sim_time;  // positive = sim lagging
```

For offline simulation (no real-time constraint), wall time is irrelevant -- advance sim_time as fast as the CPU allows.

### Determinism Requirements

For control systems, determinism means: **identical inputs produce identical outputs, every time.** This requires:

1. **Fixed dt** -- no variable-step integration
2. **Deterministic floating-point** -- consistent rounding (avoid `-ffast-math` in the physics path; use `#pragma STDC FENV_ACCESS ON` if needed)
3. **Input replay** -- record timestamped inputs for regression testing
4. **No data races** -- if multi-threaded, physics state must be fully isolated during a tick

Determinism enables replay-based debugging: record inputs, reproduce bugs exactly.

## Implementation Notes

### Single-Threaded Loop (Desktop + Emscripten)

```cpp
// Desktop: explicit loop
while (running) {
    double frame_dt = timer.restart();
    process_input();
    accumulator_step(state, frame_dt, dt);  // physics substeps
    render(interpolated_state(state, previous_state, accumulator, dt));
}

// Emscripten: browser drives the loop
void frame_callback(void* arg) {
    auto* sim = static_cast<Simulation*>(arg);
    double frame_dt = sim->timer.restart();
    sim->process_input();
    sim->accumulator_step(frame_dt);
    sim->render();
}
emscripten_set_main_loop_arg(frame_callback, &sim, 0, true);
```

The Emscripten build collapses to single-threaded by necessity -- `requestAnimationFrame` is the scheduler, and there is no `std::thread` in the browser (without SharedArrayBuffer). Physics and render share a single call stack, so no mutex is needed.

### Multi-Threaded Loop (Native Desktop)

```
Physics Thread                     Render Thread
──────────────                     ─────────────
while running:                     while running:
  wait_until(next_tick)              lock(state_mutex)
  lock(input_mutex)                  copy render_state
  read_inputs()                      unlock(state_mutex)
  unlock(input_mutex)                interpolate + draw
  physics_step(dt)
  lock(state_mutex)
  write state to shared buffer
  unlock(state_mutex)
  next_tick += dt
```

**Thread architecture considerations:**

| Approach | Pros | Cons |
|---|---|---|
| Single-threaded | Simple, no sync overhead, deterministic | Physics limited to frame budget minus render time |
| Physics + Render threads | Physics runs at exact rate, render decoupled | Mutex contention, state copying, harder debugging |
| Triple-buffered | Lock-free reads (render always gets latest complete state) | More memory, slightly stale render state |

**For the ball-balancer:** start single-threaded. Only split threads if profiling shows the render budget crowds out physics ticks. The Emscripten target is single-threaded regardless.

### Shared State Protection (Monitor Pattern)

When using two threads, wrap the shared physics state in a simple monitor:

```cpp
class SharedState {
    mutable std::mutex mtx_;
    State state_;
    State previous_state_;
public:
    void write(const State& s, const State& prev) {
        std::lock_guard lock(mtx_);
        previous_state_ = prev;
        state_ = s;
    }
    std::pair<State, State> read() const {
        std::lock_guard lock(mtx_);
        return {state_, previous_state_};
    }
};
```

This is the simple monitor pattern from RTOS design -- the mutex is private, access is only through public interface functions, making misuse structurally impossible.

### Timing: sleep_until vs. busy-wait

For the physics thread's fixed-rate pacing:

```cpp
auto next_tick = steady_clock::now();
while (running) {
    std::this_thread::sleep_until(next_tick);
    physics_step(dt);
    next_tick += microseconds(static_cast<int>(dt * 1e6));
}
```

`sleep_until` yields the CPU between ticks (power-efficient). For sub-millisecond precision on Linux, consider `clock_nanosleep` with `TIMER_ABSTIME`. Busy-waiting gives lowest jitter but wastes CPU -- only justified for hard-real-time on dedicated cores.

### Profiling the Loop

Instrument each phase to detect budget overruns:

```cpp
auto t0 = steady_clock::now();
process_input();
auto t1 = steady_clock::now();
accumulator_step(frame_dt);
auto t2 = steady_clock::now();
render();
auto t3 = steady_clock::now();

metrics.input_us  = duration_cast<microseconds>(t1 - t0).count();
metrics.physics_us = duration_cast<microseconds>(t2 - t1).count();
metrics.render_us  = duration_cast<microseconds>(t3 - t2).count();
```

If `physics_us` consistently exceeds `dt * 1e6`, the simulation cannot keep up in real time. Options: increase dt (reduce accuracy), simplify the dynamics model, or move to a faster integrator (unlikely to help -- RK4 is already efficient for 6D state).
