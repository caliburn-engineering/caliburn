---
title: Fixed-Timestep Accumulator Pattern
sources:
  - { note: "engineering experience" }
  - { article: "Glenn Fiedler - Fix Your Timestep", url: "https://gafferongames.com/post/fix_your_timestep/" }
requires:
  - rk4.md
related:
  - sim-loop.md
  - variable-vs-fixed.md
---

# Fixed-Timestep Accumulator Pattern

The core problem: frame time is variable (16.6 ms at 60 FPS, 33.3 ms at 30 FPS, and anything in between during load spikes), but physics must advance at a constant dt for determinism and numerical stability. The accumulator pattern decouples the two clocks by stockpiling elapsed wall-clock time and draining it in fixed-size physics substeps.

This is the standard approach for any real-time simulation where reproducibility or control-loop stability matters. The ball-balancer uses it to guarantee a 100 Hz physics tick regardless of render frame rate.

## Key Equations

### Accumulator Update

Each frame, the loop measures elapsed wall-clock time `frame_dt` and adds it to an accumulator:

```
accumulator += frame_dt
```

Then physics substeps drain the accumulator at fixed intervals:

```
while accumulator >= dt:
    physics_step(state, dt)
    sim_time += dt
    accumulator -= dt
```

After the loop, `0 <= accumulator < dt` -- the leftover fractional step.

### Render Interpolation

The renderer sees a state that is `accumulator` seconds behind the current wall-clock time. To eliminate visual stutter, interpolate between the previous and current physics states:

```
alpha = accumulator / dt
render_state = state_current * alpha + state_previous * (1 - alpha)
```

This produces smooth visuals even when physics runs at a lower rate than rendering. Without interpolation, objects appear to "jump" once per physics tick.

### Choosing dt

| Factor | Guidance |
|---|---|
| Accuracy | Smaller dt = lower truncation error. RK4 at dt = 0.01 s (100 Hz) gives O(10^-8) local error per step. |
| Stability | dt < T_fast / 10 where T_fast is the fastest oscillation period in the system. |
| CPU budget | Each substep costs one full physics evaluation (4 derivative calls for RK4). At 60 FPS render, dt = 0.01 s means ~1-2 substeps per frame. |
| Control loop | If a PID controller runs at the physics rate, dt sets the sample period. Too large = aliasing of high-frequency disturbances. |
| Determinism | Fixed dt guarantees bit-identical results across runs (same inputs = same outputs). Essential for replay, testing, and networked simulation. |

**Ball-balancer:** dt = 0.01 s (100 Hz). Fast enough for the rolling dynamics and PID control, cheap enough for real-time execution.

### Frame Time Clamping

If the application stalls (window drag, debugger break, alt-tab), `frame_dt` can spike to hundreds of milliseconds. Without clamping, the accumulator fills with dozens of substeps, causing a "spiral of death" -- each frame takes longer to simulate, which makes the next frame_dt even larger.

```
frame_dt = min(frame_dt, max_frame_dt)  // e.g., max_frame_dt = 0.25 s
```

This caps the number of substeps per frame. The simulation slows down relative to real time rather than locking up.

## Implementation Notes

### C++ Accumulator Loop

```cpp
const double dt = 0.01;            // 100 Hz physics
const double max_frame_dt = 0.25;  // cap to prevent spiral of death

double accumulator = 0.0;
double sim_time = 0.0;
State previous_state = state;

while (running) {
    double frame_dt = clock.elapsed();  // wall-clock since last frame
    frame_dt = std::min(frame_dt, max_frame_dt);
    accumulator += frame_dt;

    while (accumulator >= dt) {
        previous_state = state;
        state = rk4_step(state, sim_time, dt, derivatives);
        sim_time += dt;
        accumulator -= dt;
    }

    double alpha = accumulator / dt;
    State render_state = lerp(previous_state, state, alpha);
    render(render_state);
}
```

### Interpolation with Eigen

```cpp
State lerp(const State& a, const State& b, double alpha) {
    return a + alpha * (b - a);  // Eigen handles element-wise ops
}
```

For quaternion-based orientations (not applicable to the ball-balancer's small-angle model, but relevant for general 3D), use `slerp` instead of linear interpolation.

### Common Pitfalls

- **Forgetting the clamp:** a single 2-second stall without clamping produces 200 substeps at 100 Hz. The frame takes so long that the next frame_dt is even larger. This is the spiral of death.
- **Accumulator drift:** floating-point accumulation error over millions of frames. Periodically snap `sim_time` to the nearest integer multiple of dt, or use integer-based tick counting (`uint64_t tick_count * dt`).
- **Not storing previous_state:** without it, interpolation is impossible. The renderer would snap to discrete physics positions, causing visible judder at low physics rates.
- **Rendering physics state directly:** bypassing interpolation looks fine when render rate equals physics rate, but breaks visibly when they differ (e.g., 144 Hz monitor, 100 Hz physics).

### Single-Threaded vs. Multi-Threaded

In a single-threaded loop (including Emscripten/WASM builds), physics and render alternate within the same frame callback. The accumulator pattern works identically -- the only difference is that `requestAnimationFrame` drives the outer loop instead of a `while(running)`.

In a multi-threaded design (native desktop), the physics loop runs on its own thread with `std::this_thread::sleep_until` for timing, and the render thread reads interpolated state through a mutex-protected shared buffer. See `sim-loop.md` for thread architecture details.
