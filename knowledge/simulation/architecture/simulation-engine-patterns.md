---
title: Simulation Engine Patterns
sources:
  - { book: "Cooling - Software Engineering for Real-Time Systems", chapter: "3-4" }
  - { note: "engineering experience" }
requires:
  - ../sim-loop.md
  - ../fixed-timestep.md
related:
  - ../rk4.md
  - ../../control-theory/controllers/pid.md
  - ../../control-theory/observers/kalman-filter.md
reference: ../../../reference/simulation/interfaces.h
---

# Simulation Engine Patterns

A simulation engine for control systems is a modular pipeline: sense → estimate → control → integrate → render. Each stage has a well-defined interface so that concrete implementations can be swapped without changing the pipeline. This enables hot-swapping Euler ↔ RK4, PID ↔ LQR, KF ↔ EKF, or ImGui ↔ OpenGL without touching the loop.

## Architecture Overview

The engine follows a **component-based architecture** where each component implements a pure interface:

```
┌─────────────┐
│  SimEngine   │ — owns the loop, delegates to components
├─────────────┤
│ Integrator   │ — advances state by dt (RK4, Euler, implicit)
│ Controller   │ — computes control output from error/state
│ Observer     │ — estimates state from measurements
│ Plant        │ — defines system dynamics (derivative function)
│ Renderer     │ — visualises state (ImGui, OpenGL, headless)
└─────────────┘
```

### Interface Segregation

Each component is defined by a single abstract base class with one primary method:

| Component | Method | Signature |
|---|---|---|
| Integrator | `step` | `(state, t, dt, derivative_fn) → new_state` |
| Controller | `compute` | `(setpoint, measurement, dt) → control_output` |
| Observer | `update` | `(measurement, control_input, dt) → estimated_state` |
| Plant | `derivatives` | `(t, state, control_input) → state_derivative` |
| Renderer | `render` | `(state, alpha) → void` |

The `SimEngine` composes these via dependency injection (constructor parameters or setter methods). No component knows about any other component's concrete type.

### Update Loop

The canonical simulation loop using these interfaces:

```
while running:
    frame_dt = clock.elapsed()

    for each physics substep (fixed dt):
        measurement = plant.observe(state)       // sensor model
        estimate    = observer.update(measurement, u, dt)
        u           = controller.compute(setpoint, estimate, dt)
        state       = integrator.step(state, t, dt,
                        [&](t, s) { return plant.derivatives(t, s, u); })
        t += dt

    renderer.render(state, interpolation_alpha)
```

This separates physics rate (fixed dt, e.g. 1 kHz) from render rate (variable, e.g. 60 Hz) using the fixed-timestep accumulator pattern (see `fixed-timestep.md`).

### Swappability Examples

| Swap | From → To | What changes | What doesn't |
|---|---|---|---|
| Integrator | Euler → RK4 | `integrator` argument | Loop, controller, observer |
| Controller | PID → LQR | `controller` argument | Loop, integrator, observer |
| Observer | KF → EKF | `observer` argument | Loop, integrator, controller |
| Renderer | ImGui → OpenGL | `renderer` argument | Entire physics pipeline |
| Plant | Ball-balancer → Pendulum | `plant` argument | All infrastructure |

### When to Use Inheritance vs. Templates

| Approach | Use when | Trade-off |
|---|---|---|
| Virtual interface (ABC) | Components selected at runtime (user picks controller from menu) | vtable overhead (~1 ns/call) |
| CRTP / templates | Components fixed at compile time, performance-critical inner loop | No runtime overhead, but harder to read |
| `std::function` | One-off callbacks (derivative function in integrator) | Flexible, slight overhead |

For the ball-balancer, virtual interfaces are the right choice: the vtable cost is negligible at 1 kHz, and the ability to swap components at runtime enables interactive experimentation.

### Game Engine Patterns That Apply

**Entity-Component-System (ECS):** In game engines, entities are IDs, components are pure data, and systems are functions that operate on component sets. For a single-body control system simulation, ECS is overkill. It becomes relevant when simulating many bodies (e.g., particle systems, multi-agent scenarios).

**Fixed-timestep accumulator:** Already implemented in `fixed-timestep.md`. The simulation runs at a fixed rate; the render loop interpolates between physics states for smooth display.

**Double-buffering state:** The physics thread writes to a back-buffer while the render thread reads from the front-buffer, then swap. Avoids mutex contention on the shared state. Useful when physics and render run on separate threads.

## Implementation Notes

The reference implementation provides C++ abstract base classes for Integrator, Controller, Observer, Plant, and Renderer. These are header-only (no `.cpp` needed — pure virtual interfaces have no implementation). See `reference/simulation/interfaces.h`.

Concrete implementations (RK4, PID, Kalman) already exist in `reference/` and can be adapted to implement these interfaces. The ball-balancer project (Phase 6) will be the first consumer.
