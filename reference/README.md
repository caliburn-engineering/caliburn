# Reference Implementations

Golden-source C++ implementations of core engineering algorithms.

## Conventions

- **Standalone** — each file compiles and runs independently
- **Tested** — every implementation has a corresponding `test_*.cpp`
- **Eigen-based** — all linear algebra uses [Eigen](https://eigen.tuxfamily.org/)
- **Header + source** — `algorithm.h` (interface) + `algorithm.cpp` (implementation)
- **Linked from knowledge** — each implementation is referenced from its corresponding knowledge file via the `reference` frontmatter field

## Structure

| Folder | Contents |
|---|---|
| `integrators/` | Numerical integration (RK4) |
| `controllers/` | Feedback controllers (PID, LQR) |
| `observers/` | State estimators (Kalman filter, EKF) |
| `simulation/` | Simulation loop patterns (fixed-timestep, accumulator) |
| `math/` | Coordinate transforms (homogeneous matrices) |
| `rendering/` | ImGui + ImPlot setup and helpers (no tests — UI code) |

## Building

```bash
mkdir build && cd build
cmake .. && make && ctest
```

## Writing New Implementations

1. Write the algorithm fresh — do NOT extract from existing project code
2. Start concrete and simple; generalise only when multiple projects demand it
3. Include a test file that validates against known analytical solutions
4. Link from the corresponding knowledge file's `reference` frontmatter field
