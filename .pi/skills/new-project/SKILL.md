---
name: new-project
description: Scaffold a new engineering project under projects/ with CMake, Eigen, git init, and links to relevant Caliburn knowledge
---

# New Project

Scaffold a new project in the `projects/` directory. Each project is its own git repo.

## Step 1 — Gather Info

Ask the user for:

1. **Project name** — lowercase-kebab-case (e.g. `inverted-pendulum`)
2. **One-line description** — what the project does
3. **Engineering domains** — which apply (pick all relevant):
   - `control` — controllers, state-space, stability
   - `simulation` — physics loops, integration, timestep
   - `math` — linear algebra, probability, numerical methods
   - `rendering` — real-time visualization, ImGui/ImPlot
   - `observers` — Kalman filters, state estimation

## Step 2 — Create Directory

```
projects/<project-name>/
├── AGENTS.md
├── CMakeLists.txt
├── README.md
├── .gitignore
└── src/
    └── main.cpp
```

## Step 3 — File Templates

### AGENTS.md

```markdown
# <Project Name>

<one-line description>

## Caliburn Resources

<List knowledge categories and reference implementations relevant to the selected domains. Link them as relative paths from the project root, e.g. `../../knowledge/control-theory/` and `../../reference/controllers/pid.cpp`.>

## Build

cmake -B build && cmake --build build

## Structure

| Path | Purpose |
|---|---|
| `src/` | Source code |
| `build/` | Build output (gitignored) |
```

### CMakeLists.txt

```cmake
cmake_minimum_required(VERSION 3.20)
project(<project-name> LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# --- Eigen via FetchContent ---
include(FetchContent)
FetchContent_Declare(
    eigen
    GIT_REPOSITORY https://gitlab.com/libeigen/eigen.git
    GIT_TAG        3.4.0
    GIT_SHALLOW    TRUE
)
set(EIGEN_BUILD_DOC OFF CACHE BOOL "" FORCE)
set(EIGEN_BUILD_TESTING OFF CACHE BOOL "" FORCE)
set(BUILD_TESTING OFF CACHE BOOL "" FORCE)
FetchContent_MakeAvailable(eigen)

add_executable(${PROJECT_NAME} src/main.cpp)
target_link_libraries(${PROJECT_NAME} PRIVATE Eigen3::Eigen)
```

### src/main.cpp

```cpp
#include <Eigen/Dense>
#include <iostream>

// Caliburn knowledge: ../../knowledge/index.md
// Reference implementations: ../../reference/

int main() {
    std::cout << "<project-name>" << std::endl;
    return 0;
}
```

### README.md

```markdown
# <Project Name>

<one-line description>

## Build

```bash
cmake -B build
cmake --build build
./build/<project-name>
```

Part of the [Caliburn](https://github.com/caliburn-engineering/caliburn) engineering workspace.
```

### .gitignore

```
build/
.cache/
compile_commands.json
```

## Step 4 — Git Init

```bash
cd projects/<project-name>
git init
git add -A
git commit -m "init: scaffold <project-name>"
```

## Step 5 — Surface Relevant Knowledge

Based on the selected domains, list the relevant files the user should know about:

| Domain | Knowledge | Reference |
|---|---|---|
| `control` | `knowledge/control-theory/` | `reference/controllers/pid.cpp`, `reference/controllers/lqr.cpp` |
| `simulation` | `knowledge/simulation/` | `reference/simulation/sim_loop.cpp`, `reference/integrators/rk4.cpp` |
| `math` | `knowledge/math/` | — |
| `rendering` | — | `reference/rendering/imgui_setup.cpp`, `reference/rendering/implot_helpers.cpp` |
| `observers` | `knowledge/control-theory/` (observers section) | `reference/observers/kalman_filter.cpp` |

Print only the rows matching the user's selected domains. Use paths relative to Caliburn root.
