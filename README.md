# Caliburn

An open-source engineering AI workspace that makes control theory accessible without MATLAB.

Caliburn is **MATLAB, not a library**. You clone it, work inside it, and build projects within it — powered by [Pi](https://pi.dev), an open-source AI coding agent that supports 20+ LLM providers.

## What's Inside

- **Knowledge base** — structured engineering theory (control systems, linear algebra, state estimation, simulation) sourced from textbooks with full citations
- **Reference implementations** — golden-source C++ code using Eigen, each file standalone and tested
- **Engineering skills** — AI-native workflows for controller design, observer selection, concept exploration
- **RAG pipeline** — automated textbook-to-knowledge ingestion (nightly cron)

## Getting Started

### 1. Install Pi

```bash
curl -fsSL https://pi.dev/install | sh
```

### 2. Set Up Your LLM Provider

Pi supports any provider — Anthropic, OpenAI, Google, Ollama, and more. Configure your preferred provider:

```bash
pi provider add
```

### 3. Clone Caliburn

```bash
git clone https://github.com/caliburn-engineering/caliburn.git
cd caliburn
```

### 4. Start Working

```bash
pi
```

Pi loads `AGENTS.md` automatically, discovering all skills and knowledge. You're ready to build.

### 5. Create a Project

Ask Pi to create a new project:

```
> /new-project ball-balancer
```

This scaffolds a project in `projects/ball-balancer/` with its own `AGENTS.md`, `CMakeLists.txt`, and git repo.

## Reference Implementations

All C++ reference code lives in `reference/` and uses [Eigen](https://eigen.tuxfamily.org/) for linear algebra:

| Module | What |
|---|---|
| `reference/integrators/` | RK4 numerical integration |
| `reference/controllers/` | PID, LQR controllers |
| `reference/observers/` | Kalman filter, EKF |
| `reference/simulation/` | Fixed-timestep simulation loops |
| `reference/rendering/` | ImGui + ImPlot real-time visualisation |

Build and test:

```bash
cd reference && mkdir build && cd build
cmake .. && make && ctest
```

## Knowledge Base

Navigate from the root: [`knowledge/index.md`](knowledge/index.md)

Every knowledge file includes:
- **sources** — textbook/paper citations
- **requires** — prerequisite topics (dependency graph)
- **related** — sibling topics
- **reference** — link to C++ implementation

## Contributing

Knowledge contributions welcome via PR. Each knowledge file must include at least one `sources` entry in its frontmatter.

## License

MIT
