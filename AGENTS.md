# Caliburn

Caliburn is an open-source engineering AI workspace. You are an engineering assistant working inside it.

## Vision

Caliburn is an agent-first control-theory builder. Working with the user, you:

1. **Model** the system they want to control.
2. **Design** a controller that fits that model.
3. **Simulate** both, with an OpenGL scene and an ImGui/ImPlot UI.

Every project takes the same shape, so that you can assemble each one from the
golden source in `reference/` and the theory in `knowledge/`. That shape is
recorded in `docs/adr/`. Read those ADRs before designing a project or changing
its architecture, and use the vocabulary in `CONTEXT.md`.

## Repo Structure

| Path | Purpose |
|---|---|
| `knowledge/` | Structured engineering knowledge (theory, concepts, equations) |
| `reference/` | Golden-source C++ implementations (standalone, tested, Eigen-based) |
| `projects/` | User workspace — each subfolder is its own git repo |
| `tools/rag/` | Knowledge ingestion pipeline (PDF → structured markdown) |
| `.pi/skills/` | Engineering skills — `new-project`, `design-controller`, `design-observer`, `explain-concept`, `validate-implementation`, `scaffold-sim-viewer`, `grill-mechanism`, `validate-mechanism` |
| `.pi/extensions/` | TypeScript extensions (sub-agent orchestration) |
| `.sandcastle/` | Sandboxed AFK agent runs (see below) |

## Knowledge Conventions

- Knowledge files use YAML frontmatter: `sources`, `requires`, `related`, `reference`
- Navigate top-down: `knowledge/index.md` → category index → target file
- Every fact is cited via `sources`. Every prerequisite is linked via `requires`
- Reference C++ implementations are linked from knowledge files via `reference` field

## Working on a Project

When the user asks to work on a project in `projects/`:
1. Read that project's `AGENTS.md` first
2. Use knowledge files and reference implementations as needed
3. Stay in the project context unless the user asks to switch

## Agent skills

### Issue tracker

Issues live in GitHub Issues on `caliburn-engineering/caliburn`, via the `gh` CLI. See `docs/agents/issue-tracker.md`.

### Triage labels

The five canonical triage roles, each label string equal to its name. See `docs/agents/triage-labels.md`.

### Domain docs

Single-context — one `CONTEXT.md` + `docs/adr/` at the repo root. See `docs/agents/domain.md`.

### AFK runs

`.sandcastle/` configures [Sandcastle](https://github.com/mattpocock/sandcastle)
to run an agent unattended in a Docker container against `ready-for-agent` issues.

| File | Purpose |
|---|---|
| `caliburn.ts` | Shared sandbox contract — image, skill mounts, notification bridge |
| `main.ts` | The AFK loop. Commits land on `sandcastle/afk`, never on `main` |
| `smoke.ts` | Tracer bullet — verifies the image is correctly provisioned |
| `prompt.md` | What the AFK agent does each iteration |
| `notify.sh` | Host-side notify-send/TTS bridge (container hooks can't reach the host) |

Launch with `npx tsx .sandcastle/main.ts`, or `work caliburn afk` for a tmux
session with a live log tail that survives disconnects.

Two things worth knowing before you run it:

- **`projects/` is gitignored**, so it is absent from any worktree-based run. To
  work on a project, anchor the run there with sandcastle's `cwd` option — each
  project is its own git repo.
- **Both harnesses are installed** in the image. Switch between `agents.pi` and
  `agents.claude` in `main.ts`; no rebuild needed.
