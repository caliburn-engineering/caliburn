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

### Watching runs — the Factory Floor

`tools/factory/` is a local dashboard over AFK runs and the issue tracker.

```bash
npm run factory              # http://127.0.0.1:4600
HOST=0.0.0.0 npm run factory # reachable from the phone over Tailscale
```

It reads `.sandcastle/logs/*.log` — the one artefact every run leaves behind —
and re-parses the ones that grew every few seconds, so a run in flight shows up
without any wiring in `main.ts`. Parsed traces land in `.sandcastle/logs/trace.db`
(gitignored); the log file stays the source of truth, so deleting the db costs
nothing.

| Path | Purpose |
|---|---|
| `db.ts` | Schema and connection. Close to sssf.db's shape so either UI could read either db |
| `ingest.ts` | Log grammar → runs, phases, events, skills; commits recovered from git by time window |
| `issues.ts` | `gh issue list` mapped onto the five triage labels, plus an "in flight" column the tracker cannot know |
| `server.ts` | JSON API + static UI, and the ingest loop |
| `public/` | The UI: vanilla, no build step |

Each run shows the ticket it is working — headline plus description, pulled from
the tracker — and its **conclusion**: the comment it filed via `gh issue close`
or `gh issue comment`, which is where `prompt.md` tells it to explain itself and
therefore where a question back to a human lands. Its closing prose comes second,
and a harness failure (a subscription rate limit, most often) outranks both and is
labelled as such rather than passed off as a finished run.

The issue number is recovered from the run name (`ticket-44`, `afk-39`), else the
branch (`sc/44-guardrail`), else the first `gh issue view N` the agent runs — so
any of the three naming habits works, but a trailing number is what makes it free.

Below that sits the **live ticket thread**, pulled from GitHub for the selected
run only. Two flags earn their place there: the comment this run filed (matched
against the report in its own trace, because the agent posts under your token and
the author name cannot tell you), and comments that arrived *after* the run
stopped — a reply of yours, or a later run, that nothing has answered yet.

The board reads `done` from GitHub state rather than a label, per
`docs/agents/triage-labels.md`. An issue counts as **in flight** when a run whose
status is still `running` names it.

`*.console.log` files are skipped: they are orchestrator stdout, and a file with
no run banner in it would otherwise be re-parsed on every tick.

Two things worth knowing before you run it:

- **`projects/` is gitignored**, so it is absent from any worktree-based run. To
  work on a project, anchor the run there with sandcastle's `cwd` option — each
  project is its own git repo.
- **Both harnesses are installed** in the image. Switch between `agents.pi` and
  `agents.claude` in `main.ts`; no rebuild needed.
