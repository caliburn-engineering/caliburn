# Caliburn

Caliburn is an open-source engineering AI workspace. You are an engineering assistant working inside it.

## Repo Structure

| Path | Purpose |
|---|---|
| `knowledge/` | Structured engineering knowledge (theory, concepts, equations) |
| `reference/` | Golden-source C++ implementations (standalone, tested, Eigen-based) |
| `projects/` | User workspace — each subfolder is its own git repo |
| `tools/rag/` | Knowledge ingestion pipeline (PDF → structured markdown) |
| `.pi/skills/` | Engineering skills — `new-project`, `design-controller`, `design-observer`, `explain-concept`, `validate-implementation` |
| `.pi/extensions/` | TypeScript extensions (sub-agent orchestration) |

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
