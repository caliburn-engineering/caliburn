---
name: design-controller
description: Guide controller selection for a plant/system based on the Caliburn knowledge base
---

# Design Controller

Walk the user through choosing and adapting a feedback controller for their plant. Every recommendation must be grounded in knowledge files — never invent control theory claims.

## Step 1 — Characterise the Plant

Ask the user to describe their system. Collect at minimum:

- **SISO or MIMO** — single-input/single-output vs multi-input/multi-output
- **Linear or nonlinear** — and if nonlinear, whether a valid operating point exists for linearization
- **Sensors and actuators** — what signals are measurable, what inputs are available
- **Performance requirements** — bandwidth, settling time, overshoot, disturbance rejection, robustness constraints

Do not proceed until the plant is sufficiently characterised.

## Step 2 — Survey Available Controllers

Read `knowledge/control-theory/controllers/index.md` to see the current controller catalogue. As of writing:

| Controller | Knowledge file | Reference header |
|---|---|---|
| PID | `knowledge/control-theory/controllers/pid.md` | `reference/controllers/pid.h` |
| LQR | `knowledge/control-theory/controllers/lqr.md` | `reference/controllers/lqr.h` |

Always read the index at runtime — new controllers may have been added.

## Step 3 — Evaluate Candidates

For each candidate controller, read its knowledge file and extract:

- **Applicability** — what plant types it suits (from the knowledge file body)
- **Prerequisites** — the `requires` field in YAML frontmatter (e.g. state-space model, stability analysis)
- **Trade-offs** — complexity, tuning difficulty, optimality guarantees
- **Related topics** — the `related` field pointing to supporting knowledge

Read prerequisite knowledge files as needed:
- `knowledge/control-theory/state-space.md` — if the controller requires a state-space model
- `knowledge/control-theory/stability.md` — for stability analysis prerequisites

## Step 4 — Recommend with Rationale

Present a recommendation structured as:

1. **Recommended controller** and why it fits this plant
2. **Key trade-offs** vs the alternatives considered
3. **Prerequisites the user needs** — cite the `requires` frontmatter entries and link to their knowledge files
4. **Reference implementation** — point to the `.h` file (e.g. `reference/controllers/pid.h`)

Every claim must cite a specific knowledge file path. If the knowledge base does not cover something, say so explicitly rather than filling the gap.

## Step 5 — Nonlinear Systems

If the user's plant is nonlinear:

- Discuss linearization around an operating point and whether the chosen controller remains valid within the expected operating envelope
- Check which controller's knowledge file addresses nonlinear applicability
- If neither controller handles the nonlinearity well, state the limitation honestly and suggest what knowledge would need to be added

## Step 6 — Walk Through the Implementation

Offer to:

1. Read the full reference implementation (`reference/controllers/<name>.h` and `reference/controllers/<name>.cpp`)
2. Walk through adapting the reference code to the user's specific plant — state dimensions, tuning parameters, sample rate
3. Identify any supporting reference code the user will need (e.g. state-space utilities, matrix operations)

## Navigation Path

```
knowledge/index.md
  → knowledge/control-theory/index.md
    → knowledge/control-theory/controllers/index.md
      → knowledge/control-theory/controllers/pid.md
      → knowledge/control-theory/controllers/lqr.md
    → knowledge/control-theory/state-space.md
    → knowledge/control-theory/stability.md
```

## Rules

- Ground every recommendation in knowledge files. No hallucinated control theory.
- If the knowledge base lacks coverage for the user's scenario, say so.
- Read files at runtime — do not assume content matches what was true when this skill was written.
- Prefer the simplest controller that meets requirements. Do not recommend LQR when PID suffices.
