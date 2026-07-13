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
- **State observability** — are all states directly measurable, or is an observer needed?
- **Constraints** — hard limits on states or inputs (saturation, safety bounds)?
- **Performance requirements** — bandwidth, settling time, overshoot, disturbance rejection, robustness constraints
- **Model uncertainty** — how confident is the plant model? Are there significant unmodelled dynamics?

Do not proceed until the plant is sufficiently characterised.

## Step 2 — Survey Available Controllers

Read `knowledge/control-theory/controllers/index.md` to see the current controller catalogue. Also read `knowledge/control-theory/controllers/comparison.md` for the comparison table and decision tree.

As of writing:

| Controller | Knowledge file | Reference header |
|---|---|---|
| PID | `knowledge/control-theory/controllers/pid.md` | `reference/controllers/pid.h` |
| LQR | `knowledge/control-theory/controllers/lqr.md` | `reference/controllers/lqr.h` |
| LQG | `knowledge/control-theory/controllers/lqg.md` | — |
| H-infinity | `knowledge/control-theory/controllers/h-infinity.md` | — |
| SMC | `knowledge/control-theory/controllers/sliding-mode.md` | `reference/controllers/smc.h` |

Always read the index at runtime — new controllers may have been added.

## Step 3 — Apply the Decision Tree

Use the selection decision tree from `comparison.md` to narrow candidates. Key branching questions:

1. **Is the plant linear?** If no → consider SMC or gain-scheduling.
2. **Are all states measurable?** If no → need an observer → LQG, or output feedback → H-infinity.
3. **Are there hard constraints on states/inputs?** If yes → MPC is the natural choice.
4. **Is robustness to model uncertainty the primary concern?** If yes → SMC or H-infinity.
5. **Is the system SISO or MIMO?** SISO with simple dynamics favours PID; MIMO favours LQR/LQG/MPC.

Navigate the tree based on the plant characterisation from Step 1.

## Step 4 — Evaluate Candidates

For each candidate controller remaining after the decision tree, read its knowledge file and extract:

- **Applicability** — what plant types it suits (from the knowledge file body)
- **Prerequisites** — the `requires` field in YAML frontmatter (e.g. state-space model, stability analysis)
- **Trade-offs** — complexity, tuning difficulty, optimality guarantees, robustness margins
- **Related topics** — the `related` field pointing to supporting knowledge

Read prerequisite knowledge files as needed:
- `knowledge/control-theory/state-space.md` — if the controller requires a state-space model
- `knowledge/control-theory/stability.md` — for stability analysis prerequisites
- `knowledge/control-theory/frequency-response.md` — for frequency-domain design methods
- `knowledge/control-theory/observers/kalman-filter.md` — if LQG is a candidate

## Step 5 — Recommend with Rationale

Present a recommendation structured as:

1. **Recommended controller** and why it fits this plant
2. **Key trade-offs** vs the alternatives considered
3. **Prerequisites the user needs** — cite the `requires` frontmatter entries and link to their knowledge files
4. **Reference implementation** — point to the `.h` file if one exists (e.g. `reference/controllers/pid.h`)

Every claim must cite a specific knowledge file path. If the knowledge base does not cover something, say so explicitly rather than filling the gap.

## Step 6 — Nonlinear Systems

If the user's plant is nonlinear:

- **First consider SMC** — it handles nonlinear plants directly without linearization. Read `knowledge/control-theory/controllers/sliding-mode.md` for applicability.
- If SMC is not suitable (e.g., actuator bandwidth is insufficient, unmatched disturbances dominate, or optimality is needed), discuss linearization around an operating point and whether PID/LQR remains valid within the expected operating envelope.
- For plants with varying operating points: discuss gain-scheduled PID or gain-scheduled LQR.
- Check which controller's knowledge file addresses nonlinear applicability.

### SMC Selection Criteria

Recommend SMC when:
- Plant is nonlinear AND robustness is the primary design goal
- Matched disturbances dominate AND actuator bandwidth is sufficient for switching
- Finite-time convergence is required
- A disturbance bound is known or can be estimated

Prefer PID or LQR over SMC when:
- Plant is well-modelled and linear (simpler design, optimality guarantees with LQR)
- Actuator bandwidth is limited (chattering cannot be adequately suppressed)
- Optimality is the primary concern (use LQR)
- Unmatched disturbances dominate (SMC does not reject these)

### LQG Selection Criteria

Recommend LQG when:
- Plant is linear and well-modelled
- Not all states are directly measurable (observer needed)
- Noise characteristics are reasonably known
- Robustness margins are not the primary concern

Avoid LQG when:
- All states are measurable (use LQR directly — better robustness)
- Must guarantee robustness margins (LQG has none — consider H-infinity or LQG/LTR)
- Plant is highly nonlinear (separation principle does not apply)

### H-infinity Selection Criteria

Recommend H-infinity when:
- Plant model has significant, bounded uncertainty
- Must guarantee performance across an operating range
- Output feedback is required (not all states measured) AND robustness is critical
- LQG margins are insufficient for the application

Avoid H-infinity when:
- Plant is well-modelled with low uncertainty (LQR/LQG is simpler, less conservative)
- Computational tools for synthesis are unavailable
- Plant is SISO with simple dynamics (loop-shaping PID is sufficient)

## Step 7 — Walk Through the Implementation

Offer to:

1. Read the full reference implementation (`reference/controllers/<name>.h` and `reference/controllers/<name>.cpp`)
2. Walk through adapting the reference code to the user's specific plant — state dimensions, tuning parameters, sample rate
3. Identify any supporting reference code the user will need (e.g. state-space utilities, matrix operations)

## Navigation Path

```
knowledge/index.md
  → knowledge/control-theory/index.md
    → knowledge/control-theory/controllers/index.md
      → knowledge/control-theory/controllers/comparison.md    ← START HERE for selection
      → knowledge/control-theory/controllers/pid.md
      → knowledge/control-theory/controllers/lqr.md
      → knowledge/control-theory/controllers/lqg.md
      → knowledge/control-theory/controllers/h-infinity.md
      → knowledge/control-theory/controllers/sliding-mode.md
    → knowledge/control-theory/state-space.md
    → knowledge/control-theory/stability.md
    → knowledge/control-theory/frequency-response.md
    → knowledge/control-theory/observers/kalman-filter.md
```

## Rules

- Ground every recommendation in knowledge files. No hallucinated control theory.
- If the knowledge base lacks coverage for the user's scenario, say so.
- Read files at runtime — do not assume content matches what was true when this skill was written.
- Prefer the simplest controller that meets requirements. Do not recommend LQR when PID suffices.
- Always start with the comparison table and decision tree before deep-diving into individual controllers.
- Cite the specific knowledge file path for every technical claim.
