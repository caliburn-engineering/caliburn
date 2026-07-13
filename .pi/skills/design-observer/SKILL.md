---
name: design-observer
description: Guide observer/estimator selection grounded in the Caliburn knowledge base
---

# Design Observer

Walk the user through choosing and configuring a state observer/estimator. Every recommendation must cite a specific knowledge file — never generate estimation theory from training data.

## Step 1 — Gather the Estimation Problem

Ask the user to describe:

- **States to estimate** — which variables are not directly measurable?
- **Available sensors** — what measurements feed the estimator?
- **Noise characteristics** — process noise profile, measurement noise magnitude, any known covariance structure
- **Dynamics** — linear or nonlinear? Time-varying? If nonlinear, can a Jacobian be computed analytically?

Do not proceed until all four are answered.

## Step 2 — Survey Available Observers

Read `knowledge/control-theory/observers/index.md` to see the current observer catalogue. As of now:

| Observer | Knowledge File |
|---|---|
| Luenberger Observer | `knowledge/control-theory/observers/luenberger.md` |
| Discrete-Time Kalman Filter | `knowledge/control-theory/observers/kalman-filter.md` |
| Extended Kalman Filter | `knowledge/control-theory/observers/extended-kalman.md` |

### Luenberger Observer
- **When:** Deterministic system, noise characteristics unknown or unimportant, want direct control over convergence speed
- **Prerequisites:** Observable system (A, C), desired convergence rate relative to controller
- **Design:** Place observer poles 2-10x faster than controller poles
- **Strengths:** Simple, transparent, direct pole control, no noise model needed
- **Weaknesses:** No noise optimality, requires manual pole selection
- **Knowledge file:** `knowledge/control-theory/observers/luenberger.md`
- **Reference:** `reference/observers/luenberger.h`

If the index lists new entries, include them. Never recommend an observer that has no knowledge file.

## Step 3 — Evaluate Candidates

For each candidate observer, read its knowledge file. Extract:

- **Applicability conditions** — when does this observer work well?
- **Tuning considerations** — what knobs exist, what are the trade-offs?
- **Prerequisites** — check the `requires` field in YAML frontmatter (e.g., `knowledge/math/linear-algebra/eigenvalues.md`, `knowledge/control-theory/state-space.md`)

### Selection Logic

| Condition | Recommended Observer |
|---|---|
| Noise statistics known and matter | Kalman filter |
| Noise statistics unknown, want deterministic convergence | Luenberger |
| Nonlinear system | EKF (or Extended Luenberger with Jacobian) |
| Both Kalman and Luenberger need observability | Check observability first |

If the user's system is **linear with unknown noise**: start with Luenberger from `knowledge/control-theory/observers/luenberger.md`. It gives direct pole control without requiring a noise model.

If the user's system is **linear with known noise statistics**: start with the Kalman filter from `knowledge/control-theory/observers/kalman-filter.md`. It will optimally balance speed vs. noise rejection.

If the user's system is **nonlinear**, read both `knowledge/control-theory/observers/kalman-filter.md` and `knowledge/control-theory/observers/extended-kalman.md`. Compare:

- KF requires a linear state-space model — will not handle nonlinear dynamics without pre-linearization
- EKF linearizes via Jacobians at each step — handles mild nonlinearity but diverges on strongly nonlinear systems
- Luenberger can be extended to nonlinear systems by replacing A with the Jacobian (Extended Luenberger) but this is uncommon — prefer EKF
- Cite the specific trade-off sections from each knowledge file

## Step 4 — Recommend with Rationale

Present a single recommendation. Structure it as:

1. **Recommended observer** and why it fits the stated problem
2. **Cited knowledge files** that support the recommendation
3. **Prerequisites** the user should understand first (from `requires` frontmatter) — link to `knowledge/control-theory/state-space.md`, `knowledge/math/linear-algebra/eigenvalues.md`, or `knowledge/math/linear-algebra/state-space-transforms.md` as needed

## Step 5 — Q and R Tuning Guidance

Walk through process noise covariance (Q) and measurement noise covariance (R) tuning using guidance from the recommended observer's knowledge file. Cover:

- How Q/R ratio affects estimator responsiveness vs. smoothness
- Initial tuning heuristics from the knowledge file
- Common failure modes (divergence from undertuned Q, noisy estimates from overtuned R)

Ground every statement in the knowledge file content — do not improvise tuning rules.

## Step 6 — Reference Implementation

Point the user to the reference C++ implementation for the recommended observer:

| Observer | Header | Source |
|---|---|---|
| Luenberger | `reference/observers/luenberger.h` | `reference/observers/luenberger.cpp` |
| Kalman Filter | `reference/observers/kalman_filter.h` | `reference/observers/kalman_filter.cpp` |

Read the `requires` frontmatter from the observer's knowledge file to list any mathematical prerequisites the user needs before reading the implementation.

## Step 7 — Offer to Walk Through Adaptation

Ask the user if they want to:

- Read the full reference implementation together and walk through adapting it to their system
- Discuss state-space model formulation using `knowledge/control-theory/state-space.md`
- Review the linear algebra foundations using `knowledge/math/linear-algebra/eigenvalues.md` or `knowledge/math/linear-algebra/state-space-transforms.md`

## Knowledge Navigation

Use the top-down index path to discover content:

```
knowledge/index.md
  -> knowledge/control-theory/index.md
    -> knowledge/control-theory/observers/index.md
      -> individual observer files
```

Never skip levels — always verify what the index contains before reading leaf files.
