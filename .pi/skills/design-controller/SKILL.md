---
name: design-controller
description: Guide controller design using the 6-step framework — from requirements through validation
---

# Design Controller

Walk the user through a complete control design process using the [6-step design framework](knowledge/control-theory/design-framework.md). Every recommendation must be grounded in knowledge files — never invent control theory claims.

## Overview — The 6-Step Framework

Read `knowledge/control-theory/design-framework.md` at session start. The steps are:

1. **Clarify requirements** — controlled variable, performance specs, constraints
2. **Model the plant** — inputs, outputs, states, linear/nonlinear, block diagram
3. **Assess controllability and observability** — can you control and observe with available hardware?
4. **Choose control architecture** — PID / LQR / SMC / MPC / gain-scheduled? Justify the choice.
5. **Address the hard parts** — nonlinearity, disturbances, noise, saturation, delays, uncertainty
6. **Validation strategy** — MIL → SIL → HIL → hardware testing

Do not skip upstream steps. Jumping to architecture selection (Step 4) without first understanding requirements, the plant, and C/O assessment leads to poor design decisions.

## Step 1 — Clarify Requirements

Ask the user to describe what the controller must achieve. Collect at minimum:

- **Controlled variable** — what output is being regulated?
- **Performance specs** — bandwidth, settling time, overshoot, steady-state accuracy
- **Constraints** — actuator limits (saturation), safety bounds, computational latency
- **Disturbances** — what external forces act on the system? Measurable or not?
- **Failure modes** — what happens when a sensor or actuator fails? Fallback strategy?

Do not proceed until requirements are sufficiently clear. If the user cannot specify performance numbers, help them derive reasonable targets from the application domain.

## Step 2 — Model the Plant

Guide the user through plant modelling. Key questions:

- **SISO or MIMO** — single-input/single-output vs multi-input/multi-output
- **Linear or nonlinear** — does superposition hold? Does gain change with operating point?
- **Inputs and outputs** — what actuator signals go in, what sensor signals come out?
- **States** — what internal variables describe the system's memory?
- **Dominant dynamics** — what order? Which modes matter for control?
- **Block diagram** — signal flow: plant, sensor, controller, actuator, disturbance paths

For deriving plant equations from physics:
- Read `knowledge/control-theory/first-principles-modelling.md` — Newton/Euler-Lagrange pipelines
- Read `knowledge/control-theory/state-space.md` — for state-space form conversion

If the user's plant resembles a known example, point them to:
- `knowledge/control-theory/examples/` — quarter-car, inverted pendulum, mass-spring-damper

## Step 3 — Assess Controllability and Observability

Before choosing a controller, verify that control is possible with the available hardware.

- **Controllability:** Can the available actuators drive all relevant states?
- **Observability:** Can the available sensors reconstruct the internal states? If not, an observer is needed.
- **What's missing?** If a state is uncontrollable or unobservable, the design must add hardware or accept the limitation.

Read `knowledge/control-theory/state-space.md` for the formal C/O rank tests.

## Step 4 — Choose Control Architecture

Read `knowledge/control-theory/controllers/index.md` to see the current controller catalogue. Read `knowledge/control-theory/controllers/comparison.md` for the comparison table and decision tree.

As of writing:

| Controller | Knowledge file | Reference header |
|---|---|---|
| PID | `knowledge/control-theory/controllers/pid.md` | `reference/controllers/pid.h` |
| LQR | `knowledge/control-theory/controllers/lqr.md` | `reference/controllers/lqr.h` |
| LQG | `knowledge/control-theory/controllers/lqg.md` | — |
| H-infinity | `knowledge/control-theory/controllers/h-infinity.md` | — |
| SMC | `knowledge/control-theory/controllers/sliding-mode.md` | `reference/controllers/smc.h` |
| Gain Scheduling | `knowledge/control-theory/controllers/gain-scheduling.md` | — |
| MPC | `knowledge/control-theory/controllers/mpc.md` | — |

Always read the index at runtime — new controllers may have been added.

### Decision Tree

Use the selection decision tree from `comparison.md`. Key branching questions:

1. **Is the plant linear?** If no → consider SMC or gain-scheduling.
2. **Are all states measurable?** If no → need an observer → LQG, or output feedback → H-infinity.
3. **Are there hard constraints on states/inputs?** If yes → MPC is the natural choice.
4. **Is robustness to model uncertainty the primary concern?** If yes → SMC or H-infinity.
5. **Is the system SISO or MIMO?** SISO with simple dynamics favours PID; MIMO favours LQR/LQG/MPC.

### Evaluate Candidates

For each candidate remaining after the decision tree, read its knowledge file and extract:

- **Applicability** — what plant types it suits
- **Prerequisites** — the `requires` field in YAML frontmatter
- **Trade-offs** — complexity, tuning difficulty, optimality guarantees, robustness margins
- **Related topics** — the `related` field pointing to supporting knowledge

Read prerequisite knowledge files as needed:
- `knowledge/control-theory/state-space.md` — if the controller requires a state-space model
- `knowledge/control-theory/stability.md` — for stability analysis prerequisites
- `knowledge/control-theory/frequency-response.md` — for frequency-domain design methods
- `knowledge/control-theory/observers/kalman-filter.md` — if LQG is a candidate

### Present the Recommendation

Structure as:

1. **Recommended controller** and why it fits this plant (grounded in Steps 1-3)
2. **Key trade-offs** vs the alternatives considered
3. **Prerequisites the user needs** — cite the `requires` frontmatter entries
4. **Reference implementation** — point to the `.h` file if one exists

### Nonlinear Systems

If the user's plant is nonlinear:

- **First consider SMC** — it handles nonlinear plants directly without linearization. Read `knowledge/control-theory/controllers/sliding-mode.md` for applicability.
- If SMC is not suitable (e.g., actuator bandwidth insufficient, unmatched disturbances dominate, optimality needed), discuss linearization and whether PID/LQR remains valid within the expected operating envelope.
- For plants with varying operating points: discuss gain-scheduled PID or gain-scheduled LQR.

**SMC selection criteria:**
- Recommend when: nonlinear plant + robustness primary + matched disturbances + sufficient actuator bandwidth
- Avoid when: well-modelled linear plant, limited actuator bandwidth, optimality is primary concern, unmatched disturbances

**LQG selection criteria:**
- Recommend when: linear plant + not all states measured + noise known + robustness margins not primary
- Avoid when: all states measurable (use LQR), must guarantee margins (use H-infinity), highly nonlinear

**H-infinity selection criteria:**
- Recommend when: significant bounded uncertainty + must guarantee performance + output feedback + robustness critical
- Avoid when: well-modelled plant (LQR simpler), no synthesis tools available, SISO with simple dynamics

## Step 5 — Address the Hard Parts

After selecting the architecture, proactively identify what will go wrong in practice. Walk through:

| Hard part | Questions to raise |
|---|---|
| Nonlinearity | Does the plant gain change across the operating envelope? Where does linearisation break down? |
| Disturbances | What external forces act on the system? Measurable (feedforward) or not? |
| Noise | How noisy are the sensors? Does the derivative term amplify noise? Filtering needed? |
| Actuator saturation | What happens when the controller output exceeds actuator limits? Anti-windup strategy? |
| Time delays | Transport delay in actuator or sensor path? Impact on stability margins? |
| Model uncertainty | How confident is the plant model? Unmodelled dynamics? Required gain/phase margin? |
| Mode switching | Multiple operating modes? Smooth transitions? Bumpless transfer? |

If the user's problem resembles a known design problem, point them to the worked examples:
- `knowledge/control-theory/design-problems/traction-control.md` — nonlinear tyre, surface estimation, split-mu
- `knowledge/control-theory/design-problems/active-suspension.md` — semi-active constraint, skyhook, mode switching
- `knowledge/control-theory/design-problems/hybrid-torque-split.md` — optimisation-based split, torque fill, thermal derating

## Step 6 — Validation Strategy

Help the user define a testing pipeline:

| Stage | Catches |
|---|---|
| **MIL** (Model-in-the-Loop) | Algorithm bugs, tuning errors, stability issues |
| **SIL** (Software-in-the-Loop) | Fixed-point effects, timing, code correctness |
| **HIL** (Hardware-in-the-Loop) | Actuator dynamics, communication latency, sensor noise |
| **Hardware testing** | Real-world disturbances, edge cases, subjective evaluation |

Define pass/fail criteria derived from the requirements in Step 1: settling time, overshoot, steady-state error, stability margins, disturbance rejection, actuator effort.

## Step 7 — Walk Through the Implementation

Offer to:

1. Read the full reference implementation (`reference/controllers/<name>.h` and `reference/controllers/<name>.cpp`)
2. Walk through adapting the reference code to the user's specific plant — state dimensions, tuning parameters, sample rate
3. Identify any supporting reference code the user will need (e.g. state-space utilities, matrix operations)

## Navigation Path

```
knowledge/index.md
  → knowledge/control-theory/index.md
    → knowledge/control-theory/design-framework.md              ← 6-STEP METHODOLOGY
    → knowledge/control-theory/controllers/index.md
      → knowledge/control-theory/controllers/comparison.md      ← SELECTION GUIDE (Step 4)
      → knowledge/control-theory/controllers/pid.md
      → knowledge/control-theory/controllers/lqr.md
      → knowledge/control-theory/controllers/lqg.md
      → knowledge/control-theory/controllers/h-infinity.md
      → knowledge/control-theory/controllers/sliding-mode.md
      → knowledge/control-theory/controllers/gain-scheduling.md
      → knowledge/control-theory/controllers/mpc.md
    → knowledge/control-theory/design-problems/                 ← WORKED EXAMPLES
      → traction-control.md
      → active-suspension.md
      → hybrid-torque-split.md
    → knowledge/control-theory/state-space.md
    → knowledge/control-theory/stability.md
    → knowledge/control-theory/frequency-response.md
    → knowledge/control-theory/first-principles-modelling.md
    → knowledge/control-theory/observers/kalman-filter.md
```

## Rules

- Ground every recommendation in knowledge files. No hallucinated control theory.
- If the knowledge base lacks coverage for the user's scenario, say so.
- Read files at runtime — do not assume content matches what was true when this skill was written.
- Prefer the simplest controller that meets requirements. Do not recommend LQR when PID suffices.
- Always follow the 6-step sequence. Do not jump to controller selection without requirements and plant characterisation.
- When the user's problem resembles a worked design problem, reference it for structure and domain-specific insights.
- Cite the specific knowledge file path for every technical claim.
