---
name: validate-implementation
description: Compare a user's implementation against the golden reference and report correctness, stability, and missing edge cases
---

# Validate Implementation

Compare the user's code against the corresponding Caliburn reference implementation. Report what is correct, what diverges, and what is missing. Do not rewrite their code — help them understand the differences and fix it themselves.

## Step 1 — Identify the Algorithm

Ask the user which algorithm they want to validate. Accept any of:

- A file path (e.g. `projects/ball-balancer/src/pid.cpp`) — infer the algorithm from the filename and contents
- A name (e.g. "my PID controller", "Kalman filter", "RK4 integrator")

Map the answer to one of the reference categories:

| Algorithm | Reference directory |
|---|---|
| PID controller | `reference/controllers/pid.h`, `pid.cpp` |
| LQR controller | `reference/controllers/lqr.h`, `lqr.cpp` |
| Kalman filter | `reference/observers/kalman_filter.h`, `kalman_filter.cpp` |
| RK4 integrator | `reference/integrators/rk4.h`, `rk4.cpp` |
| Simulation loop | `reference/simulation/sim_loop.h`, `sim_loop.cpp` |
| Fixed timestep | `reference/simulation/fixed_timestep.h`, `fixed_timestep.cpp` |

If the algorithm does not match any reference, say so and stop. Do not validate against nothing.

## Step 2 — Read the User's Code

Read the file(s) the user points to from their project under `projects/<project-name>/`. If they give a single file, read it. If they have a header/source split, read both.

## Step 3 — Read the Reference Implementation

Read both the reference header (`.h`) and source (`.cpp`) for the matched algorithm. These are the golden source — correct algorithms with tested edge cases.

## Step 4 — Read the Reference Tests

Read the corresponding test file to understand what correctness checks and edge cases the reference covers:

| Algorithm | Test file |
|---|---|
| PID | `reference/controllers/test_pid.cpp` |
| LQR | `reference/controllers/test_lqr.cpp` |
| Kalman filter | `reference/observers/test_kalman.cpp` |
| RK4 | `reference/integrators/test_rk4.cpp` |
| Simulation loop | `reference/simulation/test_sim_loop.cpp` |
| Fixed timestep | `reference/simulation/test_fixed_timestep.cpp` |

Note: `reference/rendering/` has no test files — UI code is not validated by this skill.

Extract from the tests:

- What inputs and initial conditions are tested
- What edge cases are covered (zero gains, singular matrices, zero timestep, etc.)
- What numerical tolerances are used
- What failure modes are explicitly guarded against

## Step 5 — Read the Knowledge File

Find the knowledge file that links to this reference implementation via its `reference` frontmatter field. Use this navigation:

| Algorithm | Knowledge file |
|---|---|
| PID | `knowledge/control-theory/controllers/pid.md` |
| LQR | `knowledge/control-theory/controllers/lqr.md` |
| Kalman filter | `knowledge/control-theory/observers/kalman-filter.md` |
| RK4 | `knowledge/simulation/integration/` (check index) |
| Simulation loop | `knowledge/simulation/` (check index) |
| Fixed timestep | `knowledge/simulation/` (check index) |

Read the knowledge file for theoretical grounding. Pay attention to:

- Key equations — does the user's math match?
- Stability conditions and constraints documented in the knowledge
- The `requires` frontmatter — are there prerequisite concepts the user might be missing?

## Step 6 — Compare and Analyse

Perform a structured comparison. Check each of these categories:

### Algorithmic Correctness

- Does the user's implementation match the mathematical formulation in the knowledge file?
- Are the core equations implemented correctly (update laws, gain computation, matrix operations)?
- Is the execution order correct (e.g. predict before update in Kalman, derivative before integral in PID)?

### Numerical Stability

Algorithm-specific checks:

- **PID:** Anti-windup clamping on the integral term. Derivative filtering. Output saturation.
- **LQR:** Riccati equation convergence. Controllability check before computing gains.
- **Kalman filter:** Covariance symmetry enforcement (P = (P + P') / 2). Positive-definite checks. Innovation covariance inversion stability.
- **RK4:** Timestep validation. State vector dimension consistency across k1-k4 stages.
- **Simulation loop / Fixed timestep:** Accumulator drift handling. Timestep bounds.

### Missing Edge Cases

Cross-reference the user's code against the test file from Step 4. For each edge case the reference tests cover, check whether the user handles it. Common gaps:

- Zero or negative timestep
- Zero gains / zero noise covariance
- First-call initialization (no prior state)
- Dimension mismatches in matrix operations
- Saturation / clamping bounds

### Eigen Usage Patterns

If the user uses Eigen:

- Proper matrix sizing (fixed-size vs dynamic — does it match the reference?)
- Avoiding aliasing issues (e.g. `A = A * B` without `.eval()`)
- Using `.noalias()` where the reference does
- Correct use of solvers vs manual inversion (e.g. `ldlt().solve()` instead of `.inverse()`)

## Step 7 — Report Findings

Structure the report as:

### Correct

What the user got right. Be specific — cite the lines or patterns that match the reference.

### Potential Issues

Concrete problems found. For each issue:
- What the user's code does
- What the reference does differently and why
- The consequence if left unfixed (numerical drift, instability, incorrect output)

### Missing Edge Cases

Edge cases from the reference tests that the user's code does not handle. For each:
- The specific test case from the reference
- Why it matters (what breaks without it)

### Suggestions

Improvements that are not bugs but would bring the implementation closer to production quality. Keep this short — only include suggestions grounded in the reference or knowledge files.

## Rules

- The reference implementations are the authority. Compare against them, not against general knowledge.
- Do NOT paste the reference code into the report. Describe the differences conceptually.
- Do NOT offer to rewrite the user's code. Help them understand, then let them fix it.
- If the user's approach is valid but different from the reference (e.g. a different anti-windup strategy), acknowledge it as a legitimate alternative and note any trade-offs.
- Read files at runtime — do not assume content matches what was true when this skill was written.
- If a reference file is missing or has changed structure, say so explicitly.
