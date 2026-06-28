---
name: validate-mechanism
description: Run standard kinematic validation on a mechanism — FK/IK round-trip, workspace sweep, singularity search, trajectory feasibility. Use when the user has a working kinematic model and wants to verify it, find its limits, or stress-test trajectories.
---

# Validate Mechanism

Systematic verification of a kinematic model. Requires that FK, IK, and the velocity Jacobian are already implemented. Each analysis produces concrete numbers the user can check against physical intuition.

## Step 1 — Identify the interface

Read the project's kinematics code. Locate:

1. `forward_kinematics(inputs) → pose` — the FK solver
2. `inverse_kinematics(pose) → inputs` — the IK solver
3. `velocity_jacobian(inputs, pose) → matrix` — the Jacobian
4. Input limits (joint/servo min/max)
5. Home configuration (default inputs)

Completion: you can call all five from a test program.

## Step 2 — FK/IK round-trip

For a set of test configurations spanning the input space:

```
inputs → FK → pose → IK → recovered_inputs
error = |recovered - original|
```

Test at minimum:
- Home configuration
- Each input at its minimum, others at home
- Each input at its maximum, others at home
- An asymmetric configuration
- An extreme configuration (all inputs at different values)

Completion: max round-trip error reported. Must be < 1e-6 for passing configurations.

## Step 3 — Workspace sweep

Sweep the input space and compute FK for each configuration. Record:
- Feasibility (FK converges)
- Output pose (roll, pitch, z — or whatever the DOF are)
- Condition number

Report:
- Total configurations tested
- Feasible percentage
- Output ranges (min/max for each DOF)
- Best-conditioned region (input ranges where condition < 10)

Export to CSV: `build/workspace.csv`

Completion: CSV exported with at least 1000 data points. Feasibility boundaries identified.

## Step 4 — Singularity search

Sweep the input space and record `det(J_pose)` and `det(J_alpha)`.

Report:
- Number of near-singular configurations (`|det| < 1e-6`)
- Worst-case configuration for each singularity type
- Whether any singularities fall within the normal operating range

The operating range is defined by the user or inferred from the application (e.g., for a ball-balancer: < 10° tilt).

Completion: singularity map reported. Operating range clearance confirmed or warnings issued.

## Step 5 — Trajectory validation

Define at least 4 test trajectories:

| Trajectory | Purpose |
|---|---|
| Gentle periodic motion within operating range | Confirm nominal operation |
| Aggressive periodic motion at operating limits | Find velocity/conditioning issues |
| Step change | Confirm velocity limit violation detection |
| Ramp to workspace boundary | Find where feasibility breaks |

For each, validate:
- IK feasibility at every timestep
- Servo/joint velocity within limits (finite difference)
- Condition number below threshold

Report a summary per trajectory: total points, feasible count, velocity violations, first failure point with details.

Completion: all trajectories validated. Summary printed with pass/fail per trajectory.

## Step 6 — Report

Compile findings into `docs/validation-report.md`:

```markdown
# Kinematic Validation Report

## FK/IK Round-Trip
(table of test configs and errors)

## Workspace
(feasible ranges, best-conditioned region, CSV reference)

## Singularities
(type-1 and type-2 findings, operating range clearance)

## Trajectory Validation
(summary per trajectory)

## Recommendations
(operating envelope, configurations to avoid, controller constraints)
```

Completion: report written and committed.
