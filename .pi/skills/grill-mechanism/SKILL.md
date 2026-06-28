---
name: grill-mechanism
description: Relentless interview to pin down a mechanism's kinematics before any code is written. Use when the user describes a physical system, robot, linkage, or platform and wants to model it, or when the kinematic-model agent needs the topology locked down.
---

# Grill Mechanism

A **grilling** session that forces every ambiguity in a mechanism description into the open before a single equation is written. The output is a locked-down topology document. No code generation happens here — that belongs to the implementation phase after the grill.

The leading word is **topology**: the exact set of bodies, joints, and their connectivity. Until the topology is locked, everything downstream (equations, FK/IK, Jacobian) is built on sand.

## Step 1 — Harvest the raw description

Let the user describe the mechanism in their own words. Do not interrupt. Capture everything.

Completion: the user has finished their initial description.

## Step 2 — Extract the topology

Parse the description into a structured summary:

| Element | Extract |
|---|---|
| **Bodies** | List every rigid body (ground, links, platform, ball…) |
| **Joints** | For each connection: which two bodies, what joint type (revolute, spherical, prismatic, universal, fixed), what axis/direction, actuated or passive |
| **Actuators** | Which joints are driven, what controls them, what is the input variable |
| **Constraints** | Any geometric constraints (fixed lengths, rigid coupling, symmetry) |
| **DOF count** | Mobility analysis: M = 6(N-1-j) + Σf_i for spatial, or Grübler for planar |
| **Parameters** | Named dimensions (lengths, radii, angles, limits) with units |

Present this summary back to the user as a table.

Completion: the summary table is presented. Every body and joint from the description appears.

## Step 3 — Grill

For each element in the topology summary, challenge the user with targeted questions. Be **relentless** — do not accept vague answers. Specific angles of attack:

### Joint type ambiguity
- "You said this is a spherical joint. Does the leg actually rotate freely in all 3 axes, or is one axis locked by the geometry?"
- "Is the knee a revolute joint (1 DOF) or a universal joint (2 DOF)?"
- "You said the servo controls the angle. Which axis does it rotate about? Show me on your drawing."

### Coordinate frame ambiguity
- "Where is the origin? What direction is X? Is Z up or forward?"
- "When you say '45 degrees away from the circle' — is that in the horizontal plane or the vertical plane?"
- "What is the zero reference for this angle?"

### Constraint ambiguity
- "You said the table can roll, pitch, and heave. Can it also translate in X/Y or yaw? If not, what constrains those?"
- "When you say the legs 'can't move below 10 degrees' — is that a mechanical hard stop, a servo limit, or a collision?"

### Parameter ambiguity
- "Are these the final dimensions or placeholders? What are the real values?"
- "Are L1 and L2 the same length, or different?"

### Drawing exchange
If the mechanism is complex, ask the user to sketch it (Excalidraw, paper photo, or any image dropped into the project). Read the image and respond with your updated understanding. If you identify discrepancies between the drawing and the verbal description, flag them explicitly.

After each round of questions, update the topology summary and present it again. Repeat until no new ambiguities surface.

Completion: the user confirms "the topology is correct" or equivalent. No open questions remain.

## Step 4 — Lock the topology document

Write `docs/mechanism-topology.md` in the project with:

```markdown
# Mechanism Topology

## Bodies
(numbered list)

## Joint Table
| Joint | Body A | Body B | Type | Axis | Actuated | Variable |
|---|---|---|---|---|---|---|

## Parameters
| Symbol | Value | Units | Description |
|---|---|---|---|

## DOF Analysis
(Grübler/Kutzbach calculation)

## Coordinate Frame
(origin, axis directions, angle conventions)

## Constraints
(joint limits, coupling, symmetry)

## Drawings
(links to any images in docs/)
```

Completion: the document is written, all fields populated, and the user has reviewed it.

## Step 5 — Handoff

Print a summary of what was locked down and what the next implementation steps are (FK, IK, Jacobian, simulation). Do not implement — that is a separate task.

Completion: the user knows what was decided and what comes next.
