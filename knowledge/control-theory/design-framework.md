---
title: Control Design Framework — 6-Step Methodology
sources:
  - { note: "Mercedes interview prep — Lesson 5 applied design problems" }
  - { note: "engineering experience — structured design process" }
requires:
  - state-space.md
  - stability.md
related:
  - controllers/comparison.md
  - first-principles-modelling.md
  - frequency-response.md
  - design-problems/traction-control.md
  - design-problems/active-suspension.md
  - design-problems/hybrid-torque-split.md
---

# Control Design Framework — 6-Step Methodology

A structured methodology for approaching any "design a controller for X" problem. This framework works for interview design questions, greenfield production systems, and everything in between. The steps are sequential — skipping upstream steps (requirements, modelling) leads to wasted design effort downstream.

## The Framework

### Step 1 — Clarify Requirements

Before touching any math or architecture, nail down what the controller must achieve.

| Question | Why it matters |
|---|---|
| What is the controlled variable? | Defines the output feedback signal |
| What are the performance specs? | Bandwidth, settling time, overshoot, steady-state accuracy — these constrain the design |
| What constraints exist? | Actuator limits (saturation), safety bounds, computational latency, communication delays |
| What disturbances act on the system? | Determines rejection requirements and feedforward opportunities |
| What are the failure modes? | Drives fallback strategy and redundancy requirements |

**Anti-pattern:** Jumping straight to "I'd use PID" without understanding requirements. The controller choice should follow from the problem, not precede it.

### Step 2 — Model the Plant

Build a dynamic model of the system to be controlled.

| Aspect | Detail |
|---|---|
| Inputs and outputs | What actuator signals go in? What sensor signals come out? |
| States | What internal variables describe the system's memory? |
| Linear or nonlinear? | Does superposition hold? Are there gain variations across operating points? |
| Order | How many energy-storing elements? What is the system order? |
| Dominant dynamics | Which modes matter for control? Which can be neglected? |
| Block diagram | Sketch the signal flow — plant, sensor, controller, actuator, disturbance paths |

Use [first-principles modelling](first-principles-modelling.md) (Newton or Euler-Lagrange) to derive the plant equations. Convert to [state-space form](state-space.md) for multi-state systems.

### Step 3 — Assess Controllability and Observability

Before choosing a controller, verify that control is even possible with the available hardware.

**Controllability:** Can the available actuators drive all relevant states? Check the controllability matrix rank for linear systems. For nonlinear systems, argue from physics — which states does each actuator influence?

**Observability:** Can the available sensors reconstruct the internal states? If not, an [observer](observers/index.md) is needed, or additional sensors must be specified.

**What's missing?** If a state is uncontrollable or unobservable, the design must either add hardware (more sensors/actuators) or accept the limitation and design around it.

See [state-space representation](state-space.md) for the formal controllability and observability rank tests.

### Step 4 — Choose Control Architecture

Select the controller type based on the plant characteristics identified in Steps 1-3. Use the [controller selection guide](controllers/comparison.md) and decision tree.

Key drivers for the architecture choice:

| Plant characteristic | Points toward |
|---|---|
| Linear, SISO, simple dynamics | PID — simplest, widely understood |
| Linear, MIMO, full state access | LQR — optimal, per-state tuning via Q/R |
| Linear, not all states measured | LQG — LQR + Kalman observer |
| Nonlinear, robustness-critical | SMC — robust to matched uncertainty |
| Hard constraints on states/inputs | MPC — handles constraints natively |
| Operating point varies significantly | Gain scheduling — interpolated linear designs |
| Uncertain plant, must guarantee performance | H-infinity — worst-case robust design |

**Justify the choice.** State which specific aspect of the problem drives the architecture selection. "I'd use MPC because the actuator has hard saturation limits that a linear controller can't respect" is a strong justification. "I'd use MPC because it's the most advanced" is not.

**Simplicity heuristic:** Always prefer the simplest controller that meets requirements. Complexity costs implementation effort, debugging difficulty, more failure modes, and harder certification.

### Step 5 — Address the Hard Parts

Every real system has complications that the textbook design ignores. Identify them proactively.

| Hard part | Questions to ask |
|---|---|
| Nonlinearity | Does the plant gain change across the operating envelope? Where does linearisation break down? |
| Disturbances | What external forces act on the system? Are they measurable (feedforward) or not (feedback only)? |
| Noise | How noisy are the sensors? Does the derivative term amplify noise? Is filtering needed? |
| Actuator saturation | What happens when the controller asks for more than the actuator can deliver? Anti-windup? |
| Time delays | Is there transport delay in the actuator or sensor path? How does this affect stability margins? |
| Model uncertainty | How confident is the plant model? What are the unmodelled dynamics? How much gain/phase margin is needed? |
| Mode switching | Does the system operate in multiple modes? Are transitions smooth? |

This step separates textbook design from production-ready engineering. A controller that works in simulation but ignores saturation, noise, and delay will fail on hardware.

### Step 6 — Validation Strategy

Define the testing pipeline before implementation. Each stage catches different classes of problems.

| Stage | What it covers | Key checks |
|---|---|---|
| **MIL** (Model-in-the-Loop) | Algorithm correctness in idealised simulation | Stability, tracking performance, disturbance rejection |
| **SIL** (Software-in-the-Loop) | Production code running against plant model | Fixed-point effects, timing, code correctness |
| **HIL** (Hardware-in-the-Loop) | Real ECU/actuator against simulated plant | Actuator dynamics, communication latency, sensor noise |
| **Vehicle/hardware testing** | Full system on real hardware | Real-world disturbances, edge cases, subjective evaluation |

At each stage, define pass/fail criteria derived from the requirements in Step 1. Track: settling time, overshoot, steady-state error, stability margins, disturbance rejection, actuator effort.

## Using the Framework

### For Interview Design Questions

Walk through all six steps sequentially. Spend most time on Steps 2 (modelling) and 5 (hard parts) — these demonstrate engineering depth. The framework itself is the answer; interviewers evaluate structured thinking, not whether you arrive at the perfect controller.

### For Production Design

Steps 1-3 happen in the concept phase. Step 4 is the architecture decision (often reviewed by the team). Step 5 drives the detailed design and testing effort. Step 6 defines the validation plan that gates release.

### Worked Examples

Three automotive design problems applying this framework end-to-end:

- [Traction Control System](design-problems/traction-control.md) — nonlinear tyre curve, surface estimation, gain-scheduled/SMC architecture
- [Active Suspension Damping](design-problems/active-suspension.md) — semi-active constraint, skyhook damping, mode switching
- [Hybrid Torque Split Controller](design-problems/hybrid-torque-split.md) — optimisation-based split, ICE/motor dynamics, SoC management

## Key Vocabulary

| Term | One-liner |
|---|---|
| Controlled variable | The system output that the controller regulates to a setpoint |
| Plant | The physical system being controlled — everything between actuator output and sensor input |
| Architecture | The controller structure — PID, state feedback, MPC, etc. |
| MIL / SIL / HIL | Progressively more realistic test stages — model, software, hardware in the loop |
| Matched disturbance | A disturbance entering through the same channel as the control input — SMC rejects these |
| Passivity constraint | Semi-active actuators (e.g., variable dampers) can only dissipate energy, not inject it |
| Torque fill | Electric motor compensating for ICE response lag during transients |
| Derating | Reducing actuator output to stay within thermal or safety limits |
