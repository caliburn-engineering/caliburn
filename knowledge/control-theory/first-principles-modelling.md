---
title: First-Principles Modelling
sources:
  - { book: "Kreyszig - Advanced Engineering Mathematics", chapter: "4" }
  - { book: "Rajamani — Vehicle Dynamics and Control", chapter: "6" }
  - { video: "Steve Brunton — Mechanical Systems and State-Space", url: "https://www.youtube.com/watch?v=O_1NNB1rqiU" }
  - { note: "engineering experience" }
requires:
  - state-space.md
  - ../math/linear-algebra/matrix-operations.md
related:
  - stability.md
  - controllers/lqr.md
  - examples/double-mass-spring-damper.md
  - examples/inverted-pendulum-cart.md
  - examples/quarter-car-model.md
  - examples/practice-problems.md
---

# First-Principles Modelling

First-principles modelling is the process of deriving a mathematical model of a physical system from fundamental physical laws (Newton's laws, conservation of energy). The pipeline goes: physical system -> free body diagram -> equations of motion -> state-space form. This is the step *before* controller design — without a correct plant model, no amount of LQR or PID tuning will save you.

## The Modelling Pipeline

```
Physical system
  -> Free Body Diagram (Newton) or Energy expressions (Euler-Lagrange)
  -> Coupled ODEs (one per DOF)
  -> (optional) Laplace transform -> transfer functions
  -> State-space form: x_dot = Ax + Bu
```

## Method A: Newtonian (Force-Based)

1. **Identify bodies and coordinates.** One generalised coordinate per DOF. Define positive direction.
2. **Draw Free Body Diagram for each body.** Show ALL forces: springs, dampers, external, gravity, reaction. Each force gets a direction.
3. **Apply Newton's 2nd law to each body.** F = ma (translational) or tau = I*alpha (rotational). Sum forces in the positive direction.
4. **Write coupled ODEs.** You get n equations for n DOF.
5. **Rearrange into standard form:** isolate highest-order derivatives on the left.
6. **Choose states and assemble state-space.**

**Best for:** simple translational systems, 1-2 DOF mass-spring-damper, systems where you want to "see" the forces.

## Method B: Euler-Lagrange (Energy-Based)

1. **Identify generalised coordinates** q_1, q_2, ..., q_n (one per DOF).
2. **Write kinetic energy T** in terms of q_dot_i (velocities).
3. **Write potential energy V** in terms of q_i (positions).
4. **Form the Lagrangian:** L = T - V
5. **Apply the Euler-Lagrange equation** for each coordinate:

```
d/dt(dL/dq_dot_i) - dL/dq_i = Q_i
```

where Q_i = generalised non-conservative forces (damping, external inputs).

6. **Result:** same ODEs as Newton, but derived from energy — often cleaner for complex/rotational systems.

**Best for:** rotational systems, coupled mechanisms, pendulums, robot arms — anything where constraint forces are hard to draw but energies are easy to write.

## The 3-Minute Derivation Checklist

Pin this mentally. When given ANY mechanical system in an interview:

1. **Count DOF** -- how many independent coordinates? That is the system order / 2.
2. **Name coordinates** -- draw them on the diagram with arrows showing positive direction.
3. **FBD each body** -- springs: force = k * (relative displacement). Dampers: force = c * (relative velocity). External forces. Sign convention: force positive in the positive coordinate direction.
4. **Sum(F) = ma for each body** -- one equation per DOF. Use *relative* displacement/velocity for coupling elements.
5. **Isolate x_ddot** -- divide by mass. Group terms by state variable.
6. **States = [positions, velocities]** -- write the trivial rows (x_dot = v) and the Newton rows (v_dot = x_ddot from step 5).
7. **Read off A and B** -- coefficients in front of each state -> A column. Input coefficient -> B column.

## State-Space Assembly Pattern

Given n second-order ODEs (n DOF), you get 2n states:

```
States: x = [q_1, q_dot_1, q_2, q_dot_2, ..., q_n, q_dot_n]^T

For each DOF i:
  Row 2i-1 (position):  always [0, ..., 0, 1, 0, ..., 0]  (the 1 at column 2i)
  Row 2i   (velocity):  coefficients from Newton/E-L for q_ddot_i, divided by mass

B vector: only non-zero at row 2i if force acts on mass i -> entry = 1/m_i
```

**The universal pattern for mechanical state-space:**
- Odd rows: always `[0, 1, 0, 0, ...]` -- position derivative equals velocity.
- Even rows: from Newton/E-L -- solve for acceleration, read off coefficients of each state.
- B column: only the mass that receives the input has a non-zero entry (1/m).
- This pattern scales to 3-DOF, 4-DOF, any size.

## Force/Moment Element Reference

| Element | Force/Moment | Sign Rule |
|---|---|---|
| Spring (translational) | F = k * (x_2 - x_1) | Opposes extension: pulls bodies together |
| Damper (translational) | F = c * (x_dot_2 - x_dot_1) | Opposes relative velocity |
| Spring (rotational) | tau = k_theta * (theta_2 - theta_1) | Same pattern, rotational |
| Damper (rotational) | tau = c_theta * (theta_dot_2 - theta_dot_1) | Same pattern |
| Gravity | F = -m * g (downward) | Acts on CG, constant |
| External force | F(t) or tau(t) | As defined by problem |

## Laplace Transform for Transfer Functions

After deriving the ODE, Laplace transform (zero ICs) converts to algebraic form:

| Time domain | s-domain |
|---|---|
| x(t) | X(s) |
| x_dot(t) | s * X(s) |
| x_ddot(t) | s^2 * X(s) |
| integral x(t) dt | X(s) / s |

**Process:** Take Laplace of each ODE -> solve for Output(s)/Input(s) = G(s). For multi-DOF, you get a matrix equation (impedance matrix) to invert.

**Laplace vs state-space -- when to use which:**
- **Laplace / transfer function:** Great for SISO analysis, frequency domain design, Bode/Nyquist. An interviewer asking for "the transfer function" wants this.
- **State-space:** Natural for MIMO, simulation, observer design, LQR. Required when you need all internal states. An interviewer asking for "a model" or "state-space" wants this.

## Common Laplace Pairs

| f(t) | F(s) | Notes |
|---|---|---|
| delta(t) (impulse) | 1 | |
| 1 (step) | 1/s | |
| t (ramp) | 1/s^2 | |
| e^(-at) | 1/(s+a) | First-order decay |
| sin(omega*t) | omega/(s^2+omega^2) | |
| cos(omega*t) | s/(s^2+omega^2) | |
| e^(-at)*sin(omega*t) | omega/((s+a)^2+omega^2) | Damped oscillation |

## Common Mistakes Under Pressure

1. **Using absolute displacement in coupling elements** -- always use *relative*: k * (x_2 - x_1)
2. **Same sign on coupling forces for both bodies** -- Newton's 3rd law: opposite signs on the two masses
3. **Forgetting to divide by mass in the A matrix** -- A entries = force coefficients / m
4. **Wrong sign on damping** -- damper always opposes relative motion
5. **Linearising before writing full equations** -- write full nonlinear first, linearise last
6. **Mixing up B_input and B_disturbance** -- separate them clearly (see quarter-car example)

## Worked Examples

| Example | Method | DOF | Link |
|---|---|---|---|
| Double mass-spring-damper | Newton + Laplace | 2 | [examples/double-mass-spring-damper.md](examples/double-mass-spring-damper.md) |
| Inverted pendulum on cart | Euler-Lagrange | 2 | [examples/inverted-pendulum-cart.md](examples/inverted-pendulum-cart.md) |
| Quarter-car model | Newton | 2 | [examples/quarter-car-model.md](examples/quarter-car-model.md) |

## Practice Problems

Five systems to derive on paper: [examples/practice-problems.md](examples/practice-problems.md)
