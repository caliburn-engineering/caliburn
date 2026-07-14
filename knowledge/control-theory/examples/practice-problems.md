---
title: "Practice Problems: First-Principles Modelling"
sources:
  - { note: "engineering experience" }
requires:
  - ../first-principles-modelling.md
  - ../state-space.md
related:
  - double-mass-spring-damper.md
  - inverted-pendulum-cart.md
  - quarter-car-model.md
---

# Practice Problems: First-Principles Modelling

Five systems to derive on paper using the 3-minute checklist. Work each one completely: FBD, Newton/E-L, isolate accelerations, state-space. Answers are provided below each problem.

---

## Problem 1: Single Mass-Spring-Damper with Base Excitation (1 DOF)

**System:** A mass m sits on a spring k and damper c. The base (ground) moves with displacement y(t). The mass displacement is x(t). No external force on the mass.

```
         ┌──────┐
         │  m   │  x(t)
         └──┬───┘
            │
       k /\/\  c ││
            │
    ════════╧════════  base: y(t) (prescribed motion)
```

**Task:** Derive the state-space model with input u = y(t). Find the transfer function X(s)/Y(s).

<details>
<summary>Solution</summary>

**Newton on mass m:**
```
m * x_ddot = -k * (x - y) - c * (x_dot - y_dot)
```

**Isolate acceleration:**
```
x_ddot = [-k*(x - y) - c*(x_dot - y_dot)] / m
       = (-k*x - c*x_dot + k*y + c*y_dot) / m
```

**States: x = [x, x_dot]^T, Input: u = y (but y_dot also appears)**

Since y_dot appears, we need to either treat [y, y_dot] as two inputs or use relative displacement. Using relative displacement z = x - y:

```
m * z_ddot = -k*z - c*z_dot - m*y_ddot
```

**State-space with z = x - y:**
```
States: [z, z_dot]^T, Input: w = y_ddot

A = |  0     1   |      B = | 0  |
    | -k/m  -c/m |          | -1 |
```

**Transfer function** (from the original equation, Laplace with zero ICs):
```
m*s^2*X = -k*(X-Y) - c*s*(X-Y)
(m*s^2 + c*s + k)*X = (c*s + k)*Y

X(s)/Y(s) = (c*s + k) / (m*s^2 + c*s + k)
```

This is a low-pass filter with a zero at s = -k/c. At low frequencies, X/Y -> 1 (mass follows base). At high frequencies, X/Y -> 0 (mass is isolated).

</details>

---

## Problem 2: Triple Mass-Spring Chain (3 DOF, No Dampers)

**System:** Three masses m_1, m_2, m_3 connected in a chain by springs k_1, k_2, k_3. The left end of k_1 is fixed to a wall. No damping, no external forces.

```
         k_1          k_2          k_3
┌────┐ /\/\/\ ┌────┐ /\/\/\ ┌────┐ /\/\/\ ┌────┐
│Wall│────────│ m1 │────────│ m2 │────────│ m3 │
└────┘        └──┬─┘        └──┬─┘        └──┬─┘
                 x_1           x_2           x_3
```

**Task:** Write the 6x6 A matrix. Verify the tri-diagonal coupling pattern.

<details>
<summary>Solution</summary>

**Newton for each mass:**
```
m_1 * x_ddot_1 = -k_1*x_1 + k_2*(x_2 - x_1)
               = -(k_1+k_2)*x_1 + k_2*x_2

m_2 * x_ddot_2 = -k_2*(x_2 - x_1) + k_3*(x_3 - x_2)
               = k_2*x_1 - (k_2+k_3)*x_2 + k_3*x_3

m_3 * x_ddot_3 = -k_3*(x_3 - x_2)
               = k_3*x_2 - k_3*x_3
```

**States: [x_1, v_1, x_2, v_2, x_3, v_3]^T**

```
A = |  0              1              0            0            0           0         |
    | -(k1+k2)/m1    0              k2/m1        0            0           0         |
    |  0              0              0            1            0           0         |
    |  k2/m2         0             -(k2+k3)/m2   0            k3/m2      0         |
    |  0              0              0            0            0           1         |
    |  0              0              k3/m3        0           -k3/m3      0         |
```

**Tri-diagonal pattern:** The stiffness coupling forms a block tri-diagonal matrix. Mass i couples only to masses i-1 and i+1. No damping means no velocity coupling (all velocity columns in even rows are zero).

B = [0, 0, 0, 0, 0, 0]^T (no external forces in this problem).

</details>

---

## Problem 3: Rotational System — Motor-Shaft-Load (2 DOF)

**System:** Motor with inertia J_1 drives a load with inertia J_2 through a flexible shaft with stiffness k_theta and damping c_theta. Motor torque tau is the input.

```
      tau ->
    ┌──────┐     k_theta, c_theta     ┌──────┐
    │  J_1 │═══════════════════════════│  J_2 │
    │motor │       flexible shaft      │ load │
    └──────┘                           └──────┘
      theta_1                           theta_2
```

**Task:** Derive the state-space. Note: identical pattern to translational, just m -> J, x -> theta, F -> tau.

<details>
<summary>Solution</summary>

**Newton (rotational) for motor J_1:**
```
J_1 * theta_ddot_1 = tau - k_theta*(theta_1 - theta_2) - c_theta*(theta_dot_1 - theta_dot_2)
```

**Newton for load J_2:**
```
J_2 * theta_ddot_2 = k_theta*(theta_1 - theta_2) + c_theta*(theta_dot_1 - theta_dot_2)
```

**Isolate accelerations:**
```
theta_ddot_1 = [tau - k_theta*(theta_1-theta_2) - c_theta*(theta_dot_1-theta_dot_2)] / J_1

theta_ddot_2 = [k_theta*(theta_1-theta_2) + c_theta*(theta_dot_1-theta_dot_2)] / J_2
```

**States: [theta_1, theta_dot_1, theta_2, theta_dot_2]^T, Input: u = tau**

```
A = |  0              1              0             0           |
    | -k_theta/J_1   -c_theta/J_1   k_theta/J_1   c_theta/J_1 |
    |  0              0              0             1           |
    |  k_theta/J_2    c_theta/J_2   -k_theta/J_2  -c_theta/J_2 |

B = [0, 1/J_1, 0, 0]^T
```

**Pattern check:** Structurally identical to the double mass-spring-damper. Replace m with J, k with k_theta, c with c_theta, x with theta, F with tau. The pattern is universal for two bodies coupled by spring + damper.

The drivetrain shuffle (torsional oscillation during sudden torque application) is described by the natural frequency of this system: omega_n = sqrt(k_theta * (1/J_1 + 1/J_2)).

</details>

---

## Problem 4: DC Motor (2 First-Order ODEs)

**System:** A DC motor has two coupled domains: electrical (voltage drives current through resistance and inductance) and mechanical (current produces torque that drives inertia against friction). The back-EMF couples mechanical back to electrical.

```
Electrical:  V = R*i + L*di/dt + K_e*omega
Mechanical:  J*d(omega)/dt = K_t*i - b*omega - tau_load
```

Where:
- V = supply voltage (input)
- i = armature current
- omega = shaft speed
- R, L = armature resistance, inductance
- K_e = back-EMF constant
- K_t = torque constant (= K_e for ideal motor)
- J = rotor inertia
- b = viscous friction
- tau_load = load torque (disturbance)

**Task:** Write the state-space with states [i, omega], input V, and disturbance tau_load.

<details>
<summary>Solution</summary>

**Already in first-order form** (no need for position-velocity decomposition):

From the electrical equation, isolate di/dt:
```
di/dt = (V - R*i - K_e*omega) / L = -(R/L)*i - (K_e/L)*omega + (1/L)*V
```

From the mechanical equation, isolate d(omega)/dt:
```
d(omega)/dt = (K_t*i - b*omega - tau_load) / J = (K_t/J)*i - (b/J)*omega - (1/J)*tau_load
```

**States: [i, omega]^T, Control input: u = V, Disturbance: w = tau_load**

```
A = | -R/L     -K_e/L |      B_u = | 1/L |      B_w = |  0   |
    |  K_t/J   -b/J   |            |  0  |            | -1/J |
```

**Key insight:** This is NOT a second-order system. It is two coupled first-order ODEs from different physical domains (electrical and mechanical). The states are [current, speed], not [position, velocity]. No trivial rows in the A matrix.

**Transfer function** (V to omega, zero tau_load):
```
omega(s)/V(s) = K_t / [L*J*s^2 + (R*J + b*L)*s + (R*b + K_e*K_t)]
```

This is a second-order transfer function with two real poles (typically over-damped for small motors).

**Numerical example:** R=1 Ohm, L=0.01 H, K_e=K_t=0.01 N*m/A, J=0.001 kg*m^2, b=0.001 N*m*s:
```
A = | -100   -1   |      B = | 100 |
    |  10    -1   |          |  0  |
```

Electrical time constant tau_e = L/R = 0.01 s. Mechanical time constant tau_m = J/b = 1 s. The electrical dynamics are 100x faster, so for slow control, the electrical equation can be approximated as algebraic (quasi-static): i = (V - K_e*omega)/R.

</details>

---

## Problem 5: Simple Pendulum (Euler-Lagrange, then Linearise)

**System:** A point mass m on a rigid massless rod of length L, pivoting about a fixed point. No damping. Under gravity g. Angle theta measured from vertical downward (theta = 0 = hanging equilibrium).

```
         O (pivot, fixed)
         │
         │ L
         │
         ● m   (theta measured from vertical down)
```

**Task:** Use the Euler-Lagrange method to derive the equation of motion. Then linearise around theta = 0. Write the state-space.

<details>
<summary>Solution</summary>

**Coordinates:** q = theta (angle from vertical down). DOF = 1.

**Kinetic energy:**
```
T = (1/2) * m * L^2 * theta_dot^2
```

**Potential energy** (zero at pivot level):
```
V = -m * g * L * cos(theta)
```
(Negative because mass is below pivot when theta = 0.)

**Lagrangian:**
```
L = T - V = (1/2)*m*L^2*theta_dot^2 + m*g*L*cos(theta)
```

**Euler-Lagrange:**
```
dL/d(theta_dot) = m*L^2*theta_dot
d/dt[dL/d(theta_dot)] = m*L^2*theta_ddot

dL/d(theta) = -m*g*L*sin(theta)

E-L: m*L^2*theta_ddot + m*g*L*sin(theta) = 0
```

Simplify:
```
theta_ddot = -(g/L) * sin(theta)
```

This is the classic nonlinear pendulum equation.

**Linearise around theta = 0:** sin(theta) ~ theta

```
theta_ddot = -(g/L) * theta
```

**State-space:** States: [theta, theta_dot]^T

```
A = |  0      1   |      B = | 0 |     (no input in free pendulum)
    | -g/L    0   |          | 0 |
```

**Eigenvalues:** lambda = +/- j*sqrt(g/L). Purely imaginary — marginally stable. The linearised pendulum oscillates forever (no damping). Adding a torque input tau at the pivot:

```
B_tau = [0, 1/(m*L^2)]^T
```

**Natural frequency:** omega_n = sqrt(g/L), period T = 2*pi*sqrt(L/g).

For L = 1 m: omega_n = 3.13 rad/s, T = 2.01 s.

</details>

---

## Self-Assessment

After working through all five problems, check:

1. Can you write the FBD and Newton equations within 3 minutes for a 2-DOF system?
2. Do you instinctively use relative displacement for coupling elements?
3. Do coupling forces have opposite signs on the two bodies?
4. Can you identify the state-space assembly pattern (trivial rows + Newton rows) immediately?
5. Can you switch between Newton and Euler-Lagrange depending on the system type?

If you can do 3 different systems cold in under 5 minutes each, you are solid for any interview.
