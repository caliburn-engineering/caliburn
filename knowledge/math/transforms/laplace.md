---
title: Laplace Transform
sources:
  - { book: "Kreyszig - Advanced Engineering Mathematics", chapter: "6" }
  - { book: "Ogata - Modern Control Engineering", chapter: "2" }
  - { note: "engineering experience" }
requires:
  - ../linear-algebra/matrix-operations.md
related:
  - ../../control-theory/frequency-response.md
  - ../../control-theory/state-space.md
  - ../../control-theory/first-principles-modelling.md
  - ../../control-theory/stability.md
---

# Laplace Transform

The Laplace transform converts a function of time f(t) into a function of complex frequency F(s), where s = sigma + j*omega. Its primary value in control engineering is converting differential equations into algebraic equations -- you replace derivatives with powers of s, solve with algebra, then (optionally) invert back to the time domain.

For control design, the one-sided Laplace transform (integration from 0 to infinity) is standard. It naturally handles initial conditions and causal systems.

## Definition

```
F(s) = L{f(t)} = integral from 0 to infinity of f(t) * e^(-s*t) dt
```

The inverse:

```
f(t) = L^{-1}{F(s)} = (1 / 2*pi*j) * integral (sigma - j*inf to sigma + j*inf) F(s) * e^(s*t) ds
```

In practice, the inverse is almost never computed from the integral. Instead, use partial fraction decomposition + the transform pairs table below.

## Transform Pairs Table

| f(t) | F(s) | Notes |
|---|---|---|
| delta(t) (impulse) | 1 | Unit impulse |
| 1 (step) | 1/s | Unit step function |
| t (ramp) | 1/s^2 | |
| t^n | n!/s^(n+1) | General power |
| e^(-a*t) | 1/(s+a) | First-order decay |
| sin(omega*t) | omega/(s^2+omega^2) | |
| cos(omega*t) | s/(s^2+omega^2) | |
| e^(-a*t)*sin(omega*t) | omega/((s+a)^2+omega^2) | Damped oscillation |
| e^(-a*t)*cos(omega*t) | (s+a)/((s+a)^2+omega^2) | Damped oscillation |
| t*e^(-a*t) | 1/(s+a)^2 | |
| t^n*e^(-a*t) | n!/(s+a)^(n+1) | |

## Properties Table

| Property | Time domain | s-domain | Condition |
|---|---|---|---|
| Linearity | a*f(t) + b*g(t) | a*F(s) + b*G(s) | |
| Differentiation (1st) | f_dot(t) | s*F(s) - f(0) | |
| Differentiation (2nd) | f_ddot(t) | s^2*F(s) - s*f(0) - f'(0) | |
| Differentiation (nth) | f^(n)(t) | s^n*F(s) - s^(n-1)*f(0) - ... - f^(n-1)(0) | |
| Integration | integral_0^t f(tau) d(tau) | F(s)/s | |
| Time delay | f(t-T)*u(t-T) | e^(-s*T)*F(s) | T > 0, u = step |
| Frequency shift | e^(-a*t)*f(t) | F(s+a) | s-domain shift |
| Time scaling | f(a*t) | (1/a)*F(s/a) | a > 0 |
| Convolution | integral_0^t f(tau)*g(t-tau) d(tau) | F(s)*G(s) | Multiplication in s = convolution in t |
| Final Value Theorem | lim(t->inf) f(t) | lim(s->0) s*F(s) | All poles of s*F(s) in LHP |
| Initial Value Theorem | f(0+) | lim(s->inf) s*F(s) | F(s) is strictly proper |

## Transfer Function Derivation (Zero-IC Shortcut)

For modelling physical systems, initial conditions are always set to zero. This is the standard assumption for transfer function derivation -- the transfer function describes the system's input-output behavior, not its response to initial conditions.

**With zero ICs, the differentiation property simplifies to:**

| Time domain | s-domain (zero ICs) |
|---|---|
| x(t) | X(s) |
| x_dot(t) | s*X(s) |
| x_ddot(t) | s^2*X(s) |
| x^(n)(t) | s^n*X(s) |

This is the form used 99% of the time in control engineering. The full form with initial conditions matters only for transient response analysis.

### The Pipeline

```
Physical system
  -> Newton / Euler-Lagrange -> coupled ODEs
  -> Laplace transform (zero ICs) -> algebraic equations in s
  -> Solve for Output(s)/Input(s) = G(s)
```

### SISO Example

Given a mass-spring-damper: m*x_ddot + c*x_dot + k*x = F(t)

Take Laplace (zero ICs):

```
m*s^2*X(s) + c*s*X(s) + k*X(s) = F(s)
(m*s^2 + c*s + k)*X(s) = F(s)
G(s) = X(s)/F(s) = 1 / (m*s^2 + c*s + k)
```

### Multi-DOF: Impedance Matrix Approach

For n-DOF systems, Laplace of each Newton equation yields n algebraic equations. Stack them into a matrix equation:

```
Z(s) * X(s) = F(s)
```

where Z(s) is the n x n impedance matrix (each entry is a polynomial in s), X(s) is the vector of displacements, and F(s) is the forcing vector.

Transfer functions are obtained by inverting Z(s):

```
X(s) = Z(s)^{-1} * F(s)
```

For a 2-DOF system, use Cramer's rule to avoid full matrix inversion. See the [double mass-spring-damper example](../../control-theory/examples/double-mass-spring-damper.md) for the complete derivation.

## Connection to State-Space

The transfer function and state-space representations are related by:

```
G(s) = C * (s*I - A)^{-1} * B + D
```

This follows directly from taking the Laplace transform of x_dot = Ax + Bu, y = Cx + Du with zero ICs:

```
s*X(s) = A*X(s) + B*U(s)
(s*I - A)*X(s) = B*U(s)
X(s) = (s*I - A)^{-1} * B * U(s)
Y(s) = [C*(s*I - A)^{-1}*B + D] * U(s)
```

**Key insight:** The poles of G(s) are the eigenvalues of A (roots of det(s*I - A) = 0). The denominator polynomial of the transfer function is the characteristic polynomial of A. This means the natural frequencies from Laplace analysis are identical to the eigenvalues from state-space analysis -- two views of the same physics.

## When to Use Laplace vs State-Space

| Criterion | Laplace / Transfer Function | State-Space |
|---|---|---|
| System type | SISO (one input, one output) | MIMO (multiple inputs/outputs) |
| Analysis domain | Frequency domain (Bode, Nyquist) | Time domain (simulation, LQR) |
| Design method | Classical (lead/lag, root locus) | Modern (pole placement, LQR, Kalman) |
| Internal states | Not visible (only input-output) | All states accessible |
| Observer design | Not natural | Required formulation |
| Interview cue | "Find the transfer function" | "Write the state-space model" |
| Nonlinear systems | Not applicable | Can represent (before linearization) |

**Practical rule:** Use Laplace to derive the transfer function when you need frequency-domain insight (Bode plots, stability margins, resonance peaks). Use state-space when you need to simulate, design observers, or handle MIMO systems. For most real systems, you derive both and use each where it is strongest.

## Partial Fraction Decomposition

To invert a transfer function back to the time domain, decompose F(s) into simpler fractions that map to the pairs table.

### Distinct Real Poles

```
F(s) = (2s + 1) / [(s+1)(s+3)]
     = A/(s+1) + B/(s+3)

A = (2*(-1) + 1) / ((-1)+3) = -1/2
B = (2*(-3) + 1) / ((-3)+1) = 5/2

f(t) = -0.5*e^(-t) + 2.5*e^(-3t)
```

### Repeated Poles

```
F(s) = 1/(s+a)^2  ->  f(t) = t*e^(-a*t)
F(s) = 1/(s+a)^3  ->  f(t) = (t^2/2)*e^(-a*t)
```

### Complex Conjugate Poles

Complete the square to match the damped sinusoid pairs:

```
F(s) = 1/(s^2 + 2*s + 5) = 1/((s+1)^2 + 4) = (1/2) * 2/((s+1)^2 + 2^2)

f(t) = (1/2)*e^(-t)*sin(2t)
```

## Final Value and Initial Value Theorems

### Final Value Theorem

```
lim(t->inf) f(t) = lim(s->0) s*F(s)
```

**Prerequisite:** All poles of s*F(s) must be in the left half-plane (system must be stable). If any pole is on the imaginary axis or in the RHP, the theorem gives a wrong answer.

**Application:** For a step input R(s) = 1/s applied to a system with closed-loop transfer function T(s):

```
y_ss = lim(s->0) s * T(s) * (1/s) = T(0)
```

The steady-state output to a unit step is simply T(0) -- the DC gain of the closed-loop system.

### Initial Value Theorem

```
f(0+) = lim(s->inf) s*F(s)
```

**Prerequisite:** F(s) must be strictly proper (degree of numerator < degree of denominator). Useful for checking that initial conditions are consistent.

## Practical Notes

- The region of convergence (ROC) matters mathematically but rarely in control engineering -- for causal, stable systems, the ROC is always Re(s) > max(real parts of poles).
- Laplace tables are your friend. Memorise the common pairs (step, ramp, exponential, damped sinusoid). Everything else can be derived from these via the properties.
- The s-domain is not just a mathematical trick -- it provides genuine physical insight. Poles tell you the natural frequencies and damping of the system. Zeros tell you which frequencies the system blocks or emphasises.
- For discrete-time systems, the analogous tool is the Z-transform, where z = e^(s*T_s). The same pipeline applies: difference equation -> Z-transform -> H(z).
