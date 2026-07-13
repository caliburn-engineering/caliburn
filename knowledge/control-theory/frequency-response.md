---
title: Frequency Response & Bode Plots
sources:
  - { book: "Ogata - Modern Control Engineering", chapter: "8" }
  - { book: "Franklin, Powell, Emami-Naeini - Feedback Control of Dynamic Systems", chapter: "6" }
  - { note: "engineering experience" }
requires:
  - state-space.md
  - stability.md
  - ../math/linear-algebra/eigenvalues.md
related:
  - controllers/pid.md
  - controllers/lqr.md
  - nyquist.md
  - root-locus.md
  - compensator-design.md
reference: projects/bode-explorer/transfer_function.h
---

# Frequency Response & Bode Plots

Frequency response describes how a linear time-invariant (LTI) system amplifies or attenuates sinusoidal inputs across frequencies. Given a transfer function H(s), the frequency response is H(jw) — substitute s = jw and evaluate the resulting complex number. The magnitude tells you the gain at that frequency; the argument tells you the phase shift.

Bode plots display this information on two stacked plots: magnitude (dB) vs. log-frequency, and phase (degrees) vs. log-frequency. The log scales reveal the asymptotic structure that makes hand-analysis tractable.

## Transfer Function Representation

### Factored (Pole-Zero) Form

A transfer function is most useful for Bode analysis when written in factored form:

```
H(s) = K × product_of_zero_terms / product_of_pole_terms
```

Each factor is one of these standard building blocks:

| Element | Transfer Function | Magnitude Slope | Phase Contribution |
|---|---|---|---|
| Gain K | K | flat shift of 20·log10(K) dB | 0° (K>0), ±180° (K<0) |
| Real pole at w_c | 1 / (1 + s/w_c) | 0 dB below w_c, -20 dB/dec above | 0° → -90° centered at w_c |
| Real zero at w_c | (1 + s/w_c) | 0 dB below w_c, +20 dB/dec above | 0° → +90° centered at w_c |
| Complex pole pair (w_n, zeta) | 1 / (1 + 2·zeta·s/w_n + s²/w_n²) | 0 dB below w_n, -40 dB/dec above | 0° → -180°, sharp transition near w_n |
| Complex zero pair (w_n, zeta) | (1 + 2·zeta·s/w_n + s²/w_n²) | 0 dB below w_n, +40 dB/dec above | 0° → +180° |
| Integrator | 1/s | -20 dB/dec everywhere | -90° constant |
| Differentiator | s | +20 dB/dec everywhere | +90° constant |

### Evaluating H(jw)

For each element, substitute s = jw and compute the complex value. The total response is the product of all element contributions times the gain K:

```
H(jw) = K × ∏ element_i(jw)
```

Magnitude in dB: `|H(jw)|_dB = 20 · log10(|H(jw)|)`
Phase in degrees: `∠H(jw) = arg(H(jw)) × 180/π`

Because the product of complex numbers adds their magnitudes (in dB) and phases, each element's contribution can be analyzed independently and then summed — the fundamental insight that makes Bode analysis practical.

## Key Rules of Thumb

### Magnitude

- Each real pole contributes -20 dB/decade above its corner frequency
- Each real zero contributes +20 dB/decade above its corner frequency
- Complex pairs contribute ±40 dB/decade (two poles or two zeros)
- At the corner frequency, a real pole/zero is at -3 dB / +3 dB from its asymptote
- An underdamped complex pole pair (zeta < 0.707) produces a resonance peak at w_n; peak magnitude ≈ -20·log10(2·zeta) dB

### Phase

- A real pole transitions from 0° to -90° over roughly one decade centered at w_c (the transition spans from w_c/10 to 10·w_c)
- A real zero transitions from 0° to +90° over the same range
- Complex pole pairs transition from 0° to -180°; the transition is sharper for lower damping
- The phase at the corner frequency of a real pole/zero is exactly -45° / +45°

### Gain and Phase Margins

- **Gain crossover frequency (w_gc):** where |H(jw)| = 0 dB
- **Phase margin:** 180° + ∠H(jw_gc) — how far above -180° the phase is at gain crossover
- **Phase crossover frequency (w_pc):** where ∠H(jw) = -180°
- **Gain margin:** -|H(jw_pc)|_dB — how many dB below 0 dB the magnitude is at phase crossover
- Rule of thumb: phase margin > 30° and gain margin > 6 dB for robust stability

## Common Transfer Functions

### 1st-Order Low-Pass Filter

```
H(s) = 1 / (1 + s/w_c)
```

One real pole at w_c. Flat at 0 dB below w_c, rolls off at -20 dB/dec above. Phase goes from 0° to -90°.

### 1st-Order High-Pass Filter

```
H(s) = s / (s + w_c) = (1/w_c) · s · 1/(1 + s/w_c)
```

One differentiator + one real pole at w_c, with gain = 1/w_c. Rises at +20 dB/dec below w_c, flattens to 0 dB above. Phase goes from +90° to 0°.

Note: when using factored form with normalized elements, the gain factor 1/w_c is required to ensure unity gain at high frequencies. A common mistake is omitting this and getting a system whose DC behavior is wrong.

### 2nd-Order System

```
H(s) = w_n² / (s² + 2·zeta·w_n·s + w_n²) = 1 / (1 + 2·zeta·s/w_n + s²/w_n²)
```

One complex pole pair. Behavior depends on damping ratio zeta:
- zeta > 1: overdamped — equivalent to two real poles, no resonance
- zeta = 1: critically damped — fastest non-oscillatory response
- zeta < 1: underdamped — resonance peak at w_n, oscillatory step response
- zeta = 0.707: maximally flat (Butterworth) — no resonance peak, -3 dB at w_n

### PID Controller

```
C(s) = K_p + K_i/s + K_d·s = (K_d·s² + K_p·s + K_i) / s
```

In factored form: an integrator + two real zeros (from the quadratic numerator). The Bode plot shows -20 dB/dec at low frequencies (integrator dominance), rising through the zeros, then +20 dB/dec at high frequencies (derivative dominance).

### Notch Filter

```
H(s) = (s² + 2·zeta_z·w_n·s + w_n²) / (s² + 2·zeta_p·w_n·s + w_n²)
```

Complex zero pair + complex pole pair at the same w_n but with different damping. The zeros create a deep notch (narrow if zeta_z is small), the poles recover the response on either side.

## Closed-Loop Frequency Response

### Unity Feedback Configuration

Given a plant G(s) and controller C(s) in a standard negative feedback loop:

```
        +       ┌──────┐    ┌──────┐
r(s) -->(+)---->│ C(s) ├--->│ G(s) ├--+--> y(s)
         ^-     └──────┘    └──────┘  │
         │                            │
         └────────────────────────────┘
```

**Open-loop transfer function:** `L(s) = G(s) · C(s)`

**Closed-loop transfer function:** `T(s) = L(s) / (1 + L(s))`

### Computing Closed-Loop from Frequency Response

Since we already have H(jw) as complex numbers, the closed-loop can be computed directly without factored form:

```
L(jw) = G(jw) · C(jw)           // complex multiplication
T(jw) = L(jw) / (1 + L(jw))     // complex division
```

This is evaluated point-by-point on the same frequency grid. No need to combine poles and zeros algebraically — just multiply and divide complex numbers.

### Stability Margins on the Open-Loop

Margins are always analyzed on L(s), not T(s):
- **Phase margin** is measured on L(jw) at the gain crossover
- **Gain margin** is measured on L(jw) at the phase crossover
- When designing C(s), the goal is to shape L(s) to achieve adequate margins

### Closed-Loop Bandwidth

The closed-loop bandwidth is the frequency where |T(jw)| = -3 dB. This determines the speed of response:
- Higher bandwidth = faster response to setpoint changes
- But also more noise amplification
- A resonance peak in |T(jw)| above 0 dB indicates an underdamped closed-loop

## Phase Unwrapping

When computing phase from arg(H(jw)), the result is in [-180°, +180°]. For systems with many poles, the phase can accumulate beyond -180°, causing discontinuous jumps. Phase unwrapping removes these jumps by tracking the accumulated phase:

```
for each frequency point i > 0:
    diff = phase[i] - phase[i-1]
    while diff > 180:  diff -= 360
    while diff < -180: diff += 360
    phase[i] = phase[i-1] + diff
```

This produces a smooth, monotonically decreasing phase curve for stable minimum-phase systems.

## Implementation Notes

### Numerical Considerations

- Evaluate on a logarithmically spaced frequency grid (equal spacing in log10 space). 500 points across 5 decades (0.01 to 1000 rad/s) gives good visual resolution.
- Clamp magnitude to avoid log(0): use a floor of ~1e-30 before taking 20·log10.
- Pure integrators (1/s) evaluate to infinity at w=0 — start the frequency grid above zero.

### Plotting with ImPlot

- Use `ImPlotScale_Log10` on the X axis for log-frequency
- Magnitude plot: Y axis in dB (linear scale), X axis log
- Phase plot: Y axis in degrees (linear scale), X axis log
- Draw reference lines at 0 dB and -180° for quick margin assessment
- Mark corner frequencies with vertical lines, color-coded by pole (blue) vs. zero (green)

### Building the Transfer Function Incrementally

The factored form maps naturally to an element list. Each element stores its type, corner frequency, and damping ratio. To evaluate H(jw):

1. Start with h = K (the DC gain)
2. For each element, evaluate element.H(jw) and multiply into h
3. Take magnitude and phase from the final complex number

This approach lets the user add/remove individual poles and zeros and see the effect immediately — each element's contribution is independent.

## First-Order System Quick Reference

| Feature | Value |
|---|---|
| Corner frequency | w = 1/tau |
| Magnitude at corner | -3 dB |
| High-frequency slope | -20 dB/decade |
| Phase at corner | -45° |

## Phase Margin to Damping/Overshoot Approximation

| PM | zeta | Overshoot % |
|---|---|---|
| 70° | 0.7 | ~5% |
| 60° | 0.6 | ~10% |
| 50° | 0.5 | ~16% |
| 45° | 0.45 | ~23% |
| 30° | 0.3 | ~37% |

Rough formula: zeta ≈ PM / 100 (valid for PM in the 30°–70° range).

This table is critical for translating between frequency-domain specs (phase margin) and time-domain behavior (overshoot). When a spec says "overshoot < 10%", you need PM >= 60°.

## Time Delay Effect on Bode

A pure time delay e^{-sT} has unity magnitude at all frequencies but adds phase lag that grows linearly:

```
|e^{-jwT}| = 1          (magnitude unchanged)
angle(e^{-jwT}) = -wT   (radians, linear with frequency)
```

### Practical Impact

- At w_c = 20 rad/s with T = 50 ms: extra phase lag = 20 × 0.05 = 1 rad = 57° — devastating for stability
- Any digital controller introduces at least one sample period of delay (T = T_s)
- CAN bus communication adds 1–5 ms of delay depending on bus load
- Rule of thumb: delay is manageable if w_c × T < 0.5 rad (about 30° of extra lag)

### Design Implications

- Higher bandwidth (higher w_c) makes the system more sensitive to delay
- If delay is fixed, there is a maximum achievable bandwidth: w_c_max ≈ 0.5 / T
- For a 1 kHz control loop (T_s = 1 ms): w_c_max ≈ 500 rad/s — usually fine
- For a 100 Hz loop (T_s = 10 ms): w_c_max ≈ 50 rad/s — may limit performance

## Practical Notes

- **Minimum-phase systems** have all poles and zeros in the left half-plane. Their phase is uniquely determined by the magnitude curve. Most physical systems you'll encounter are minimum-phase.
- **Non-minimum-phase systems** (right half-plane zeros) have additional phase lag not visible in the magnitude plot. They impose fundamental bandwidth limitations on achievable closed-loop performance.
- **The Bode gain-phase relationship**: for minimum-phase systems, the phase at any frequency is determined by the slope of the magnitude curve. A slope of -20 dB/dec corresponds to -90° of phase. This is why you can sketch approximate phase from the magnitude asymptotes.
