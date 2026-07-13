---
title: Nyquist Stability Criterion
sources:
  - { book: "Ogata — Modern Control Engineering", chapter: "8" }
  - { book: "Franklin, Powell, Emami-Naeini — Feedback Control of Dynamic Systems", chapter: "6" }
requires:
  - frequency-response.md
  - stability.md
related:
  - compensator-design.md
  - root-locus.md
---

# Nyquist Stability Criterion

The Nyquist criterion determines closed-loop stability by examining the open-loop frequency response L(jw) as w sweeps from -infinity to +infinity. It is the only rigorous frequency-domain stability test for systems with open-loop unstable poles or time delays.

## The Criterion

```
N = # clockwise encirclements of (-1, 0) by the Nyquist contour
P = # open-loop RHP poles (poles of L(s) with Re > 0)
Z = P - N = # closed-loop RHP poles

Stability requirement: Z = 0
```

### Interpretation

- **Stable open-loop plant (P = 0):** The Nyquist plot must NOT encircle (-1, 0). Any encirclement means the closed-loop is unstable.
- **Unstable open-loop plant (P > 0):** The Nyquist plot must encircle (-1, 0) exactly P times counter-clockwise (N = -P) to achieve Z = 0.

### Sign Convention

Counter-clockwise encirclements are counted as negative N. So for an unstable plant with P = 2, we need N = -2, meaning 2 CCW encirclements.

## When to Use Nyquist vs Bode

| Criterion | Best for | Limitations |
|---|---|---|
| Bode (gain/phase margins) | Quick assessment, designing compensators, reading margins | Only rigorous when OL is stable and minimum-phase |
| Nyquist | OL unstable plants, time delays, conditional stability | Harder to design compensators directly from |
| Root Locus | Gain selection, pole placement, parametric sensitivity | Limited to rational transfer functions |

**Use Nyquist when:**
- The open-loop system has RHP poles (unstable plant)
- There is a significant time delay (e^{-sT})
- The system exhibits conditional stability (stable for a range of gains, unstable outside)
- The Bode plot suggests marginal stability but you need certainty

## Reading Margins from the Nyquist Plot

### Gain Margin

The gain margin corresponds to where the Nyquist plot crosses the negative real axis:

```
GM = 1 / |L(jw)| at the negative real-axis crossing
```

Example: If the plot crosses the real axis at (-0.5, 0), then GM = 1/0.5 = 2 = 6 dB.

### Phase Margin

The phase margin corresponds to where the Nyquist plot crosses the unit circle:

```
PM = angle from the negative real axis to the point where |L(jw)| = 1
```

Visually: draw the unit circle centered at the origin. Where the Nyquist contour crosses it, the angle measured from the -180° direction (negative real axis) to that crossing point is the phase margin.

## Constructing the Nyquist Plot

1. Evaluate L(jw) for w from 0 to +infinity — this gives the upper half of the contour
2. The lower half (w from -infinity to 0) is the complex conjugate mirror of the upper half
3. For poles on the imaginary axis (integrators), indent the contour with a small semicircle to the right
4. Plot Re(L(jw)) vs Im(L(jw)) parametrically

### Indentation for Imaginary-Axis Poles

If L(s) has a pole at s = 0 (integrator), the standard Nyquist contour detours around it with an infinitesimal semicircle in the RHP. This produces a large arc at infinity in the Nyquist plot. For a single integrator: a semicircle of infinite radius from +90° to -90°.

## Conditional Stability

A system is conditionally stable if it is stable for a range of gains but unstable for gains above or below that range. On the Nyquist plot, this appears as the contour passing near (-1, 0) multiple times.

```
Example: A system stable for 2 < K < 10
- At K = 2: contour just avoids encircling (-1, 0)
- At K = 10: contour begins to encircle (-1, 0)
- Bode margins at the nominal gain may look healthy, but Nyquist reveals the fragility
```

Conditional stability cannot be reliably detected from Bode plots alone — Nyquist is required.

## Time Delay Systems

A pure time delay e^{-sT} adds phase lag linearly with frequency:

```
|e^{-jwT}| = 1 (magnitude unchanged)
angle(e^{-jwT}) = -wT radians = -wT × 180/pi degrees
```

This means:
- The Bode magnitude plot is unaffected
- The phase drops without bound at high frequencies
- The Nyquist plot spirals inward toward the origin as w → infinity

For stability analysis of delay systems, Nyquist is the correct tool because:
- The delay makes the system non-rational (infinite-dimensional)
- Root locus cannot be directly applied
- The Nyquist contour correctly captures the spiral behavior

## Practical Examples

### Type-1 System (One Integrator)

```
L(s) = K / (s * (s + a))
```

- P = 0 (open-loop stable, integrator is on the boundary)
- Nyquist plot starts at -90° on the infinite semicircle (from the integrator indent)
- Approaches origin from the third quadrant as w → infinity
- For K/a < some threshold: no encirclement → stable
- Gain margin visible where the plot crosses the negative real axis

### Unstable Plant

```
L(s) = K / ((s - 1) * (s + 2) * (s + 3))
```

- P = 1 (one RHP pole at s = 1)
- Need N = -1 (one CCW encirclement) for Z = 0
- The Nyquist plot must wrap around (-1, 0) once counter-clockwise

## Implementation Notes

### Computing the Nyquist Plot

```
For w in logarithmically-spaced grid from w_min to w_max:
    L_jw = evaluate_transfer_function(L, j*w)
    Re_upper[i] = real(L_jw)
    Im_upper[i] = imag(L_jw)

    // Mirror for lower half (w negative)
    Re_lower[i] = real(L_jw)      // same real part
    Im_lower[i] = -imag(L_jw)     // conjugate
```

### Counting Encirclements Numerically

Use the winding number formula:

```
N = (1 / 2*pi) * sum of angle changes around (-1, 0)

For each consecutive pair of points on the contour:
    dtheta = angle(L[i+1] - (-1)) - angle(L[i] - (-1))
    unwrap dtheta to [-pi, pi]
    total += dtheta

N = round(total / (2*pi))
```

Positive N = clockwise encirclements.
