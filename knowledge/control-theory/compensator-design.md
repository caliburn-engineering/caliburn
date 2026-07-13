---
title: Compensator Design (Lead, Lag, Notch)
sources:
  - { book: "Ogata — Modern Control Engineering", chapter: "7", pages: "310-380" }
  - { book: "Franklin, Powell, Emami-Naeini — Feedback Control of Dynamic Systems", chapter: "6" }
requires:
  - frequency-response.md
  - controllers/pid.md
related:
  - stability.md
  - nyquist.md
  - root-locus.md
---

# Compensator Design (Lead, Lag, Notch)

Compensators are additional transfer functions inserted into the control loop to reshape the open-loop frequency response without changing the plant. The goal is to achieve adequate stability margins and steady-state accuracy by strategically adding phase or adjusting gain at specific frequencies.

## Compensator Types

| Type | Transfer Function | Purpose | Trade-off |
|---|---|---|---|
| Lead | (s+z)/(s+p), p>z | Add phase at crossover — increase PM | Amplifies high-frequency noise |
| Lag | (s+z)/(s+p), z>p | Increase low-frequency gain — reduce e_ss | Adds phase lag (place well below w_gc) |
| Lead-lag | Both combined | Improve both e_ss and PM | Complexity |
| Notch | (s^2+2*z1*wn*s+wn^2)/(s^2+2*z2*wn*s+wn^2) | Kill resonance peak | Narrow — must hit right frequency |

## Lead Compensator Design (Quick Method)

The lead compensator adds positive phase at its geometric center frequency. The design procedure places maximum phase contribution at the desired gain crossover frequency.

```
1. Measure PM deficit: phi_needed = PM_desired - PM_current + 5° (margin for gain shift)
2. Calculate alpha: sin(phi_max) = (1 - alpha) / (1 + alpha)
   → alpha = (1 - sin(phi_max)) / (1 + sin(phi_max))
3. Place max phase at new gain crossover: w_max = w_gc_new
4. Zero: z = w_max * sqrt(alpha)
5. Pole: p = w_max / sqrt(alpha)
6. Gain adjustment: compensator adds 10*log10(1/alpha) dB at w_max
```

### Design Intuition

- alpha < 1 always (ratio of zero to pole frequency)
- Smaller alpha = more phase boost but more noise amplification
- Maximum achievable phase per stage: ~60° (alpha ~ 0.05). Beyond that, use two cascaded lead stages.
- The 5° extra accounts for the gain crossover shifting due to the compensator's magnitude contribution.

## Lag Compensator Design

The lag compensator increases low-frequency gain (improving steady-state error) without significantly affecting the phase at crossover — provided it is placed well below w_gc.

```
1. Determine required gain increase: K_boost = e_ss_current / e_ss_desired
2. Set ratio: beta = K_boost (the zero-to-pole ratio)
3. Place zero at w_z = w_gc / 10 (one decade below crossover)
4. Pole at w_p = w_z / beta
5. The phase lag at crossover is small because the compensator's transition is far below w_gc
```

### Key Constraint

The lag compensator's negative phase contribution must be negligible at crossover. Rule of thumb: place the compensator's upper frequency (the zero) at least one decade below w_gc. If this is violated, the lag compensator will eat into phase margin rather than improve things.

## Lead-Lag Compensator

When both steady-state accuracy and phase margin are deficient, combine:

1. Design the lag section first (for the required DC gain boost)
2. Then design the lead section (for the required phase boost at crossover)
3. Verify that the lag section doesn't interfere with the lead section's phase contribution

The sections should be well-separated in frequency: lag below w_gc/10, lead centered at w_gc.

## Notch Compensator

Used to eliminate a specific resonance peak in the plant (e.g., a mechanical flexibility mode):

```
C_notch(s) = (s^2 + 2*zeta_z*w_n*s + w_n^2) / (s^2 + 2*zeta_p*w_n*s + w_n^2)
```

- w_n = resonance frequency to suppress
- zeta_z << zeta_p: narrow notch (zeros have low damping, poles have higher damping)
- The zeros cancel the plant's resonance; the poles provide a controlled rolloff

### Practical Concerns

- The notch must be placed precisely at the resonance frequency. If the plant's resonance drifts with temperature or load, the notch becomes ineffective.
- A wider notch (higher zeta_z) is more robust to frequency uncertainty but provides less suppression.
- Always verify that the notch doesn't create problems at nearby frequencies.

## Design Workflow

1. Analyze open-loop Bode plot: identify current margins, crossover frequency, and deficiencies
2. Choose compensator type based on the deficiency:
   - PM too low → lead
   - e_ss too high → lag
   - Both → lead-lag
   - Resonance peak → notch
3. Design the compensator using the quick methods above
4. Verify on the Bode plot: check that margins meet specs
5. Simulate the closed-loop step response to confirm time-domain performance
6. Iterate if needed — frequency-domain design is rarely one-shot

## Connection to PID

A PID controller can be viewed through the compensator lens:
- The integral term (K_i/s) provides infinite DC gain — the ultimate lag compensator
- The derivative term (K_d*s) provides phase lead at crossover
- The proportional term (K_p) sets the overall gain level

The advantage of explicit lead/lag design over PID tuning: direct control over where phase is added and where gain is boosted, with clear frequency-domain reasoning at each step.

## Implementation Notes

For the ball-balancer or any real system:
- Lead compensation is typically needed when sensor noise is low and the plant has significant phase lag at the desired bandwidth
- Lag compensation is appropriate when steady-state accuracy is the primary concern (e.g., position tracking)
- Notch filters are essential for mechanical systems with structural resonances (e.g., flexible beams, gear backlash oscillations)
