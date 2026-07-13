---
title: Root Locus Method
sources:
  - { book: "Ogata — Modern Control Engineering", chapter: "6" }
  - { book: "Franklin, Powell, Emami-Naeini — Feedback Control of Dynamic Systems", chapter: "5" }
requires:
  - frequency-response.md
  - ../math/linear-algebra/eigenvalues.md
related:
  - stability.md
  - nyquist.md
  - compensator-design.md
---

# Root Locus Method

The root locus shows how the closed-loop poles of a system move in the complex plane as a parameter (typically the loop gain K) varies from 0 to infinity. It provides direct insight into how gain affects stability, damping, and natural frequency — making it essential for gain selection and pole placement.

## The Characteristic Equation

For a unity-feedback system with open-loop transfer function KG(s)H(s):

```
1 + K * G(s) * H(s) = 0
```

The root locus is the set of all s satisfying this equation as K varies from 0 to infinity.

## Construction Rules

For `1 + K * G(s)H(s) = 0`, where G(s)H(s) has n poles and m zeros (n >= m):

```
1. n branches (one per open-loop pole)
2. Start at OL poles (K = 0), end at OL zeros or infinity (K → ∞)
3. On real axis: locus exists to the left of an odd number of real poles + zeros
4. Asymptotes: angles = (2k+1) × 180° / (n-m), k = 0, 1, ..., n-m-1
   Centroid = (sum of poles - sum of zeros) / (n-m)
5. Breakaway/break-in points: solve dK/ds = 0 on real-axis segments
6. jw-axis crossing: substitute s = jw into characteristic equation,
   solve for K and w (gives the gain at which system becomes unstable)
7. Departure angle from complex pole p_i:
   theta_d = 180° - sum(angles from other poles to p_i) + sum(angles from zeros to p_i)
8. Arrival angle at complex zero z_i:
   theta_a = 180° + sum(angles from poles to z_i) - sum(angles from other zeros to z_i)
```

## Key Insights

### What Root Locus Tells You

| Feature | Meaning |
|---|---|
| Branches crossing jw-axis | Critical gain for instability |
| Branches in RHP | Unstable closed-loop poles for those K values |
| Breakaway from real axis | Transition from overdamped to underdamped (complex poles appear) |
| Distance from real axis | Natural frequency of oscillation |
| Distance from jw-axis | Damping (further left = more damped) |
| Asymptote angles | Ultimate behavior at high gain |

### Gain Selection Strategy

1. Identify the region of the locus where all branches are in the LHP (stable region)
2. Within that region, pick K to place dominant poles at desired damping ratio and natural frequency
3. Verify that non-dominant poles are far enough left to not significantly affect the response

## Common Patterns

### Two Real Poles, No Zeros

```
G(s) = 1 / ((s + a)(s + b))     [a, b > 0]
```

- Two branches start at -a and -b on the real axis
- They meet at a breakaway point between -a and -b
- Break into the complex plane and head toward infinity along ±90° asymptotes
- As K increases: overdamped → critically damped → underdamped → always stable (never crosses jw-axis)

### Two Poles, One Zero

```
G(s) = (s + z) / ((s + a)(s + b))
```

- One branch ends at the zero (K → ∞), the other goes to infinity along 180°
- Adding a zero "pulls" the locus toward the LHP — stabilizing effect
- This is the geometric explanation for why derivative action (a zero) improves stability

### Three Poles, No Zeros

```
G(s) = 1 / ((s + a)(s + b)(s + c))
```

- Asymptotes at 60°, 180°, 300° (three branches, centroid at -(a+b+c)/3)
- Two branches eventually enter the RHP — there exists a critical gain
- The jw-axis crossing gives the maximum stable gain

## Connection to Frequency Domain

The root locus and Bode/Nyquist methods are complementary views of the same underlying mathematics:

- **Root locus** shows closed-loop pole locations explicitly — direct connection to time-domain response
- **Bode** shows open-loop frequency response — easier for compensator design
- **Nyquist** gives a rigorous stability count — handles delays and unstable plants

Key correspondences:
- The critical gain from root locus (jw crossing) equals the gain margin from Bode
- Adding a zero in root locus corresponds to adding phase lead in Bode
- Adding a pole in root locus corresponds to adding phase lag in Bode

## Design with Root Locus

### Adding a Compensator

To move the root locus into a desired region:

1. **Add a zero** (lead compensator): pulls branches toward the zero location, adds phase
2. **Add a pole** (lag compensator or integrator): pushes branches toward RHP, adds gain at low frequencies
3. **Pole-zero pair** (notch, lead-lag): reshapes specific regions of the locus

### Dominant Pole Placement

For a desired second-order response (specified by zeta and w_n):

1. Mark the desired closed-loop pole location: s* = -zeta*w_n ± j*w_n*sqrt(1-zeta^2)
2. Check if s* lies on the current root locus (angle condition)
3. If not, design a compensator whose zero/pole placement makes s* satisfy the angle condition
4. Find K from the magnitude condition at s*

## Practical Limitations

- Root locus shows the trajectory but not the closed-loop frequency response directly
- For MIMO systems, root locus is less useful (use eigenvalue loci instead)
- Time delays cannot be represented in standard root locus (use Pade approximation or Nyquist)
- The method assumes a single gain parameter varies — for multi-parameter sensitivity, use other tools

## Implementation Notes

### Computing the Root Locus Numerically

For each K value in a range:
1. Form the closed-loop characteristic polynomial: den(s) + K * num(s)
2. Find all roots of the resulting polynomial (eigenvalues of companion matrix)
3. Plot each root in the complex plane

```
For K in logspace(K_min, K_max, N_points):
    coeffs = polynomial_add(den_coeffs, K * num_coeffs)
    roots = find_roots(coeffs)
    plot each root as a point
```

### Finding the Critical Gain

The jw-axis crossing is found by substituting s = jw:

```
1 + K * G(jw) * H(jw) = 0
→ G(jw) * H(jw) = -1/K

Real part: Re(G(jw)*H(jw)) = -1/K
Imaginary part: Im(G(jw)*H(jw)) = 0

Solve Im = 0 for w, then K = -1/Re at that w.
```

This is equivalent to finding where the Nyquist plot crosses the negative real axis.
