---
title: Interpolation Techniques in Numerical Methods
sources:
  - { book: "Yet Another Introduction to Numerical Methods", author: "Dmitri V. Fedorov", chapter: "Ch.1 Interpolation (pp. 1-12)", pages: "1-12" }
requires: []
related: []
---

# Interpolation Techniques in Numerical Methods

Interpolation is a fundamental numerical method for approximating values between known data points. It is critical in simulation for creating smooth models and in control systems for signal processing and dynamic system modeling. The chapter explores Lagrange polynomials, spline interpolation, and their implementation in C.

## Key Equations

### Lagrange Interpolation Polynomial
$$
S(x) = \sum_{i=1}^{n} y_i \prod_{\substack{j=1 \\ j \neq i}}^{n} \frac{x - x_j}{x_i - x_j}
$$

### Spline Continuity Conditions
For cubic splines:
$$
S_i(x_i) = y_i, \quad S_i(x_{i+1}) = y_{i+1}
$$
$$
S_i'(x_{i+1}) = S_{i+1}'(x_{i+1}), \quad S_i''(x_{i+1}) = S_{i+1}''(x_{i+1})
$$

## Source Implementation

### Dmitri V. Fedorov — Ch.1 Interpolation (pp. 1-12)
**Lagrange Interpolation (C):**
```c
double interp(int n, double* x, double* y, double z) {
    double s = 0.0;
    for (int i = 0; i < n; i++) {
        double p = 1.0;
        for (int j = 0; j < n; j++) {
            if (j != i) p *= (z - x[j]) / (x[i] - x[j]);
        }
        s += y[i] * p;
    }
    return s;
}
```

**Linear Interpolation (C):**
```c
double interp(int n, double* x, double* y, double z) {
    assert(z >= x[0] && z <= x[n-1] && n > 1);
    int i = 0, j = n - 1;
    while (j - i > 1) {
        int m = (i + j) / 2;
        if (z > x[m]) i = m;
        else j = m;
    }
    return y[i] + (y[i+1] - y[i]) / (x[i+1] - x[i]) * (z - x[i]);
}
```

## Implementation Notes

- **Numerical Stability:** Avoid division by zero in linear interpolation by ensuring sorted `x` arrays. Use binary search for efficient interval finding.
- **Cubic Splines:** Solve a tridiagonal system for second derivatives using the Thomas algorithm. Implement continuity conditions explicitly for smoothness.
- **Quadratic Splines:** Use forward/backward recursion with arbitrary boundary conditions (e.g., $ c_1 = 0 $) to determine coefficients recursively.
- **Linear Algebra:** Leverage matrix operations (e.g., Eigen library) for solving spline coefficient systems, particularly for higher-order splines.