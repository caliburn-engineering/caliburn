#pragma once
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <algorithm>

// Relative scalar assertion.
// Passes when |a - b| <= tol * max(|a|, |b|, 1.0).
// The max-1 floor prevents absurdly tight absolute thresholds on near-zero values.
#define ASSERT_REL_NEAR(a, b, tol)                                             \
    do {                                                                        \
        double _a = (double)(a), _b = (double)(b), _t = (double)(tol);        \
        double _scale = std::max({std::fabs(_a), std::fabs(_b), 1.0});        \
        double _diff  = std::fabs(_a - _b);                                    \
        if (_diff > _t * _scale) {                                              \
            std::fprintf(stderr,                                                \
                "%s:%d: ASSERT_REL_NEAR failed: "                              \
                "|%.6g - %.6g| = %.6g > %.6g * %.6g\n",                       \
                __FILE__, __LINE__, _a, _b, _diff, _t, _scale);                \
            std::exit(1);                                                       \
        }                                                                       \
    } while (false)

// Relative matrix assertion (available when Eigen is included before this header).
// Passes when ||A - B||_F <= tol * max(||A||_F, ||B||_F, 1.0).
#ifdef EIGEN_CORE_H
#define ASSERT_MATRIX_REL_NEAR(A, B, tol)                                      \
    do {                                                                        \
        double _nA = (A).norm(), _nB = (B).norm(), _t = (double)(tol);        \
        double _scale = std::max({_nA, _nB, 1.0});                            \
        double _diff  = ((A) - (B)).norm();                                    \
        if (_diff > _t * _scale) {                                              \
            std::fprintf(stderr,                                                \
                "%s:%d: ASSERT_MATRIX_REL_NEAR failed: "                      \
                "||A - B||_F = %.6g > %.6g * %.6g\n",                         \
                __FILE__, __LINE__, _diff, _t, _scale);                        \
            std::exit(1);                                                       \
        }                                                                       \
    } while (false)
#endif
