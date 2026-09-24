#include <Eigen/Dense>
#include "assert_rel.h"

#include <cstdio>

static void test_scalar_exact_match() {
    ASSERT_REL_NEAR(1.0, 1.0, 1e-12);
    ASSERT_REL_NEAR(-3.14, -3.14, 1e-12);
    ASSERT_REL_NEAR(0.0, 0.0, 1e-12);
    std::printf("  [PASS] scalar: exact match\n");
}

static void test_scalar_within_tolerance() {
    // 100.0 vs 100.5: difference 0.5, scale max(100, 100.5, 1) = 100.5, ratio ≈ 0.00498
    ASSERT_REL_NEAR(100.0, 100.5, 0.01);
    // Large values: same ratio logic applies
    ASSERT_REL_NEAR(1e8, 1e8 + 1e3, 1e-4);
    std::printf("  [PASS] scalar: within tolerance\n");
}

static void test_scalar_near_zero_falls_back_to_absolute() {
    // Near-zero operands: scale falls back to 1.0, so tol acts as absolute epsilon
    ASSERT_REL_NEAR(0.0, 1e-10, 1e-9);
    ASSERT_REL_NEAR(1e-15, 0.0, 1e-9);
    std::printf("  [PASS] scalar: near-zero fallback to absolute scale\n");
}

static void test_matrix_exact_match() {
    Eigen::Matrix3d A = Eigen::Matrix3d::Identity();
    ASSERT_MATRIX_REL_NEAR(A, A, 1e-12);
    std::printf("  [PASS] matrix: exact match\n");
}

static void test_matrix_within_tolerance() {
    Eigen::Matrix2d A;
    A << 1.0, 2.0, 3.0, 4.0;
    // Perturb by less than 1e-6 relative
    Eigen::Matrix2d B = A * (1.0 + 5e-7);
    ASSERT_MATRIX_REL_NEAR(A, B, 1e-6);
    std::printf("  [PASS] matrix: within tolerance\n");
}

static void test_matrix_zero_matrices() {
    Eigen::Matrix2d Z = Eigen::Matrix2d::Zero();
    ASSERT_MATRIX_REL_NEAR(Z, Z, 1e-12);
    std::printf("  [PASS] matrix: zero matrices\n");
}

static void test_matrix_falls_back_to_absolute_when_small() {
    // Both norms are below 1.0, so scale is clamped to 1.0
    Eigen::Matrix2d A = 1e-10 * Eigen::Matrix2d::Identity();
    Eigen::Matrix2d B = Eigen::Matrix2d::Zero();
    ASSERT_MATRIX_REL_NEAR(A, B, 1e-9);
    std::printf("  [PASS] matrix: fallback to absolute scale for tiny matrices\n");
}

int main() {
    std::printf("assert_rel helper tests:\n");
    test_scalar_exact_match();
    test_scalar_within_tolerance();
    test_scalar_near_zero_falls_back_to_absolute();
    test_matrix_exact_match();
    test_matrix_within_tolerance();
    test_matrix_zero_matrices();
    test_matrix_falls_back_to_absolute_when_small();
    std::printf("All assert_rel tests passed.\n");
    return 0;
}
