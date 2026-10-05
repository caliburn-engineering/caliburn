#include "homogeneous_transform.h"

#include <Eigen/Geometry>
#include "assert_rel.h"
#include <cmath>
#include <cstdio>

using namespace caliburn;

// 1e-10: rigid transform entries are O(1); worst case ~16 FP ops per multiply; rounding ≤ 16·2⁻⁵³ ≈ 2e-15
static constexpr double TOL = 1e-10;

static bool vec3_near(const Eigen::Vector3d& a, const Eigen::Vector3d& b, double tol = TOL) {
    return (a - b).norm() < tol;
}

static bool mat4_near(const Eigen::Matrix4d& a, const Eigen::Matrix4d& b, double tol = TOL) {
    return (a - b).norm() < tol;
}

// ---------------------------------------------------------------------------
// 1. Identity transform: no change
// ---------------------------------------------------------------------------
static void test_identity() {
    Eigen::Matrix4d I = Eigen::Matrix4d::Identity();
    Eigen::Vector3d p(1.0, 2.0, 3.0);

    auto result = transform_point(I, p);
    ASSERT_CHECK(vec3_near(result, p), "identity transform should preserve point within TOL");
    std::printf("  [PASS] Identity transform\n");
}

// ---------------------------------------------------------------------------
// 2. Pure translation
// ---------------------------------------------------------------------------
static void test_translation() {
    auto T = translation(1.0, 2.0, 3.0);
    Eigen::Vector3d p(0.0, 0.0, 0.0);

    auto result = transform_point(T, p);
    ASSERT_CHECK(vec3_near(result, Eigen::Vector3d(1.0, 2.0, 3.0)), "translated point should equal offset within TOL");

    // Vector should NOT be translated
    auto v_result = transform_vector(T, Eigen::Vector3d(1.0, 0.0, 0.0));
    ASSERT_CHECK(vec3_near(v_result, Eigen::Vector3d(1.0, 0.0, 0.0)), "translation should not affect direction vectors within TOL");
    std::printf("  [PASS] Pure translation\n");
}

// ---------------------------------------------------------------------------
// 3. Rotation about Z by 90 degrees
// ---------------------------------------------------------------------------
static void test_rot_z_90() {
    auto T = rot_z(M_PI / 2.0);
    Eigen::Vector3d p(1.0, 0.0, 0.0);

    auto result = transform_point(T, p);
    ASSERT_CHECK(vec3_near(result, Eigen::Vector3d(0.0, 1.0, 0.0)), "rot_z(90°) should map (1,0,0) to (0,1,0) within TOL");
    std::printf("  [PASS] Rotation about Z by 90°\n");
}

// ---------------------------------------------------------------------------
// 4. Rotation about X by 90 degrees
// ---------------------------------------------------------------------------
static void test_rot_x_90() {
    auto T = rot_x(M_PI / 2.0);
    Eigen::Vector3d p(0.0, 1.0, 0.0);

    auto result = transform_point(T, p);
    ASSERT_CHECK(vec3_near(result, Eigen::Vector3d(0.0, 0.0, 1.0)), "rot_x(90°) should map (0,1,0) to (0,0,1) within TOL");
    std::printf("  [PASS] Rotation about X by 90°\n");
}

// ---------------------------------------------------------------------------
// 5. Composed transform: translate then rotate
// ---------------------------------------------------------------------------
static void test_compose() {
    auto T1 = translation(1.0, 0.0, 0.0);
    auto T2 = rot_z(M_PI / 2.0);

    // Rotate first, then translate (reading right-to-left: T1 * T2 * p)
    Eigen::Matrix4d T = T1 * T2;
    Eigen::Vector3d p(1.0, 0.0, 0.0);

    auto result = transform_point(T, p);
    // Rotate (1,0,0) by 90° about Z → (0,1,0), then translate by (1,0,0) → (1,1,0)
    ASSERT_CHECK(vec3_near(result, Eigen::Vector3d(1.0, 1.0, 0.0)), "composed transform should map (1,0,0) to (1,1,0) within TOL");
    std::printf("  [PASS] Composed transform (translate + rotate)\n");
}

// ---------------------------------------------------------------------------
// 6. Inverse transform: T * T^{-1} = I
// ---------------------------------------------------------------------------
static void test_inverse() {
    Eigen::Matrix4d T = translation(3.0, -1.0, 2.0) * rot_x(0.5) * rot_y(-0.3);
    Eigen::Matrix4d T_inv = inverse_transform(T);
    Eigen::Matrix4d product = T * T_inv;

    ASSERT_CHECK(mat4_near(product, Eigen::Matrix4d::Identity()), "T * T_inv should equal identity within TOL");
    std::printf("  [PASS] Inverse transform\n");
}

// ---------------------------------------------------------------------------
// 7. Inverse roundtrip: transform then inverse returns original point
// ---------------------------------------------------------------------------
static void test_inverse_roundtrip() {
    Eigen::Matrix4d T = translation(1.0, 2.0, 3.0) * rot_z(0.7);
    Eigen::Vector3d p(4.0, 5.0, 6.0);

    auto p_transformed = transform_point(T, p);
    auto p_back = transform_point(inverse_transform(T), p_transformed);

    ASSERT_CHECK(vec3_near(p_back, p), "inverse roundtrip should recover original point within TOL");
    std::printf("  [PASS] Inverse roundtrip\n");
}

// ---------------------------------------------------------------------------
// 8. Extract rotation and translation
// ---------------------------------------------------------------------------
static void test_extract() {
    Eigen::Matrix3d R = Eigen::AngleAxisd(0.5, Eigen::Vector3d::UnitZ()).toRotationMatrix();
    Eigen::Vector3d t(1.0, 2.0, 3.0);

    auto T = make_transform(R, t);
    auto R_out = extract_rotation(T);
    auto t_out = extract_translation(T);

    // ||R||_F = sqrt(3) ≈ 1.73 > 1, so ASSERT_MATRIX_REL_NEAR would widen the bound; keep absolute check
    ASSERT_CHECK((R - R_out).norm() < TOL, "extracted rotation should match original within TOL");
    ASSERT_CHECK(vec3_near(t, t_out), "extracted translation should match original within TOL");
    std::printf("  [PASS] Extract rotation and translation\n");
}

// ---------------------------------------------------------------------------
// 9. Ball-balancer frame chain: plate tilt + ball position
// ---------------------------------------------------------------------------
static void test_ball_balancer_chain() {
    double alpha = 0.1;   // plate tilt about X
    double beta = 0.05;   // plate tilt about Y
    double h_pivot = 0.3; // pivot height
    double ball_x = 0.05;
    double ball_y = -0.02;
    double ball_r = 0.02;

    // Frame chain: World -> Plate -> Ball
    Eigen::Matrix4d T_plate = translation(0.0, 0.0, h_pivot) * rot_x(alpha) * rot_y(beta);
    auto T_ball = translation(ball_x, ball_y, ball_r);

    Eigen::Vector3d ball_world = transform_point(T_plate * T_ball,
                                                  Eigen::Vector3d::Zero());

    // Ball should be approximately at (ball_x, ball_y, h_pivot + ball_r) for small angles
    // 0.01: small-angle approximation error O(alpha^2 * h_pivot); measured 4.6e-3 (z) and 9.4e-4 (x)
    // in Release. Values are O(0.3) < 1, so the floor-1 scale keeps this the original absolute check.
    ASSERT_REL_NEAR(ball_world(2), h_pivot + ball_r, 0.01);
    ASSERT_REL_NEAR(ball_world(0), ball_x, 0.01);
    std::printf("  [PASS] Ball-balancer frame chain (ball at %.3f, %.3f, %.3f)\n",
                ball_world(0), ball_world(1), ball_world(2));
}

// ---------------------------------------------------------------------------
int main() {
    std::printf("Homogeneous transform tests:\n");

    test_identity();
    test_translation();
    test_rot_z_90();
    test_rot_x_90();
    test_compose();
    test_inverse();
    test_inverse_roundtrip();
    test_extract();
    test_ball_balancer_chain();

    std::printf("All homogeneous transform tests passed.\n");
    return 0;
}
