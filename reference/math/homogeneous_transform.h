#pragma once

#include <Eigen/Core>

namespace caliburn {

// Construct a 4x4 homogeneous rotation about the X axis.
Eigen::Matrix4d rot_x(double theta);

// Construct a 4x4 homogeneous rotation about the Y axis.
Eigen::Matrix4d rot_y(double theta);

// Construct a 4x4 homogeneous rotation about the Z axis.
Eigen::Matrix4d rot_z(double theta);

// Construct a 4x4 pure translation.
Eigen::Matrix4d translation(double tx, double ty, double tz);
Eigen::Matrix4d translation(const Eigen::Vector3d& t);

// Construct a 4x4 transform from a 3x3 rotation and 3x1 translation.
Eigen::Matrix4d make_transform(const Eigen::Matrix3d& R, const Eigen::Vector3d& t);

// Efficient inverse of a homogeneous transform (uses R^T, not general inverse).
Eigen::Matrix4d inverse_transform(const Eigen::Matrix4d& T);

// Transform a point (applies rotation + translation).
Eigen::Vector3d transform_point(const Eigen::Matrix4d& T, const Eigen::Vector3d& p);

// Transform a vector (applies rotation only, no translation).
Eigen::Vector3d transform_vector(const Eigen::Matrix4d& T, const Eigen::Vector3d& v);

// Extract the 3x3 rotation from a 4x4 transform.
Eigen::Matrix3d extract_rotation(const Eigen::Matrix4d& T);

// Extract the translation from a 4x4 transform.
Eigen::Vector3d extract_translation(const Eigen::Matrix4d& T);

}  // namespace caliburn
