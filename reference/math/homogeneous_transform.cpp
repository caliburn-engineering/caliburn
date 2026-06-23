#include "homogeneous_transform.h"

#include <cmath>

namespace caliburn {

Eigen::Matrix4d rot_x(double theta) {
    double c = std::cos(theta), s = std::sin(theta);
    Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
    T(1, 1) = c;  T(1, 2) = -s;
    T(2, 1) = s;  T(2, 2) = c;
    return T;
}

Eigen::Matrix4d rot_y(double theta) {
    double c = std::cos(theta), s = std::sin(theta);
    Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
    T(0, 0) = c;  T(0, 2) = s;
    T(2, 0) = -s; T(2, 2) = c;
    return T;
}

Eigen::Matrix4d rot_z(double theta) {
    double c = std::cos(theta), s = std::sin(theta);
    Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
    T(0, 0) = c;  T(0, 1) = -s;
    T(1, 0) = s;  T(1, 1) = c;
    return T;
}

Eigen::Matrix4d translation(double tx, double ty, double tz) {
    Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
    T(0, 3) = tx;
    T(1, 3) = ty;
    T(2, 3) = tz;
    return T;
}

Eigen::Matrix4d translation(const Eigen::Vector3d& t) {
    return translation(t.x(), t.y(), t.z());
}

Eigen::Matrix4d make_transform(const Eigen::Matrix3d& R, const Eigen::Vector3d& t) {
    Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
    T.block<3, 3>(0, 0) = R;
    T.block<3, 1>(0, 3) = t;
    return T;
}

Eigen::Matrix4d inverse_transform(const Eigen::Matrix4d& T) {
    Eigen::Matrix3d R = T.block<3, 3>(0, 0);
    Eigen::Vector3d t = T.block<3, 1>(0, 3);
    Eigen::Matrix4d T_inv = Eigen::Matrix4d::Identity();
    T_inv.block<3, 3>(0, 0) = R.transpose();
    T_inv.block<3, 1>(0, 3) = -R.transpose() * t;
    return T_inv;
}

Eigen::Vector3d transform_point(const Eigen::Matrix4d& T, const Eigen::Vector3d& p) {
    Eigen::Vector4d ph;
    ph << p, 1.0;
    Eigen::Vector4d result = T * ph;
    return result.head<3>();
}

Eigen::Vector3d transform_vector(const Eigen::Matrix4d& T, const Eigen::Vector3d& v) {
    return T.block<3, 3>(0, 0) * v;
}

Eigen::Matrix3d extract_rotation(const Eigen::Matrix4d& T) {
    return T.block<3, 3>(0, 0);
}

Eigen::Vector3d extract_translation(const Eigen::Matrix4d& T) {
    return T.block<3, 1>(0, 3);
}

}  // namespace caliburn
