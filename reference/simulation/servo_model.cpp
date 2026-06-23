#include "servo_model.h"

#include <algorithm>
#include <cmath>

namespace caliburn {

ServoModel::ServoModel(const ServoParams& params, double initial_angle)
    : params_(params), theta_(initial_angle), omega_(0.0) {}

double ServoModel::step(double theta_cmd, double dt) {
    // Clamp command to joint limits
    theta_cmd = std::clamp(theta_cmd, params_.theta_min, params_.theta_max);

    // Compute effective error (with dead zone)
    double error = params_.K * theta_cmd - theta_;
    if (params_.dead_zone > 0.0 && std::abs(error) < params_.dead_zone) {
        error = 0.0;
    } else if (params_.dead_zone > 0.0) {
        error -= std::copysign(params_.dead_zone, error);
    }

    // First-order dynamics: omega = error / tau
    omega_ = error / params_.tau;

    // Velocity saturation
    omega_ = std::clamp(omega_, -params_.omega_max, params_.omega_max);

    // Integrate (forward Euler)
    theta_ += omega_ * dt;

    // Position clamping (safety)
    theta_ = std::clamp(theta_, params_.theta_min, params_.theta_max);

    return theta_;
}

double ServoModel::angle() const { return theta_; }

double ServoModel::angular_velocity() const { return omega_; }

void ServoModel::reset(double angle) {
    theta_ = angle;
    omega_ = 0.0;
}

const ServoParams& ServoModel::params() const { return params_; }

}  // namespace caliburn
