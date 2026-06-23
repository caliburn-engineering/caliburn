#include "constraint_enforcer.h"

#include <algorithm>
#include <cmath>

namespace caliburn {

ConstraintEnforcer::ConstraintEnforcer(std::vector<JointLimit> limits)
    : limits_(std::move(limits)) {}

int ConstraintEnforcer::enforce(Eigen::VectorXd& state) const {
    int active = 0;
    for (const auto& lim : limits_) {
        // Position clamping
        double& pos = state(lim.state_index);
        if (pos < lim.min_pos) {
            pos = lim.min_pos;
            if (lim.velocity_index >= 0) {
                state(lim.velocity_index) = 0.0;
            }
            ++active;
        } else if (pos > lim.max_pos) {
            pos = lim.max_pos;
            if (lim.velocity_index >= 0) {
                state(lim.velocity_index) = 0.0;
            }
            ++active;
        }

        // Velocity clamping
        if (lim.velocity_index >= 0 && lim.max_velocity > 0.0) {
            double& vel = state(lim.velocity_index);
            if (std::abs(vel) > lim.max_velocity) {
                vel = clamp(vel, -lim.max_velocity, lim.max_velocity);
                ++active;
            }
        }
    }
    return active;
}

double ConstraintEnforcer::clamp(double value, double min_val, double max_val) {
    return std::max(min_val, std::min(max_val, value));
}

const std::vector<JointLimit>& ConstraintEnforcer::limits() const {
    return limits_;
}

}  // namespace caliburn
