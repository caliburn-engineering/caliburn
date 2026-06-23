#pragma once

#include <Eigen/Core>
#include <vector>

namespace caliburn {

struct JointLimit {
    int state_index;      // index into state vector for the position
    int velocity_index;   // index into state vector for the velocity (-1 if N/A)
    double min_pos;
    double max_pos;
    double max_velocity;  // absolute value, 0 = no velocity limit
};

class ConstraintEnforcer {
public:
    explicit ConstraintEnforcer(std::vector<JointLimit> limits);

    // Enforce all constraints on the state vector. Returns number of active constraints.
    int enforce(Eigen::VectorXd& state) const;

    // Clamp a scalar control signal to [min, max].
    static double clamp(double value, double min_val, double max_val);

    const std::vector<JointLimit>& limits() const;

private:
    std::vector<JointLimit> limits_;
};

}  // namespace caliburn
