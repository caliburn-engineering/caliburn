#include "smc.h"
#include <algorithm>

namespace caliburn {

SlidingModeController::SlidingModeController(SMCParams params, SMCMode mode,
                                             SlidingSurface surface,
                                             EquivalentControl eq_control)
    : params_(params), mode_(mode),
      surface_(std::move(surface)), eq_control_(std::move(eq_control)) {}

double SlidingModeController::compute(const Eigen::VectorXd& x, double dt) {
    double s = surface_(x);
    s_last_ = s;

    double u_eq = eq_control_(x);
    double u_sw = 0.0;

    switch (mode_) {
        case SMCMode::Sign:
            // Pure discontinuous control: u_sw = -K * sign(s)
            if (s > 0.0) u_sw = -params_.K;
            else if (s < 0.0) u_sw = params_.K;
            else u_sw = 0.0;
            break;

        case SMCMode::BoundaryLayer:
            // Continuous approximation: u_sw = -K * sat(s/phi)
            if (params_.phi > 0.0) {
                double sat = std::clamp(s / params_.phi, -1.0, 1.0);
                u_sw = -params_.K * sat;
            } else {
                // Degenerate to sign if phi = 0
                if (s > 0.0) u_sw = -params_.K;
                else if (s < 0.0) u_sw = params_.K;
                else u_sw = 0.0;
            }
            break;

        case SMCMode::SuperTwisting:
            // u = -k1 * |s|^(1/2) * sign(s) + v
            // v_dot = -k2 * sign(s)
            {
                double abs_s = std::abs(s);
                double sign_s = (s > 0.0) ? 1.0 : ((s < 0.0) ? -1.0 : 0.0);
                u_sw = -params_.k1 * std::sqrt(abs_s) * sign_s + v_integral_;
                v_integral_ += -params_.k2 * sign_s * dt;
            }
            break;
    }

    return u_eq + u_sw;
}

double SlidingModeController::getSlidingVariable() const {
    return s_last_;
}

void SlidingModeController::reset() {
    v_integral_ = 0.0;
    s_last_ = 0.0;
}

} // namespace caliburn
