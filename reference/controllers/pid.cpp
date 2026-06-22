#include "pid.h"

#include <algorithm>

namespace caliburn {

PidController::PidController(const PidGains& gains, double output_min, double output_max)
    : gains_(gains), output_min_(output_min), output_max_(output_max) {}

double PidController::compute(double setpoint, double measurement, double dt) {
    if (dt <= 0.0) {
        return 0.0;
    }

    double error = setpoint - measurement;

    // --- Proportional ---
    double p_term = gains_.Kp * error;

    // --- Integral with anti-windup clamping ---
    integral_ += error * dt;

    if (gains_.Ki != 0.0) {
        double integral_min = output_min_ / gains_.Ki;
        double integral_max = output_max_ / gains_.Ki;
        integral_ = std::clamp(integral_, integral_min, integral_max);
    }

    double i_term = gains_.Ki * integral_;

    // --- Derivative on measurement (not error) to avoid derivative kick ---
    double d_term = 0.0;
    if (!first_call_) {
        double raw_derivative = -(measurement - prev_measurement_) / dt;
        // Exponential low-pass filter
        derivative_filtered_ = kDerivativeAlpha * raw_derivative
                              + (1.0 - kDerivativeAlpha) * derivative_filtered_;
        d_term = gains_.Kd * derivative_filtered_;
    } else {
        first_call_ = false;
    }

    prev_measurement_ = measurement;

    // --- Output clamping ---
    double output = p_term + i_term + d_term;
    output = std::clamp(output, output_min_, output_max_);

    return output;
}

void PidController::reset() {
    integral_ = 0.0;
    prev_measurement_ = 0.0;
    derivative_filtered_ = 0.0;
    first_call_ = true;
}

void PidController::set_gains(const PidGains& gains) {
    gains_ = gains;
}

const PidGains& PidController::gains() const {
    return gains_;
}

double PidController::integral() const {
    return integral_;
}

}  // namespace caliburn
