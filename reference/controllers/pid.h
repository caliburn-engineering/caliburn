#pragma once

namespace caliburn {

struct PidGains {
    double Kp;
    double Ki;
    double Kd;
};

class PidController {
public:
    PidController(const PidGains& gains, double output_min, double output_max);

    double compute(double setpoint, double measurement, double dt);
    void reset();
    void set_gains(const PidGains& gains);
    const PidGains& gains() const;
    double integral() const;

private:
    PidGains gains_;
    double output_min_;
    double output_max_;
    double integral_ = 0.0;
    double prev_measurement_ = 0.0;
    double derivative_filtered_ = 0.0;
    bool first_call_ = true;

    static constexpr double kDerivativeAlpha = 0.1;
};

}  // namespace caliburn
