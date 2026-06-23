#pragma once

namespace caliburn {

struct ServoParams {
    double tau;            // time constant [s]
    double K;              // steady-state gain (typically 1.0)
    double omega_max;      // max angular velocity [rad/s]
    double theta_min;      // min angle [rad]
    double theta_max;      // max angle [rad]
    double dead_zone;      // dead zone half-width [rad], 0 = disabled
};

class ServoModel {
public:
    explicit ServoModel(const ServoParams& params, double initial_angle = 0.0);

    // Advance one timestep given commanded angle. Returns new angle.
    double step(double theta_cmd, double dt);

    double angle() const;
    double angular_velocity() const;
    void reset(double angle = 0.0);
    const ServoParams& params() const;

private:
    ServoParams params_;
    double theta_;
    double omega_;
};

}  // namespace caliburn
