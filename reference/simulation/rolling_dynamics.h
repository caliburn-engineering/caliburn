#pragma once

#include <Eigen/Core>

namespace caliburn {

struct BallParams {
    double radius;           // ball radius [m]
    double mass;             // ball mass [kg]
    double rolling_friction; // coefficient of rolling resistance
};

struct PlateParams {
    double half_width;   // plate half-width [m] (square plate)
    double gravity;      // gravitational acceleration [m/s²]
};

class RollingBallDynamics {
public:
    RollingBallDynamics(const BallParams& ball, const PlateParams& plate);

    // Compute state derivative: state = [x, y, vx, vy], tilt = [alpha, beta] in radians.
    // Returns [vx, vy, ax, ay].
    Eigen::Vector4d derivatives(const Eigen::Vector4d& state,
                                double alpha, double beta) const;

    // Check if ball is on plate.
    bool on_plate(const Eigen::Vector4d& state) const;

    // The 5/7 effective acceleration factor for a uniform solid sphere.
    static constexpr double rolling_factor() { return 5.0 / 7.0; }

    const BallParams& ball_params() const;
    const PlateParams& plate_params() const;

private:
    BallParams ball_;
    PlateParams plate_;
};

}  // namespace caliburn
