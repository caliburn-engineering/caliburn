#pragma once

namespace caliburn {

// Cubic polynomial trajectory: q(t) with position + velocity BCs at t=0 and t=T.
class CubicTrajectory {
public:
    CubicTrajectory(double q0, double v0, double qf, double vf, double T);

    double position(double t) const;
    double velocity(double t) const;
    double acceleration(double t) const;
    double duration() const;

private:
    double a0_, a1_, a2_, a3_;
    double T_;
};

// Minimum-jerk (5th-order) trajectory: rest-to-rest.
class MinJerkTrajectory {
public:
    MinJerkTrajectory(double q0, double qf, double T);

    double position(double t) const;
    double velocity(double t) const;
    double acceleration(double t) const;
    double duration() const;

private:
    double q0_, qf_, T_;
};

// Trapezoidal velocity profile: constant accel → cruise → constant decel.
class TrapezoidalTrajectory {
public:
    TrapezoidalTrajectory(double q0, double qf, double v_max, double a_max);

    double position(double t) const;
    double velocity(double t) const;
    double acceleration(double t) const;
    double duration() const;

private:
    double q0_, qf_;
    double v_max_, a_max_;
    double t_a_;     // acceleration phase duration
    double T_;       // total duration
    bool triangular_; // true if cruise phase vanishes
};

}  // namespace caliburn
