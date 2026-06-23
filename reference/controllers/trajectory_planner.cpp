#include "trajectory_planner.h"

#include <algorithm>
#include <cmath>

namespace caliburn {

// ---------------------------------------------------------------------------
// CubicTrajectory
// ---------------------------------------------------------------------------
CubicTrajectory::CubicTrajectory(double q0, double v0, double qf, double vf, double T)
    : T_(T) {
    a0_ = q0;
    a1_ = v0;
    a2_ = (3.0 * (qf - q0) - (2.0 * v0 + vf) * T) / (T * T);
    a3_ = (2.0 * (q0 - qf) + (v0 + vf) * T) / (T * T * T);
}

double CubicTrajectory::position(double t) const {
    t = std::clamp(t, 0.0, T_);
    return a0_ + a1_ * t + a2_ * t * t + a3_ * t * t * t;
}

double CubicTrajectory::velocity(double t) const {
    t = std::clamp(t, 0.0, T_);
    return a1_ + 2.0 * a2_ * t + 3.0 * a3_ * t * t;
}

double CubicTrajectory::acceleration(double t) const {
    t = std::clamp(t, 0.0, T_);
    return 2.0 * a2_ + 6.0 * a3_ * t;
}

double CubicTrajectory::duration() const { return T_; }

// ---------------------------------------------------------------------------
// MinJerkTrajectory
// ---------------------------------------------------------------------------
MinJerkTrajectory::MinJerkTrajectory(double q0, double qf, double T)
    : q0_(q0), qf_(qf), T_(T) {}

double MinJerkTrajectory::position(double t) const {
    t = std::clamp(t, 0.0, T_);
    double tau = t / T_;
    double tau3 = tau * tau * tau;
    double tau4 = tau3 * tau;
    double tau5 = tau4 * tau;
    double s = 10.0 * tau3 - 15.0 * tau4 + 6.0 * tau5;
    return q0_ + (qf_ - q0_) * s;
}

double MinJerkTrajectory::velocity(double t) const {
    t = std::clamp(t, 0.0, T_);
    double tau = t / T_;
    double tau2 = tau * tau;
    double tau3 = tau2 * tau;
    double tau4 = tau3 * tau;
    double ds = 30.0 * tau2 - 60.0 * tau3 + 30.0 * tau4;
    return (qf_ - q0_) * ds / T_;
}

double MinJerkTrajectory::acceleration(double t) const {
    t = std::clamp(t, 0.0, T_);
    double tau = t / T_;
    double tau2 = tau * tau;
    double tau3 = tau2 * tau;
    double dds = 60.0 * tau - 180.0 * tau2 + 120.0 * tau3;
    return (qf_ - q0_) * dds / (T_ * T_);
}

double MinJerkTrajectory::duration() const { return T_; }

// ---------------------------------------------------------------------------
// TrapezoidalTrajectory
// ---------------------------------------------------------------------------
TrapezoidalTrajectory::TrapezoidalTrajectory(double q0, double qf,
                                             double v_max, double a_max)
    : q0_(q0), qf_(qf), v_max_(v_max), a_max_(a_max) {
    double dist = std::abs(qf - q0);

    // Distance needed to reach v_max (accel + decel phases only)
    double dist_to_cruise = v_max * v_max / a_max;

    if (dist < dist_to_cruise) {
        // Triangular profile: never reaches v_max
        triangular_ = true;
        v_max_ = std::sqrt(dist * a_max);
        t_a_ = v_max_ / a_max;
        T_ = 2.0 * t_a_;
    } else {
        // Trapezoidal profile
        triangular_ = false;
        t_a_ = v_max / a_max;
        double cruise_dist = dist - dist_to_cruise;
        double cruise_time = cruise_dist / v_max;
        T_ = 2.0 * t_a_ + cruise_time;
    }
}

double TrapezoidalTrajectory::position(double t) const {
    t = std::clamp(t, 0.0, T_);
    double sign = (qf_ > q0_) ? 1.0 : -1.0;
    double a = sign * a_max_;
    double v = sign * v_max_;

    if (t <= t_a_) {
        // Acceleration phase
        return q0_ + 0.5 * a * t * t;
    } else if (t <= T_ - t_a_) {
        // Cruise phase
        double q_a = q0_ + 0.5 * a * t_a_ * t_a_;
        return q_a + v * (t - t_a_);
    } else {
        // Deceleration phase
        double dt = T_ - t;
        return qf_ - 0.5 * a * dt * dt;
    }
}

double TrapezoidalTrajectory::velocity(double t) const {
    t = std::clamp(t, 0.0, T_);
    double sign = (qf_ > q0_) ? 1.0 : -1.0;
    double a = sign * a_max_;
    double v = sign * v_max_;

    if (t <= t_a_) {
        return a * t;
    } else if (t <= T_ - t_a_) {
        return v;
    } else {
        return a * (T_ - t);
    }
}

double TrapezoidalTrajectory::acceleration(double t) const {
    t = std::clamp(t, 0.0, T_);
    double sign = (qf_ > q0_) ? 1.0 : -1.0;

    if (t < t_a_) {
        return sign * a_max_;
    } else if (t < T_ - t_a_) {
        return 0.0;
    } else {
        return -sign * a_max_;
    }
}

double TrapezoidalTrajectory::duration() const { return T_; }

}  // namespace caliburn
