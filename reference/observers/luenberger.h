#pragma once

#include <Eigen/Dense>

namespace caliburn {

class LuenbergerObserver {
public:
    // A: state matrix, B: input matrix, C: output matrix, L: observer gain
    LuenbergerObserver(const Eigen::MatrixXd& A,
                       const Eigen::MatrixXd& B,
                       const Eigen::MatrixXd& C,
                       const Eigen::MatrixXd& L);

    // Update estimate given current input u and measurement y
    void update(const Eigen::VectorXd& u, const Eigen::VectorXd& y, double dt);

    // Get current state estimate
    const Eigen::VectorXd& state() const;

    // Set state estimate (for initialization)
    void set_state(const Eigen::VectorXd& x0);

    // Get estimation error norm (for diagnostics, requires true state)
    double errorNorm(const Eigen::VectorXd& x_true) const;

    // Get last innovation (y - C*x_hat) for diagnostics
    Eigen::VectorXd innovation() const;

private:
    Eigen::MatrixXd A_, B_, C_, L_;
    Eigen::VectorXd x_hat_;
    Eigen::VectorXd last_innovation_;
};

// Utility: compute observer gain L for desired pole locations
// (Ackermann's formula for SISO, place() equivalent)
Eigen::MatrixXd placeObserverPoles(const Eigen::MatrixXd& A,
                                    const Eigen::MatrixXd& C,
                                    const Eigen::VectorXcd& desired_poles);

}  // namespace caliburn
