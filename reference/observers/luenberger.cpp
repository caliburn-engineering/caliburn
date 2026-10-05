#include "luenberger.h"

namespace caliburn {

LuenbergerObserver::LuenbergerObserver(const Eigen::MatrixXd& A,
                                       const Eigen::MatrixXd& B,
                                       const Eigen::MatrixXd& C,
                                       const Eigen::MatrixXd& L)
    : A_(A), B_(B), C_(C), L_(L) {
    int n = A.rows();
    int m = C.rows();
    x_hat_ = Eigen::VectorXd::Zero(n);
    last_innovation_ = Eigen::VectorXd::Zero(m);
}

void LuenbergerObserver::update(const Eigen::VectorXd& u,
                                 const Eigen::VectorXd& y,
                                 double dt) {
    // Compute innovation: y - C * x_hat
    last_innovation_ = y - C_ * x_hat_;

    // Continuous-time observer equation discretized via forward Euler:
    // x_hat_dot = A*x_hat + B*u + L*(y - C*x_hat)
    // x_hat_new = x_hat + dt * x_hat_dot
    Eigen::VectorXd x_hat_dot = A_ * x_hat_ + B_ * u + L_ * last_innovation_;
    x_hat_ = x_hat_ + dt * x_hat_dot;
}

const Eigen::VectorXd& LuenbergerObserver::state() const {
    return x_hat_;
}

void LuenbergerObserver::set_state(const Eigen::VectorXd& x0) {
    x_hat_ = x0;
}

double LuenbergerObserver::errorNorm(const Eigen::VectorXd& x_true) const {
    return (x_true - x_hat_).norm();
}

Eigen::VectorXd LuenbergerObserver::innovation() const {
    return last_innovation_;
}

Eigen::MatrixXd placeObserverPoles(const Eigen::MatrixXd& A,
                                    const Eigen::MatrixXd& C,
                                    const Eigen::VectorXcd& desired_poles) {
    // Ackermann's formula for SISO systems (single-output), the dual of
    // controller placement: L^T = acker(A^T, C^T, desired_poles).
    //
    // desired_poles: eigenvalues of (A - LC)

    int n = A.rows();

    // Build observability matrix of (A, C) = controllability matrix of (A^T, C^T)
    Eigen::MatrixXd Ob(n * C.rows(), n);
    Eigen::MatrixXd CA = C;
    for (int i = 0; i < n; ++i) {
        Ob.block(i * C.rows(), 0, C.rows(), n) = CA;
        CA = CA * A;
    }

    // For SISO (single output), Ob is n x n
    // Compute characteristic polynomial coefficients of desired poles
    // alpha(s) = (s - p1)(s - p2)...(s - pn), stored highest power first:
    // poly_coeffs = [1, c_1, ..., c_n] for s^n + c_1*s^{n-1} + ... + c_n
    Eigen::VectorXcd poly_coeffs = Eigen::VectorXcd::Ones(1);
    for (int i = 0; i < n; ++i) {
        // Multiply polynomial by (s - p_i)
        Eigen::VectorXcd new_poly = Eigen::VectorXcd::Zero(poly_coeffs.size() + 1);
        for (int j = 0; j < poly_coeffs.size(); ++j) {
            new_poly(j) += poly_coeffs(j);
            new_poly(j + 1) -= desired_poles(i) * poly_coeffs(j);
        }
        poly_coeffs = new_poly;
    }

    // L = alpha(A) * O^{-1} * e_n, where O is the observability matrix.
    // poly_coeffs is highest power first, so Horner's rule gives
    // alpha(A) = A^n + c_1*A^{n-1} + ... + c_n*I. The coefficients are real
    // when complex poles come in conjugate pairs.
    Eigen::VectorXd e_n = Eigen::VectorXd::Zero(n);
    e_n(n - 1) = 1.0;

    Eigen::MatrixXd alpha_A = Eigen::MatrixXd::Identity(n, n) * poly_coeffs(0).real();
    for (int i = 1; i <= n; ++i) {
        alpha_A = alpha_A * A + poly_coeffs(i).real() * Eigen::MatrixXd::Identity(n, n);
    }

    Eigen::VectorXd L_vec = alpha_A * Ob.inverse() * e_n;

    // Return as column vector (n x 1 gain matrix for SISO)
    return L_vec;
}

}  // namespace caliburn
