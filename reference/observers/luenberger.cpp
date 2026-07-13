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
    // Ackermann's formula for SISO systems (single-output)
    // For the observer: we work with A^T and C^T (duality with controller)
    //
    // Observer pole placement via controllability of (A^T, C^T):
    //   L^T = acker(A^T, C^T, desired_poles)
    //
    // desired_poles: eigenvalues of (A - LC)

    int n = A.rows();

    // Compute the desired characteristic polynomial coefficients
    // p(s) = (s - p1)(s - p2)...(s - pn)
    // We evaluate p(A^T) using the Cayley-Hamilton approach

    Eigen::MatrixXd At = A.transpose();
    Eigen::MatrixXd Ct = C.transpose();

    // Build observability matrix of (A, C) = controllability matrix of (A^T, C^T)
    Eigen::MatrixXd Ob(n * C.rows(), n);
    Eigen::MatrixXd CA = C;
    for (int i = 0; i < n; ++i) {
        Ob.block(i * C.rows(), 0, C.rows(), n) = CA;
        CA = CA * A;
    }

    // For SISO (single output), Ob is n x n
    // Compute characteristic polynomial coefficients of desired poles
    // alpha(s) = s^n + a_{n-1}*s^{n-1} + ... + a_0
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

    // Evaluate alpha(A^T) using Horner's method
    Eigen::MatrixXcd alpha_At = Eigen::MatrixXcd::Zero(n, n);
    alpha_At.diagonal().setConstant(poly_coeffs(0));
    Eigen::MatrixXcd At_c = At.cast<std::complex<double>>();
    for (int i = 1; i <= n; ++i) {
        alpha_At = alpha_At * At_c +
                   poly_coeffs(i) * Eigen::MatrixXcd::Identity(n, n);
    }

    // L^T = last row of Ob^{-1} * alpha(A^T)
    // For SISO: L = (Ob^{-1} * alpha(A) * e_n) where e_n = [0,...,0,1]^T
    // Using duality: L^T = e_n^T * controllability_matrix^{-1} * alpha(A^T)
    Eigen::VectorXd e_n = Eigen::VectorXd::Zero(n);
    e_n(n - 1) = 1.0;

    Eigen::MatrixXd Ob_inv = Ob.inverse();
    Eigen::MatrixXd alpha_A_real = (alpha_At).real();

    // Ackermann: L = alpha(A) * O^{-1} * e_n
    // Where O is observability matrix and alpha is desired char poly evaluated at A
    Eigen::MatrixXd alpha_A = Eigen::MatrixXd::Identity(n, n) * poly_coeffs(0).real();
    Eigen::MatrixXd A_power = Eigen::MatrixXd::Identity(n, n);
    for (int i = 1; i <= n; ++i) {
        A_power = A_power * A;
        alpha_A += poly_coeffs(i).real() * A_power;
    }

    Eigen::VectorXd L_vec = alpha_A * Ob.inverse() * e_n;

    // Return as column vector (n x 1 gain matrix for SISO)
    return L_vec;
}

}  // namespace caliburn
