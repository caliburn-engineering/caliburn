#pragma once
#include <Eigen/Dense>

namespace caliburn {

struct LqrResult {
    Eigen::MatrixXd K;  // feedback gain matrix
    Eigen::MatrixXd P;  // Riccati equation solution
};

// Discrete-time LQR via iterative DARE solution
// A: state transition (n×n), B: input matrix (n×m)
// Q: state cost (n×n, PSD), R: input cost (m×m, PD)
LqrResult dlqr(const Eigen::MatrixXd& A, const Eigen::MatrixXd& B,
               const Eigen::MatrixXd& Q, const Eigen::MatrixXd& R,
               int max_iter = 5000, double tol = 1e-10);

// Continuous-time LQR via Hamiltonian eigendecomposition
LqrResult lqr(const Eigen::MatrixXd& A, const Eigen::MatrixXd& B,
              const Eigen::MatrixXd& Q, const Eigen::MatrixXd& R);

} // namespace caliburn
