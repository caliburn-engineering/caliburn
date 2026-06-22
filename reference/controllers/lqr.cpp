#include "lqr.h"
#include <stdexcept>
#include <complex>
#include <algorithm>
#include <vector>

namespace caliburn {

LqrResult dlqr(const Eigen::MatrixXd& A, const Eigen::MatrixXd& B,
               const Eigen::MatrixXd& Q, const Eigen::MatrixXd& R,
               int max_iter, double tol) {
    const int n = A.rows();

    Eigen::MatrixXd P = Q;

    for (int iter = 0; iter < max_iter; ++iter) {
        // P_next = A^T P A - A^T P B (R + B^T P B)^{-1} B^T P A + Q
        Eigen::MatrixXd BtP = B.transpose() * P;
        Eigen::MatrixXd BtPB = BtP * B;
        Eigen::MatrixXd S = R + BtPB;
        Eigen::MatrixXd AtP = A.transpose() * P;
        Eigen::MatrixXd AtPA = AtP * A;
        Eigen::MatrixXd AtPB = AtP * B;

        // Solve S * X = B^T P A  using Cholesky instead of explicit inverse
        Eigen::MatrixXd X = S.llt().solve(BtP * A);

        Eigen::MatrixXd P_next = AtPA - AtPB * X + Q;

        // Enforce symmetry
        P_next = 0.5 * (P_next + P_next.transpose());

        double diff = (P_next - P).norm();
        P = P_next;

        if (diff < tol) {
            // Compute gain: K = (R + B^T P B)^{-1} B^T P A
            Eigen::MatrixXd Sf = R + B.transpose() * P * B;
            Eigen::MatrixXd K = Sf.llt().solve(B.transpose() * P * A);
            return {K, P};
        }
    }

    throw std::runtime_error("dlqr: DARE did not converge within max_iter iterations");
}

LqrResult lqr(const Eigen::MatrixXd& A, const Eigen::MatrixXd& B,
              const Eigen::MatrixXd& Q, const Eigen::MatrixXd& R) {
    const int n = A.rows();

    // Solve R^{-1} via Cholesky for building the Hamiltonian
    Eigen::MatrixXd Rinv_Bt = R.llt().solve(B.transpose());
    Eigen::MatrixXd BR_invBt = B * Rinv_Bt;

    // Build Hamiltonian: H = [A, -B*R^{-1}*B^T; -Q, -A^T]
    Eigen::MatrixXd H(2 * n, 2 * n);
    H.topLeftCorner(n, n) = A;
    H.topRightCorner(n, n) = -BR_invBt;
    H.bottomLeftCorner(n, n) = -Q;
    H.bottomRightCorner(n, n) = -A.transpose();

    // Eigendecomposition of the Hamiltonian
    Eigen::ComplexEigenSolver<Eigen::MatrixXd> ces(H);
    if (ces.info() != Eigen::Success) {
        throw std::runtime_error("lqr: eigendecomposition of Hamiltonian failed");
    }

    auto eigenvalues = ces.eigenvalues();
    auto eigenvectors = ces.eigenvectors();

    // Select eigenvectors with eigenvalues that have negative real part (stable)
    std::vector<int> stable_indices;
    for (int i = 0; i < 2 * n; ++i) {
        if (eigenvalues(i).real() < 0.0) {
            stable_indices.push_back(i);
        }
    }

    if (static_cast<int>(stable_indices.size()) < n) {
        throw std::runtime_error(
            "lqr: fewer than n stable eigenvalues found in Hamiltonian");
    }

    // Build V from the n stable eigenvectors (each 2n-dimensional)
    Eigen::MatrixXcd V(2 * n, n);
    for (int j = 0; j < n; ++j) {
        V.col(j) = eigenvectors.col(stable_indices[j]);
    }

    // Extract U1 (top n rows) and U2 (bottom n rows)
    Eigen::MatrixXcd U1 = V.topRows(n);
    Eigen::MatrixXcd U2 = V.bottomRows(n);

    // P = real(U2 * U1^{-1})
    Eigen::MatrixXd P = (U2 * U1.inverse()).real();

    // Enforce symmetry
    P = 0.5 * (P + P.transpose());

    // K = R^{-1} B^T P  via Cholesky
    Eigen::MatrixXd K = R.llt().solve(B.transpose() * P);

    return {K, P};
}

} // namespace caliburn
