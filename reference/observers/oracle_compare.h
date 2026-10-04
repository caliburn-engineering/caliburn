#pragma once
#include <Eigen/Dense>
#include <cmath>

namespace caliburn {

// Returns true when every element of A and B agrees to within rtol relative
// to the larger magnitude.  Elements where both magnitudes are below abstol
// are considered equal.
inline bool matrices_close_rel(const Eigen::MatrixXd& A,
                                const Eigen::MatrixXd& B,
                                double rtol   = 1e-6,
                                double abstol = 1e-12) {
    if (A.rows() != B.rows() || A.cols() != B.cols()) return false;
    for (int i = 0; i < A.rows(); ++i) {
        for (int j = 0; j < A.cols(); ++j) {
            double scale = std::max({std::abs(A(i, j)), std::abs(B(i, j)), abstol});
            if (std::abs(A(i, j) - B(i, j)) > rtol * scale)
                return false;
        }
    }
    return true;
}

}  // namespace caliburn
