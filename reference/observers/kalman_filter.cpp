#include "kalman_filter.h"

namespace caliburn {

KalmanFilter::KalmanFilter(int state_dim, int measurement_dim)
    : n_(state_dim), m_(measurement_dim) {
    x_ = Eigen::VectorXd::Zero(n_);
    P_ = Eigen::MatrixXd::Identity(n_, n_);
    F_ = Eigen::MatrixXd::Identity(n_, n_);
    H_ = Eigen::MatrixXd::Zero(m_, n_);
    Q_ = Eigen::MatrixXd::Identity(n_, n_);
    R_ = Eigen::MatrixXd::Identity(m_, m_);
    B_ = Eigen::MatrixXd::Zero(n_, 1);
    last_innovation_ = Eigen::VectorXd::Zero(m_);
}

void KalmanFilter::set_model(const Eigen::MatrixXd& F, const Eigen::MatrixXd& H,
                              const Eigen::MatrixXd& Q, const Eigen::MatrixXd& R) {
    F_ = F;
    H_ = H;
    Q_ = Q;
    R_ = R;
}

void KalmanFilter::set_control_model(const Eigen::MatrixXd& B) {
    B_ = B;
    has_control_ = true;
}

void KalmanFilter::set_state(const Eigen::VectorXd& x, const Eigen::MatrixXd& P) {
    x_ = x;
    P_ = P;
}

void KalmanFilter::predict() {
    // State prediction (no control input)
    x_ = F_ * x_;

    // Covariance prediction
    P_ = F_ * P_ * F_.transpose() + Q_;
}

void KalmanFilter::predict(const Eigen::VectorXd& u) {
    // State prediction with control input
    if (has_control_) {
        x_ = F_ * x_ + B_ * u;
    } else {
        x_ = F_ * x_;
    }

    // Covariance prediction
    P_ = F_ * P_ * F_.transpose() + Q_;
}

void KalmanFilter::update(const Eigen::VectorXd& z) {
    // Innovation
    Eigen::VectorXd y = z - H_ * x_;
    last_innovation_ = y;

    // Innovation covariance
    Eigen::MatrixXd S = H_ * P_ * H_.transpose() + R_;

    // Kalman gain via Cholesky solve (avoids explicit inverse)
    // K = P * H^T * S^{-1}  computed as  K = S.llt().solve(H * P).transpose()
    Eigen::MatrixXd K = S.llt().solve(H_ * P_).transpose();

    // State update
    x_ = x_ + K * y;

    // Joseph form covariance update (numerically stable)
    Eigen::MatrixXd I_n = Eigen::MatrixXd::Identity(n_, n_);
    Eigen::MatrixXd A = I_n - K * H_;
    P_ = A * P_ * A.transpose() + K * R_ * K.transpose();

    // Enforce symmetry
    P_ = 0.5 * (P_ + P_.transpose());
}

const Eigen::VectorXd& KalmanFilter::state() const {
    return x_;
}

const Eigen::MatrixXd& KalmanFilter::covariance() const {
    return P_;
}

Eigen::VectorXd KalmanFilter::innovation() const {
    return last_innovation_;
}

}  // namespace caliburn
