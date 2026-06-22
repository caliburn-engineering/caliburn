#pragma once

#include <Eigen/Dense>

namespace caliburn {

class KalmanFilter {
public:
    KalmanFilter(int state_dim, int measurement_dim);

    void set_model(const Eigen::MatrixXd& F, const Eigen::MatrixXd& H,
                   const Eigen::MatrixXd& Q, const Eigen::MatrixXd& R);
    void set_control_model(const Eigen::MatrixXd& B);
    void set_state(const Eigen::VectorXd& x, const Eigen::MatrixXd& P);

    void predict();                          // no control input
    void predict(const Eigen::VectorXd& u);  // with control input
    void update(const Eigen::VectorXd& z);

    const Eigen::VectorXd& state() const;
    const Eigen::MatrixXd& covariance() const;
    Eigen::VectorXd innovation() const;  // last innovation (for diagnostics)

private:
    int n_, m_;  // state dim, measurement dim
    Eigen::VectorXd x_;
    Eigen::MatrixXd P_;
    Eigen::MatrixXd F_, H_, Q_, R_;
    Eigen::MatrixXd B_;  // optional control input matrix
    bool has_control_ = false;
    Eigen::VectorXd last_innovation_;
};

}  // namespace caliburn
