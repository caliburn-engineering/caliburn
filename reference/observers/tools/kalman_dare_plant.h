// reference/observers/tools/kalman_dare_plant.h
//
// The plant the Kalman DARE fixture is solved for, and its hash.  One
// definition, shared by the exporter that hands the plant to the oracle and the
// default-suite test that checks the filter against the oracle's answer, so the
// two cannot drift apart.  Ported from ball-balancer's tools/plant_hash.h.
//
// The plant is the double mass-spring-damper from reference/models/ at its
// default parameters, discretised by zero-order hold (F = exp(A dt)), observed
// at both mass positions (H = C), with white process and measurement noise.
//
// The hash is a DJB2-64 digest of F, H, Q and R as full-precision (%.17g)
// MATLAB-style strings joined by "|".  It covers the matrices, not the
// parameters, so a change to the model's equations is caught as well as a
// change to its numbers.

#pragma once

#include "double_mass_spring_damper.h"

#include <Eigen/Dense>
#include <unsupported/Eigen/MatrixFunctions>
#include <cstdint>
#include <cstdio>
#include <string>

namespace caliburn::kalman_dare {

constexpr double kDt = 0.01;           // [s]
constexpr double kProcessNoise = 1e-4;  // Q = kProcessNoise * I4
constexpr double kMeasNoise = 1e-2;     // R = kMeasNoise * I2

struct Plant {
    Eigen::Matrix4d F;
    Eigen::Matrix<double, 2, 4> H;
    Eigen::Matrix4d Q;
    Eigen::Matrix2d R;
};

inline Plant dmsdPlant() {
    const DoubleMassSpringDamperModel model =
        build_double_msd(DoubleMassSpringDamperParams{});
    Plant p;
    p.F = (model.A * kDt).exp();
    p.H = model.C;
    p.Q = kProcessNoise * Eigen::Matrix4d::Identity();
    p.R = kMeasNoise * Eigen::Matrix2d::Identity();
    return p;
}

inline std::string matrixFull(const Eigen::MatrixXd& m) {
    std::string s;
    char buf[32];
    for (int r = 0; r < m.rows(); ++r) {
        if (r > 0) s += "; ";
        for (int c = 0; c < m.cols(); ++c) {
            if (c > 0) s += " ";
            std::snprintf(buf, sizeof(buf), "%.17g", m(r, c));
            s += buf;
        }
    }
    return s;
}

inline uint64_t djb2_64(const std::string& s) {
    uint64_t h = 5381;
    for (unsigned char c : s) h = h * 33u + c;
    return h;
}

inline uint64_t plantHash(const Plant& p) {
    return djb2_64(matrixFull(p.F) + "|" + matrixFull(p.H) + "|" +
                   matrixFull(p.Q) + "|" + matrixFull(p.R));
}

}  // namespace caliburn::kalman_dare
