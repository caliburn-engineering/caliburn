#pragma once

#include <Eigen/Core>
#include <functional>

namespace caliburn {

// --- Integrator interface ---
// Advances a state vector by one timestep using a derivative function.
class Integrator {
public:
    using DerivativeFn = std::function<Eigen::VectorXd(double, const Eigen::VectorXd&)>;

    virtual ~Integrator() = default;
    virtual Eigen::VectorXd step(const Eigen::VectorXd& state, double t, double dt,
                                 const DerivativeFn& f) = 0;
};

// --- Controller interface ---
// Computes a control output from a setpoint and measurement.
class Controller {
public:
    virtual ~Controller() = default;
    virtual Eigen::VectorXd compute(const Eigen::VectorXd& setpoint,
                                    const Eigen::VectorXd& measurement,
                                    double dt) = 0;
    virtual void reset() = 0;
};

// --- Observer interface ---
// Estimates system state from noisy measurements and known control input.
class Observer {
public:
    virtual ~Observer() = default;
    virtual Eigen::VectorXd update(const Eigen::VectorXd& measurement,
                                   const Eigen::VectorXd& control_input,
                                   double dt) = 0;
    virtual Eigen::VectorXd state() const = 0;
};

// --- Plant interface ---
// Defines system dynamics and sensor model.
class Plant {
public:
    virtual ~Plant() = default;
    virtual Eigen::VectorXd derivatives(double t, const Eigen::VectorXd& state,
                                        const Eigen::VectorXd& control_input) = 0;
    virtual Eigen::VectorXd observe(const Eigen::VectorXd& state) = 0;
};

// --- Renderer interface ---
// Visualises the current simulation state.
class Renderer {
public:
    virtual ~Renderer() = default;
    virtual void render(const Eigen::VectorXd& state, double alpha) = 0;
};

}  // namespace caliburn
