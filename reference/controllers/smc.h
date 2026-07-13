#pragma once
#include <Eigen/Dense>
#include <functional>
#include <cmath>

namespace caliburn {

// Sliding surface function: takes state x, returns scalar s
using SlidingSurface = std::function<double(const Eigen::VectorXd& x)>;

// Equivalent control function: takes state x, returns u_eq
using EquivalentControl = std::function<double(const Eigen::VectorXd& x)>;

struct SMCParams {
    double K;           // switching gain (must exceed max disturbance)
    double phi;         // boundary layer thickness (0 = pure sign, >0 = sat)
    // Super-twisting parameters (optional)
    double k1 = 0.0;   // super-twisting gain 1
    double k2 = 0.0;   // super-twisting gain 2
};

enum class SMCMode {
    Sign,           // Pure sign(s) — aggressive, chatters
    BoundaryLayer,  // sat(s/phi) — smooth, approximate sliding
    SuperTwisting   // 2nd-order, continuous, no chattering
};

class SlidingModeController {
public:
    SlidingModeController(SMCParams params, SMCMode mode,
                          SlidingSurface surface,
                          EquivalentControl eq_control);

    // Compute control output given current state and timestep
    double compute(const Eigen::VectorXd& x, double dt);

    // Get the last computed sliding variable (for diagnostics)
    double getSlidingVariable() const;

    // Reset internal state (super-twisting integrator)
    void reset();

private:
    SMCParams params_;
    SMCMode mode_;
    SlidingSurface surface_;
    EquivalentControl eq_control_;
    double v_integral_ = 0.0;  // super-twisting integrator state
    double s_last_ = 0.0;
};

} // namespace caliburn
