#pragma once

#include "fixed_timestep.h"

#include <functional>

namespace caliburn {

class SimLoop {
public:
    using PhysicsFn = std::function<void(double t, double dt)>;
    using RenderFn = std::function<void(double alpha)>;

    explicit SimLoop(double physics_dt, double max_frame_dt = 0.25);

    // Process one frame: run physics substeps, then call render with interpolation alpha
    void frame(double frame_dt, const PhysicsFn& physics, const RenderFn& render);

    // Run for a fixed duration (no rendering, useful for testing/offline sim)
    void run_offline(double duration, const PhysicsFn& physics);

    double sim_time() const;
    int last_substep_count() const;

private:
    FixedTimestep timestep_;
    int last_substeps_ = 0;
};

} // namespace caliburn
