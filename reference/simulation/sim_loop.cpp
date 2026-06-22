#include "sim_loop.h"

namespace caliburn {

SimLoop::SimLoop(double physics_dt, double max_frame_dt)
    : timestep_(physics_dt, max_frame_dt) {}

void SimLoop::frame(double frame_dt, const PhysicsFn& physics, const RenderFn& render) {
    double dt = timestep_.dt();
    last_substeps_ = timestep_.accumulate(frame_dt, [&]() {
        physics(timestep_.sim_time(), dt);
    });
    render(timestep_.alpha());
}

void SimLoop::run_offline(double duration, const PhysicsFn& physics) {
    double dt = timestep_.dt();
    // Compute total substeps from integer division to avoid floating-point drift.
    // Offline simulation has no spiral-of-death risk, so we bypass frame clamping.
    int total_steps = static_cast<int>(duration / dt + 0.5);

    for (int i = 0; i < total_steps; i++) {
        timestep_.accumulate(dt, [&]() {
            physics(timestep_.sim_time(), dt);
        });
    }
}

double SimLoop::sim_time() const {
    return timestep_.sim_time();
}

int SimLoop::last_substep_count() const {
    return last_substeps_;
}

} // namespace caliburn
