#include "fixed_timestep.h"

#include <algorithm>

namespace caliburn {

FixedTimestep::FixedTimestep(double dt, double max_frame_dt)
    : dt_(dt), max_frame_dt_(max_frame_dt) {}

int FixedTimestep::accumulate(double frame_dt, const std::function<void()>& step_fn) {
    // Clamp to prevent spiral of death
    accumulator_ += std::min(frame_dt, max_frame_dt_);

    int substeps = 0;
    while (accumulator_ >= dt_) {
        step_fn();
        sim_time_ += dt_;
        accumulator_ -= dt_;
        substeps++;
    }
    return substeps;
}

double FixedTimestep::alpha() const {
    return accumulator_ / dt_;
}

double FixedTimestep::sim_time() const {
    return sim_time_;
}

double FixedTimestep::dt() const {
    return dt_;
}

} // namespace caliburn
