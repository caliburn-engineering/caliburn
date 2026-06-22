#pragma once

#include <functional>

namespace caliburn {

class FixedTimestep {
public:
    explicit FixedTimestep(double dt, double max_frame_dt = 0.25);

    // Accumulate frame_dt, execute step_fn for each substep. Returns substep count.
    int accumulate(double frame_dt, const std::function<void()>& step_fn);

    double alpha() const;      // interpolation factor for rendering [0, 1)
    double sim_time() const;
    double dt() const;

private:
    double dt_;
    double max_frame_dt_;
    double accumulator_ = 0.0;
    double sim_time_ = 0.0;
};

} // namespace caliburn
