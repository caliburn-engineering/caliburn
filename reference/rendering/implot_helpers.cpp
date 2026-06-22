// implot_helpers.cpp — Real-time plotting helpers for engineering simulations
//
// Provides scrolling time-series buffers and convenience wrappers for common
// control-system visualization patterns: state trajectories, control signals,
// error plots, and phase portraits.

#include "implot.h"
#include "imgui.h"
#include <vector>
#include <string>
#include <algorithm>
#include <cmath>

namespace caliburn {

// Rolling buffer for real-time time-series data.
// Keeps the last `max_size` samples in a circular buffer.
struct ScrollingBuffer {
    int max_size;
    int offset = 0;
    std::vector<float> time;
    std::vector<float> data;

    explicit ScrollingBuffer(int max_size = 2000)
        : max_size(max_size) {
        time.reserve(max_size);
        data.reserve(max_size);
    }

    void push(float t, float value) {
        if (static_cast<int>(time.size()) < max_size) {
            time.push_back(t);
            data.push_back(value);
        } else {
            time[offset] = t;
            data[offset] = value;
            offset = (offset + 1) % max_size;
        }
    }

    void clear() {
        time.clear();
        data.clear();
        offset = 0;
    }

    int size() const { return static_cast<int>(time.size()); }
};

// Plot a scrolling time series in the current ImPlot context.
// Call between ImPlot::BeginPlot() and ImPlot::EndPlot().
void plot_scrolling(const char* label, const ScrollingBuffer& buf) {
    if (buf.size() == 0) return;
    ImPlot::PlotLine(label,
                     buf.time.data(), buf.data.data(),
                     buf.size(), 0, buf.offset, sizeof(float));
}

// Plot multiple channels on the same axes with auto-scrolling X axis.
// `window` is the visible time window in seconds.
void plot_time_series(const char* title,
                      const std::vector<const ScrollingBuffer*>& buffers,
                      const std::vector<const char*>& labels,
                      float current_time, float window = 10.0f) {
    ImPlot::SetNextAxesLimits(
        current_time - window, current_time,
        -1.0, 1.0, ImPlotCond_Always);

    if (ImPlot::BeginPlot(title, ImVec2(-1, 200))) {
        for (size_t i = 0; i < buffers.size() && i < labels.size(); ++i) {
            plot_scrolling(labels[i], *buffers[i]);
        }
        ImPlot::EndPlot();
    }
}

// Plot a 2D phase portrait (state vs. derivative or two states).
void plot_phase_portrait(const char* title,
                         const ScrollingBuffer& x_buf,
                         const ScrollingBuffer& y_buf,
                         const char* x_label, const char* y_label) {
    if (x_buf.size() == 0 || y_buf.size() == 0) return;

    if (ImPlot::BeginPlot(title, ImVec2(-1, 300))) {
        ImPlot::SetupAxes(x_label, y_label);
        int n = std::min(x_buf.size(), y_buf.size());
        ImPlot::PlotLine("trajectory",
                         x_buf.data.data(), y_buf.data.data(),
                         n, 0, x_buf.offset, sizeof(float));
        ImPlot::EndPlot();
    }
}

// Render a compact status panel showing key simulation values.
void status_panel(const char* title,
                  const std::vector<std::pair<std::string, float>>& values) {
    if (ImGui::Begin(title)) {
        for (const auto& [name, value] : values) {
            ImGui::Text("%s: %.4f", name.c_str(), value);
        }
    }
    ImGui::End();
}

}  // namespace caliburn
