#pragma once
#include "trajectory_generator.h"
#include <algorithm>
#include <cassert>
#include <cmath>

/** Critically damped second-order position reference; bandwidth is in 1/s. */
class FilterTrajectory final : public TrajectoryGenerator {
    bool initialized = false;
    float bandwidth;
public:
    float goal = 0, reference = 0, velocity = 0;
    explicit FilterTrajectory(float bandwidth = 0) noexcept : bandwidth(bandwidth) {}

    bool configure(float value) {
        if (!(value > 0) || !std::isfinite(value)) return false;
        bandwidth = value;
        return true;
    }

    bool start(TrajectoryState initial, float target) override {
        if (!(bandwidth > 0) || !std::isfinite(bandwidth) || !std::isfinite(target) ||
            !std::isfinite(initial.position) || !std::isfinite(initial.velocity)) return false;
        goal = target;
        reference = initial.position;
        velocity = initial.velocity;
        initialized = true;
        return true;
    }
    bool retarget(TrajectoryState, float target) override {
        if (!initialized || !std::isfinite(target)) return false;
        goal = target;
        return true;
    }
    void on_update(const TrajectoryGenerator* active, float) override {
        if (active) {
            const auto state = active->get_state();
            reference = state.position;
            velocity = state.velocity;
        }
    }
    void on_activate(TrajectoryState initial) override {
        reference = initial.position;
        velocity = initial.velocity;
    }
    /** Integrate velocity first, then position, without measurement feedback. */
    float step(float dt) override {
        assert(initialized && dt > 0 && std::isfinite(dt));
        const float b = std::min(bandwidth, 0.25f / dt); // Effective bandwidth, 1/s.
        const float acceleration = b * b * (goal - reference) - 2 * b * velocity;
        velocity += acceleration * dt;
        reference += velocity * dt;
        return reference;
    }
    void reset() override {
        initialized = false;
        goal = reference = velocity = 0;
    }
    TrajectoryState get_state() const override { return {reference, velocity}; }
};
