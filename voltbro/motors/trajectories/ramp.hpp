#pragma once
#include "trajectory_generator.h"
#include <algorithm>
#include <cassert>
#include <cmath>

/** Velocity reference limited by rate in position units/second squared. */
class RampTrajectory final : public TrajectoryGenerator {
    bool initialized = false;
    float rate;
public:
    float goal = 0, reference = 0;
    explicit RampTrajectory(float rate = 0) noexcept : rate(rate) {}

    bool configure(float value) {
        if (!(value > 0) || !std::isfinite(value)) return false;
        rate = value;
        return true;
    }

    bool start(TrajectoryState initial, float target) override {
        if (!(rate > 0) || !std::isfinite(rate) || !std::isfinite(target) ||
            !std::isfinite(initial.position) || !std::isfinite(initial.velocity)) return false;
        goal = target;
        reference = initial.velocity;
        initialized = true;
        return true;
    }
    bool retarget(TrajectoryState, float target) override {
        if (!initialized || !std::isfinite(target)) return false;
        goal = target;
        return true;
    }
    void on_publish(const TrajectoryGenerator* active, float) override {
        if (active) {
            const auto& current = static_cast<const RampTrajectory&>(*active);
            reference = current.reference;
        }
    }
    void on_activate(TrajectoryState initial) override {
        reference = initial.velocity;
    }
    /** Move the reference toward its target by at most rate * dt. */
    float step(float dt) override {
        assert(initialized && dt > 0 && std::isfinite(dt));
        reference += std::clamp(goal - reference, -rate * dt, rate * dt);
        return reference;
    }
    void reset() override {
        initialized = false;
        goal = reference = 0;
    }
};
