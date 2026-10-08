#pragma once
#include "trajectory_generator.h"
#include <algorithm>
#include <cassert>
#include <cmath>
#include <cstdint>

/** Piecewise constant acceleration, trapezoidal/triangular velocity reference. */
class PolyTrajectory final : public TrajectoryGenerator {
    bool initialized = false;
    float time_error = 0; // Rounding residual for accumulated elapsed seconds.
    float velocity_limit, acceleration_limit, deceleration_limit;
    struct Phase {
        float duration = 0, position = 0, velocity = 0, acceleration = 0;
    };
    Phase phases[4]{};
    /** Build phases on a candidate; start publishes only a successful plan. */
    bool plan(float measured_position, float measured_velocity) {
        const float velocity_limit = this->velocity_limit; // Cruise speed magnitude, position units/s.
        const float acceleration_limit = this->acceleration_limit; // Acceleration magnitude, position units/s^2.
        const float deceleration_limit = this->deceleration_limit; // Deceleration magnitude, position units/s^2.
        if (!(velocity_limit > 0 && acceleration_limit > 0 && deceleration_limit > 0)) return false;
        uint8_t phase_count = 0; // Number of planned phases.
        float position = measured_position; // Reference position at the next phase, position units.
        float velocity = measured_velocity; // Reference speed at the next phase, position units/s.
        finish_time = 0;
        auto append_phase = [&](float duration, float acceleration) {
            if (duration <= 0) return;
            phases[phase_count++] = {duration, position, velocity, acceleration};
            position += velocity * duration + 0.5f * acceleration * duration * duration;
            velocity += acceleration * duration;
            finish_time += duration;
        };

        // First remove motion away from the goal, excessive speed, or speed
        // that cannot be stopped before the goal. A stop beyond the goal is
        // followed by a new profile in the opposite direction.
        float direction = goal >= position ? 1.0f : -1.0f; // Planned travel sign.
        if (goal == position && velocity != 0) direction = velocity > 0 ? -1.0f : 1.0f;
        const float directed_velocity = direction * velocity; // Speed toward goal, position units/s.
        const float directed_distance = direction * (goal - position); // Distance toward goal, position units.
        if (directed_velocity < 0 ||
            directed_distance < directed_velocity * directed_velocity / (2 * deceleration_limit)) {
            append_phase(std::fabs(velocity) / deceleration_limit,
                         -std::copysign(deceleration_limit, velocity));
            direction = goal >= position ? 1.0f : -1.0f;
        } else if (directed_velocity > velocity_limit) {
            append_phase((directed_velocity - velocity_limit) / deceleration_limit,
                         -direction * deceleration_limit);
        }

        // Solve the triangular or trapezoidal profile from the conditioned
        // initial velocity; each phase is integrated without motor feedback.
        const float distance = std::max(0.0f, direction * (goal - position)); // Remaining travel, position units.
        const float initial_speed = std::max(0.0f, direction * velocity); // Speed toward goal, position units/s.
        const float full_accel_distance = (velocity_limit * velocity_limit - initial_speed * initial_speed) /
                                          (2 * acceleration_limit); // Distance to cruise, position units.
        const float full_decel_distance = velocity_limit * velocity_limit /
                                          (2 * deceleration_limit); // Distance from cruise to rest, position units.
        float peak_speed = velocity_limit; // Peak reference speed magnitude, position units/s.
        float cruise_time = 0; // Duration at constant speed, s.
        if (distance < full_accel_distance + full_decel_distance) {
            const float peak_squared = (2 * acceleration_limit * deceleration_limit * distance +
                                        deceleration_limit * initial_speed * initial_speed) /
                                       (acceleration_limit + deceleration_limit);
            peak_speed = std::sqrt(std::max(0.0f, peak_squared));
        } else if (velocity_limit > 0) {
            cruise_time = (distance - full_accel_distance - full_decel_distance) / velocity_limit;
        }
        append_phase((peak_speed - initial_speed) / acceleration_limit, direction * acceleration_limit);
        append_phase(cruise_time, 0);
        append_phase(peak_speed / deceleration_limit, -direction * deceleration_limit);
        if (!std::isfinite(finish_time) || !std::isfinite(position) ||
            !std::isfinite(velocity) || finish_time < 0) return false;
        elapsed = 0;
        reference = measured_position;
        this->velocity = measured_velocity;
        return true;
    }

public:
    float goal = 0, reference = 0, velocity = 0, elapsed = 0, finish_time = 0;
    PolyTrajectory(float speed = 0, float acceleration = 0, float deceleration = 0) noexcept
        : velocity_limit(speed), acceleration_limit(acceleration), deceleration_limit(deceleration) {}

    /** Parameters apply to the next start/retarget; the current phases remain intact. */
    bool configure(float speed, float acceleration, float deceleration) {
        if (!(speed > 0 && acceleration > 0 && deceleration > 0) ||
            !std::isfinite(speed) || !std::isfinite(acceleration) || !std::isfinite(deceleration)) return false;
        velocity_limit = speed;
        acceleration_limit = acceleration;
        deceleration_limit = deceleration;
        return true;
    }

    /** Plan from the supplied motion, braking/reversing as required to reach rest at target. */
    bool start(TrajectoryState initial, float target) override {
        if (!(velocity_limit > 0 && acceleration_limit > 0 && deceleration_limit > 0) ||
            !std::isfinite(velocity_limit) || !std::isfinite(acceleration_limit) ||
            !std::isfinite(deceleration_limit) || !std::isfinite(target) ||
            !std::isfinite(initial.position) || !std::isfinite(initial.velocity)) return false;
        PolyTrajectory next(velocity_limit, acceleration_limit, deceleration_limit);
        next.goal = target;
        if (!next.plan(initial.position, initial.velocity)) return false;
        next.time_error = 0;
        next.initialized = true;
        *this = next;
        return true;
    }
    bool retarget(TrajectoryState initial, float target) override {
        return initialized && start(initial, target);
    }
    /** Advance to the first scheduled tick; step adds the future reference horizon once. */
    void on_update(const TrajectoryGenerator*, float delay) override {
        assert(initialized && delay >= 0 && std::isfinite(delay));
        sample_at(delay);
    }
    /** Evaluate at an absolute elapsed time without accumulating timestep error. */
    float sample_at(float time) {
        assert(initialized && time >= 0 && std::isfinite(time));
        time_error = 0;
        elapsed = std::min(time, finish_time);
        if (elapsed >= finish_time) {
            reference = goal;
            velocity = 0;
        } else {
            float phase_time = elapsed; // Seconds elapsed within the current phase.
            for (const Phase& phase : phases) {
                if (phase_time < phase.duration) {
                    reference = phase.position + phase.velocity * phase_time +
                                0.5f * phase.acceleration * phase_time * phase_time;
                    velocity = phase.velocity + phase.acceleration * phase_time;
                    break;
                }
                phase_time -= phase.duration;
            }
        }
        return reference;
    }
    /** Advance profile time with compensated summation to avoid long-run float drift. */
    float step(float dt) override {
        assert(dt > 0 && std::isfinite(dt));
        const float increment = dt - time_error;
        const float next_time = elapsed + increment;
        const float residual = (next_time - elapsed) - increment;
        const float value = sample_at(next_time);
        time_error = elapsed < finish_time ? residual : 0;
        return value;
    }
    void reset() override {
        initialized = false;
        time_error = 0;
        for (auto& phase : phases) phase = {};
        goal = reference = velocity = elapsed = finish_time = 0;
    }
    TrajectoryState get_state() const override { return {reference, velocity}; }
};
