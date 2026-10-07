#pragma once
#include <cmath>
#include <cstdint>
#include <optional>
#include <variant>
#include "voltbro/motors/trajectories/filter.hpp"
#include "voltbro/motors/trajectories/poly.hpp"
#include "voltbro/motors/trajectories/ramp.hpp"

enum ServoControlType : uint8_t {
        VELOCITY_DIRECT = 0, VELOCITY_RAMP = 1, TORQUE_DIRECT = 2,
        POSITION_DIRECT = 3, POSITION_FILTER = 4, POSITION_POLY = 5,
        VOLTAGE_DIRECT = 6
    };

struct ServoInputConfig {
    float input_bandwidth = 0; // Position filter bandwidth, 1/s.
    float velocity_limit = 0; // Position trajectory cruise speed, output rad/s.
    float acceleration_limit = 0; // Position trajectory acceleration, output rad/s^2.
    float deceleration_limit = 0; // Position trajectory deceleration, output rad/s^2.
    float velocity_ramp_rate = 0; // Velocity command slew rate, output rad/s^2.
    float velocity_planning_tolerance = .5f; // Allowed reference/measured speed difference, output rad/s.
};

struct ServoCommand {
    uint8_t type;
    float value;
    bool has_index;
    uint8_t index;
};

using ServoTrajectoryStorage = std::variant<FilterTrajectory, PolyTrajectory, RampTrajectory>;

inline TrajectoryGenerator& get_trajectory(ServoTrajectoryStorage& storage) {
    return std::visit([](auto& generator) -> TrajectoryGenerator& { return generator; }, storage);
}

inline std::optional<ServoTrajectoryStorage> make_servo_trajectory(uint8_t type, const ServoInputConfig& config) {
    switch (type) {
        case POSITION_FILTER:
            return std::optional<ServoTrajectoryStorage>(std::in_place, std::in_place_type<FilterTrajectory>, config.input_bandwidth);
        case POSITION_POLY:
            return std::optional<ServoTrajectoryStorage>(std::in_place, std::in_place_type<PolyTrajectory>, config.velocity_limit,
                                          config.acceleration_limit, config.deceleration_limit);
        case VELOCITY_RAMP:
            return std::optional<ServoTrajectoryStorage>(std::in_place, std::in_place_type<RampTrajectory>, config.velocity_ramp_rate);
        default: return std::nullopt;
    }
}
