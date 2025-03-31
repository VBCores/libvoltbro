#pragma once
#if defined(STM32G474xx) || defined(STM32_G)

#include "stm32g4xx_hal.h"

#include "voltbro/encoders/generic.h"
#include "voltbro/utils.hpp"

#ifndef ARM_MATH_CM4
constexpr float PI = 3.14159265359;
#endif

struct CommonDriverConfig {
    const uint8_t ppairs = 1;
    const uint8_t gear_ratio = 1;
};

/**
 * Abstract base class for all motor APIs
 */
class AbstractMotor {
protected:
    float current_limit = 0;  // Real current limit for operation
    float user_current_limit = 0;  // Limit set by user. Can be superseded by motor parameters - stall current, etc.
public:
    virtual HAL_StatusTypeDef init() = 0;
    virtual HAL_StatusTypeDef stop() = 0;
    virtual HAL_StatusTypeDef start() = 0;
    virtual HAL_StatusTypeDef set_state(bool) = 0;
    virtual void update() = 0;
};

inline float calculate_angle_simple(
    const CommonDriverConfig& drive,
    GenericEncoder& encoder
) {
    float current_circle_part = (float)encoder.get_value() / (float)encoder.CPR;

    if (encoder.is_electrical) {
        float circle_part_per_revolution = 1.0f / drive.ppairs;
        int32_t revolutions = encoder.get_revolutions() % (int16_t)drive.ppairs;
        if (revolutions < 0) {
            revolutions += drive.ppairs;
        }
        current_circle_part = (revolutions + current_circle_part) * circle_part_per_revolution;
    }

    return current_circle_part * pi2;
}

#endif
