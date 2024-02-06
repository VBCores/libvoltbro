#pragma once
#if defined(STM32G474xx) || defined(STM32_G)

#include "encoders/generic.h"
#include "dsp/low_pass_filter.hpp"

struct CommonDriverConfig {
    uint8_t ppairs = 0;
    uint8_t gear_ratio = 1;
};

class AbstractMotor {
protected:
    const LowPassFilter angle_filter;
    const LowPassFilter speed_filter;
public:
    AbstractMotor(
        float angle_filter = 1,
        float speed_filter = 1
    ):
        angle_filter(angle_filter),
        speed_filter(speed_filter)
    {}
    float calculate_angle(
        const CommonDriverConfig& drive,
        GenericEncoder& speed_encoder
    ) const;
    float calculate_speed(const float shaft_angle, const float dt) const;

    virtual void regulate(float dt) = 0;
    virtual void update_angle(GenericEncoder& speed_encoder) = 0;
    virtual void update_speed(const float dt) = 0;
};

#endif
