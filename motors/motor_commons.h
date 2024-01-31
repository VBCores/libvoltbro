#pragma once

#include "encoders/generic.h"
#include "dsp/low_pass_filter.hpp"

struct CommonDriverConfig {
    uint8_t ppairs = 0;
    uint8_t gear_ratio = 1;
};

float calculate_angles(
    const CommonDriverConfig& drive,
    const LowPassFilter& filter,
    GenericEncoder& speed_encoder
);
float calculate_speed(
    const CommonDriverConfig& drive,
    const LowPassFilter& filter,
    float dt
);
