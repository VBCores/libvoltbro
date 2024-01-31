#pragma once

#include "arm_math.h"

class LowPassFilter {
public:
    const float beta;
    LowPassFilter(float beta = 0.5): beta(beta) {};

    float operator () (float prev_value, float new_value) const {
        if (is_close(beta, 1)) {
            return new_value;
        }
        return (float)((1.0f - beta) * prev_value + beta * new_value);
    }
};