#pragma once


class LowPassFilter {
public:
    const float beta;
    LowPassFilter(float beta = 0.5): beta(beta) {};

    float operator () (float prev_value, float new_value) const {
        return (float)((1.0f - beta) * prev_value + beta * new_value);
    }
};