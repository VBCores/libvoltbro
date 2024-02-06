#pragma once

struct PIDConfig {
    float multiplier = 1.0;
    float p_gain;
    float i_gain;
    float d_gain;
    float integral_error_lim;
    float tolerance;
};

class PIDRegulator  {
private:
    float signal = 0.0f;
    float integral_error = 0.0f;
    float prev_error = 0.0f;
    const PIDConfig config;
public:
    // expect copy-elision
    explicit PIDRegulator(const PIDConfig&& config) : config(config) {}

    float regulation(float error, float dt);
};