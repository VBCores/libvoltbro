#pragma once

struct PIDConfig {
    double p_gain;
    double i_gain;
    double d_gain;
    double integral_error_lim;
    double tolerance;
};

class PIDRegulator  {
private:
    double signal = 0;
    double integral_error = 0;
    double prev_error = 0;
    const PIDConfig config;
public:
    // expect copy-elision
    explicit PIDRegulator(const PIDConfig&& config) : config(config) {}

    double regulation(double error, double dt);
};