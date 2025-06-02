#pragma once

#include <utility>

#ifdef ARM_MATH_CM4
#include <arm_math.h>
#else
#include <math.h>
#endif

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
    PIDConfig config;
public:
    // expect copy-elision
    explicit PIDRegulator(const PIDConfig&& config) : config(std::move(config)) {}

    float regulation(float error, float dt, bool zero_in_threshold=true) {
        if (zero_in_threshold && fabs(error) <= config.tolerance) {
            signal = 0;
            integral_error = 0;
            return 0;
        }

        float error_diff = error - prev_error;
        prev_error = error;

        if (isnan(integral_error)) {
            integral_error = 0;
            // TODO: warn?
        }
        double new_integral_error = integral_error + error * dt;
        if (fabs(new_integral_error) > config.integral_error_lim) {
            new_integral_error = copysign(config.integral_error_lim, integral_error);
        }
        integral_error = new_integral_error;

        signal = config.multiplier * (
             config.p_gain * error + config.i_gain * integral_error +
             config.d_gain * error_diff / dt
         );

        return signal;
    }

    void update_config(PIDConfig&& new_config) {
        config = std::move(new_config);
    }
    PIDConfig get_config() {
        return config;
    }
};
